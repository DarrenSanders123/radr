#include "update.h"

#include <LittleFS.h>
#include <WiFi.h>
#include <esp_flash.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>

#include <esp_heap_caps.h>

#include "FirmwareUpdateRuntime.h"
#include "constants/Version.h"
#include "services/ble_lifecycle.h"
#include "state/remote.h"

#ifndef FIRMWARE_BUILD_SHA
#define FIRMWARE_BUILD_SHA "unknown"
#endif

#ifndef FIRMWARE_TRACK
#define FIRMWARE_TRACK "main"
#endif

// Largest free internal block the HTTPS check needs once BLE is down.
// Measured 2026-09-08: 32,756 B largest / 127 KB free -> check succeeds.
// Override with -D RADR_OTA_MIN_LARGEST_BLOCK.
#ifndef RADR_OTA_MIN_LARGEST_BLOCK
#define RADR_OTA_MIN_LARGEST_BLOCK (28 * 1024)
#endif

// Fleet builds take their updates from the builder's own server.
#ifdef FLEET_SERVER
#define RADR_UPDATE_SERVER FLEET_SERVER
#define RADR_UNREACHABLE_TEXT "Could not reach your dashboard."
#else
#define RADR_UPDATE_SERVER RAD_SERVER
#define RADR_UNREACHABLE_TEXT "Could not reach the update server."
#endif

namespace {

constexpr const char *UPDATE_TAG = "UPDATE";
firmware::Decision pendingDecision;
bool decisionReady = false;

std::string stableDeviceId() {
    char identifier[17] = {};
    snprintf(identifier, sizeof(identifier), "%016llx",
             static_cast<unsigned long long>(ESP.getEfuseMac()));
    return identifier;
}

std::uint32_t physicalFlashSize() {
    std::uint32_t size = 0;
    return esp_flash_get_physical_size(nullptr, &size) == ESP_OK
               ? size
               : ESP.getFlashChipSize();
}

std::string runningFirmwareHash() {
    const esp_partition_t *running = esp_ota_get_running_partition();
    unsigned char digest[32] = {};
    if (running == nullptr ||
        esp_partition_get_sha256(running, digest) != ESP_OK) {
        return {};
    }
    return firmware::sha256Hex(digest);
}

firmware::DeviceReport makeDeviceReport() {
    firmware::DeviceReport report;
    report.deviceType = "radr";
    report.deviceId = stableDeviceId();
    report.reportedTrack = FIRMWARE_TRACK;
    report.currentVersion = VERSION;
    report.currentBuild = FIRMWARE_BUILD_SHA;
    report.firmwareHash = runningFirmwareHash();
    report.chip = ESP.getChipModel();
    report.chipRevision = ESP.getChipRevision();
    report.chipCores = ESP.getChipCores();
    report.hardwareRevision = "radr-v1";
    report.flashSizeBytes = physicalFlashSize();
    const esp_partition_t *nextUpdatePartition =
        esp_ota_get_next_update_partition(nullptr);
    report.otaSlotSizeBytes =
        nextUpdatePartition == nullptr ? 0 : nextUpdatePartition->size;
    report.psramSizeBytes = ESP.getPsramSize();
    report.partitionLayout = "radr-ota-v1";
    firmware::provenance::reconcile(report);
    return report;
}

bool validateInstallPlan(const firmware::Decision &decision, String &error) {
    bool sawFilesystem = false;
    bool sawApplication = false;
    for (std::size_t index = 0; index < decision.artifactCount; ++index) {
        const auto &artifact = decision.artifacts[index];
        if (artifact.role == "filesystem") {
            if (sawFilesystem || sawApplication) {
                error = "filesystem must be installed once and before application";
                return false;
            }
            sawFilesystem = true;
        } else if (artifact.role == "application") {
            if (sawApplication) {
                error = "application artifact appears more than once";
                return false;
            }
            sawApplication = true;
        } else if (firmware::isMetadataArtifactRole(artifact.role)) {
            continue;
        } else {
            error = ("unsupported RADR artifact role: " + artifact.role).c_str();
            return false;
        }
    }
    if (!sawApplication) {
        error = "release has no application artifact";
        return false;
    }
    return true;
}

const firmware::Artifact *artifactForRole(const char *role) {
    if (!decisionReady) return nullptr;
    for (std::size_t index = 0; index < pendingDecision.artifactCount; ++index) {
        if (pendingDecision.artifacts[index].role == role) {
            return &pendingDecision.artifacts[index];
        }
    }
    return nullptr;
}

void finishTask() {
    stateMachine->process_event(done{});
    vTaskDelete(nullptr);
}

}  // namespace

TaskHandle_t updateTaskHandle = nullptr;
TaskHandle_t updateFilesystemTaskHandle = nullptr;
TaskHandle_t updateSoftwareTaskHandle = nullptr;

bool isSoftwareUpdateAvailable = false;
bool isFilesystemUpdateAvailable = false;
String updateFailureReason;

bool confirmRunningFirmware() {
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == nullptr) return false;

    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    const esp_err_t stateResult = esp_ota_get_state_partition(running, &state);
    if (stateResult == ESP_ERR_NOT_SUPPORTED ||
        (stateResult == ESP_OK && state != ESP_OTA_IMG_PENDING_VERIFY)) {
        return true;
    }
    if (stateResult != ESP_OK) {
        ESP_LOGW(UPDATE_TAG, "Unable to read running OTA state: %s",
                 esp_err_to_name(stateResult));
        return false;
    }

    const esp_err_t confirmResult = esp_ota_mark_app_valid_cancel_rollback();
    if (confirmResult != ESP_OK) {
        ESP_LOGE(UPDATE_TAG, "Unable to confirm running firmware: %s",
                 esp_err_to_name(confirmResult));
        return false;
    }
    ESP_LOGI(UPDATE_TAG, "Confirmed running OTA image");
    return true;
}

bool isUpdateAvailable() {
    if (WiFi.status() != WL_CONNECTED) {
        updateFailureReason = "Wi-Fi is not connected.";
        return false;
    }

    firmware::Decision decision;
    String error;
    const auto report = makeDeviceReport();
    if (!firmware::postCheck(RADR_UPDATE_SERVER, report, decision, error)) {
        ESP_LOGE(UPDATE_TAG, "Firmware resolver failed: %s", error.c_str());
        updateFailureReason = RADR_UNREACHABLE_TEXT;
        return false;
    }
    firmware::provenance::observeCurrent(report, decision);
    firmware::provenance::stageUpdate(report, decision);
    ESP_LOGI(UPDATE_TAG,
             "Resolver assigned track=%s update=%s target=%s next=%s",
             decision.assignedTrack.c_str(),
             decision.shouldUpdate ? "true" : "false",
             decision.targetVersion.c_str(), decision.nextHopVersion.c_str());
    if (!decision.shouldUpdate) return false;  // up to date, no reason
    if (!validateInstallPlan(decision, error)) {
        ESP_LOGE(UPDATE_TAG, "Invalid install plan: %s", error.c_str());
        updateFailureReason = "The update server sent an invalid plan.";
        return false;
    }

    pendingDecision = decision;
    decisionReady = true;
    isFilesystemUpdateAvailable = artifactForRole("filesystem") != nullptr;
    isSoftwareUpdateAvailable = artifactForRole("application") != nullptr;
    return isSoftwareUpdateAvailable;
}

// The whole self-update runs with BLE torn down: NimBLE plus the RAD BLE
// service leave far too little contiguous internal RAM for a TLS handshake.
// Every exit of the update flow ends in a restart, which brings BLE back.
void updateTask(void *pvParameters) {
    isFilesystemUpdateAvailable = false;
    isSoftwareUpdateAvailable = false;
    decisionReady = false;
    updateFailureReason = "";

    const BleShutdownResult ble = shutdownBleForNetwork();
    if (!ble.deinitOk || ble.largestInternal < RADR_OTA_MIN_LARGEST_BLOCK) {
        ESP_LOGE(UPDATE_TAG,
                 "Not enough memory for the update check after BLE shutdown: "
                 "largest=%u need=%u deinit=%d",
                 (unsigned)ble.largestInternal,
                 (unsigned)RADR_OTA_MIN_LARGEST_BLOCK, ble.deinitOk ? 1 : 0);
        updateFailureReason = "Not enough memory to check for updates.";
        finishTask();
        return;
    }

    isUpdateAvailable();
    finishTask();
}

void updateFilesystemTask(void *pvParameters) {
    const firmware::Artifact *artifact = artifactForRole("filesystem");
    if (WiFi.status() != WL_CONNECTED || artifact == nullptr) {
        ESP_LOGE(UPDATE_TAG, "Filesystem update is no longer available");
        updateFailureReason = "The filesystem update is no longer available.";
        isFilesystemUpdateAvailable = false;
        isSoftwareUpdateAvailable = false;
        finishTask();
        return;
    }

    LittleFS.end();
    String error;
    const bool installed =
        firmware::installStreamedArtifact(*artifact, U_SPIFFS, error);
    if (!LittleFS.begin()) {
        ESP_LOGE(UPDATE_TAG, "Failed to remount LittleFS after update");
    }
    if (!installed) {
        ESP_LOGE(UPDATE_TAG, "Filesystem update failed: %s", error.c_str());
        updateFailureReason = "The filesystem update failed to install.";
        // Do not install the application when the ordered filesystem step did
        // not verify. The existing bootable application remains selected.
        isSoftwareUpdateAvailable = false;
    }
    isFilesystemUpdateAvailable = false;
    finishTask();
}

void updateSoftwareTask(void *pvParameters) {
    const firmware::Artifact *artifact = artifactForRole("application");
    if (WiFi.status() != WL_CONNECTED || artifact == nullptr) {
        ESP_LOGE(UPDATE_TAG, "Application update is no longer available");
        updateFailureReason = "The application update is no longer available.";
        isSoftwareUpdateAvailable = false;
        finishTask();
        return;
    }

    String error;
    if (!firmware::installStreamedArtifact(*artifact, U_FLASH, error)) {
        ESP_LOGE(UPDATE_TAG, "Application update failed: %s", error.c_str());
        updateFailureReason = "The application update failed to install.";
        isSoftwareUpdateAvailable = false;
        finishTask();
        return;
    }

    ESP_LOGI(UPDATE_TAG, "Verified application installed; restarting");
    esp_restart();
}

#ifdef FLEET_SERVER
namespace {

// Installs the update the fleet server offers, straight away and without the
// update screens. Used by the fleet check-in at start-up, before Bluetooth is
// initialised, so there is plenty of memory for TLS. Does not return when the
// update installs: the remote restarts on the new firmware.
bool fleetInstallUpdateImpl(String &detail) {
    if (WiFi.status() != WL_CONNECTED) {
        detail = "Wi-Fi is not connected";
        return false;
    }
    firmware::Decision decision;
    String error;
    const auto report = makeDeviceReport();
    if (!firmware::postCheck(RADR_UPDATE_SERVER, report, decision, error)) {
        detail = "check-failed: " + error;
        return false;
    }
    firmware::provenance::observeCurrent(report, decision);
    firmware::provenance::stageUpdate(report, decision);
    if (!decision.shouldUpdate) {
        detail = "server offered no update for this remote";
        return false;
    }
    if (!validateInstallPlan(decision, error)) {
        detail = "invalid install plan: " + error;
        return false;
    }

    firmware::installProgressHook = [](std::uint32_t done, std::uint32_t total) {
        static std::uint32_t lastFifth = 99;
        const std::uint32_t fifth = total == 0 ? 0 : (done * 5ULL) / total;
        if (done == 0 || fifth != lastFifth) {
            lastFifth = fifth;
            ESP_LOGW(UPDATE_TAG, "Download %u%%: %u of %u bytes",
                     static_cast<unsigned>(fifth * 20),
                     static_cast<unsigned>(done), static_cast<unsigned>(total));
        }
    };

    // The device registry goes first, then the application. The registry
    // partition stays unmounted until the end: mounting it in between would
    // take memory the next download needs.
    bool registryInstalled = false;
    for (std::size_t index = 0; index < decision.artifactCount; ++index) {
        const firmware::Artifact &artifact = decision.artifacts[index];
        if (artifact.role != "filesystem" && artifact.role != "application") {
            continue;
        }
        ESP_LOGW(UPDATE_TAG, "Installing %s: %u bytes (free=%u largest=%u)",
                 artifact.role.c_str(), static_cast<unsigned>(artifact.sizeBytes),
                 static_cast<unsigned>(heap_caps_get_free_size(
                     MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
                 static_cast<unsigned>(heap_caps_get_largest_free_block(
                     MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
        if (artifact.role == "filesystem") {
            LittleFS.end();
            const bool installed =
                firmware::installStreamedArtifact(artifact, U_SPIFFS, error);
            if (!installed) {
                LittleFS.begin();
                detail = "registry install failed: " + error;
                return false;
            }
            registryInstalled = true;
            vTaskDelay(pdMS_TO_TICKS(500));  // let the connection's memory settle
        } else {
            if (!firmware::installStreamedArtifact(artifact, U_FLASH, error)) {
                if (registryInstalled) LittleFS.begin();
                detail = "install-failed: " + error;
                return false;
            }
            ESP_LOGW(UPDATE_TAG, "Verified application installed; restarting");
            esp_restart();
        }
    }
    if (registryInstalled) LittleFS.begin();
    detail = "release has no application artifact";
    return false;
}

struct FleetInstallJob {
    String *detail = nullptr;
    bool ok = false;
    volatile bool done = false;
};

void fleetInstallTask(void *argument) {
    auto *job = static_cast<FleetInstallJob *>(argument);
    job->ok = fleetInstallUpdateImpl(*job->detail);
    job->done = true;
    vTaskDelete(nullptr);
}

}  // namespace

// The install runs in a task of its own: the TLS handshake and the download
// buffers need far more stack than the Arduino main task has, and running them
// there crashed the remote.
bool fleetInstallUpdate(String &detail) {
    static FleetInstallJob job;
    job.detail = &detail;
    job.ok = false;
    job.done = false;
    // Idle priority so a long TLS handshake cannot starve the idle task.
    if (xTaskCreatePinnedToCore(fleetInstallTask, "fleetInstall", 16384, &job,
                                tskIDLE_PRIORITY, nullptr, 0) != pdPASS) {
        detail = "could not start the install task (not enough memory)";
        return false;
    }
    while (!job.done) vTaskDelay(pdMS_TO_TICKS(200));
    return job.ok;
}
#endif
