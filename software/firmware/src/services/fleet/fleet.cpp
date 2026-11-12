#include "fleet.h"

#if defined(FLEET_SERVER) && defined(FLEET_KEY)

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_crt_bundle.h>
#include <esp_heap_caps.h>
#include <esp_http_client.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_rom_uart.h>
#include <esp_system.h>
#include <esp_timer.h>

#include <vector>

#include "constants/Version.h"
#include "tasks/update.h"
#include "utils/psramTask.h"

#ifndef FIRMWARE_BUILD_SHA
#define FIRMWARE_BUILD_SHA "unknown"
#endif
#ifndef FIRMWARE_TRACK
#define FIRMWARE_TRACK "main"
#endif

#if defined(FIRMWARE_USE_IDF_CRT_BUNDLE)
extern "C" esp_err_t esp_crt_bundle_attach(void *configuration);
#define FLEET_CRT_BUNDLE_ATTACH esp_crt_bundle_attach
#else
extern "C" esp_err_t arduino_esp_crt_bundle_attach(void *configuration);
#define FLEET_CRT_BUNDLE_ATTACH arduino_esp_crt_bundle_attach
#endif

namespace {

const char *const TAG = "FLEET";

constexpr uint32_t WIFI_WAIT_MS = 6000;
constexpr uint32_t HTTP_TIMEOUT_MS = 6000;
constexpr uint32_t TASK_WAIT_MS = 25000;
constexpr uint32_t FAILURES_BEFORE_BACKOFF = 3;
constexpr uint32_t BACKOFF_EVERY = 5;
constexpr uint32_t CRASHES_FOR_SAFE_MODE = 3;

// ---------------------------------------------------------------------------
// State that survives a restart or a crash (but not a power cut), kept in RTC
// memory: how often in a row the remote crashed, the log lines of the last
// session, and the outcome of a command that restarted the remote.
// ---------------------------------------------------------------------------
constexpr uint32_t RTC_MAGIC = 0xF1EE7AD2;
constexpr uint32_t CHECK_AT_START = 0xC4EC4A7D;
constexpr uint32_t ASK_MACHINE_UPDATE = 0xA5C0DA7E;
constexpr size_t LOG_RING_BYTES = 3072;

RTC_NOINIT_ATTR uint32_t rtcMagic;
RTC_NOINIT_ATTR uint32_t crashStreak;
RTC_NOINIT_ATTR uint32_t logHead;    // next byte to write
RTC_NOINIT_ATTR uint32_t logLength;  // bytes in the ring
RTC_NOINIT_ATTR char logRing[LOG_RING_BYTES];
RTC_NOINIT_ATTR uint32_t checkAtStartMagic;  // set by the menu's update entry
RTC_NOINIT_ATTR uint32_t askMachineMagic;    // set when a machine update is requested
RTC_NOINIT_ATTR char askMachineAddress[20];
RTC_NOINIT_ATTR int32_t pendingResultId;  // 0 = none
RTC_NOINIT_ATTR uint32_t pendingResultOk;
RTC_NOINIT_ATTR char pendingResultDetail[80];

portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t hookCalls = 0;      // lines the log system handed to the hook
volatile uint32_t hookKept = 0;       // of those, how many were warnings/errors
char hookSample[48] = {};             // the first line seen, to show its format
uint32_t ringBytesAtStart = 0;        // what the last session left in the ring
vprintf_like_t previousVprintf = nullptr;

esp_reset_reason_t resetReason = ESP_RST_UNKNOWN;
bool safeMode = false;

const char *resetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON: return "power on";
        case ESP_RST_SW: return "restart";
        case ESP_RST_PANIC: return "crash";
        case ESP_RST_INT_WDT: return "watchdog (interrupt)";
        case ESP_RST_TASK_WDT: return "watchdog (task)";
        case ESP_RST_WDT: return "watchdog";
        case ESP_RST_BROWNOUT: return "power dip (brownout)";
        case ESP_RST_DEEPSLEEP: return "deep sleep";
        case ESP_RST_EXT: return "reset pin";
        default: return "unknown";
    }
}

void evaluateLastReset() {
    ringBytesAtStart = logLength;  // before anything is checked or cleared
    resetReason = esp_reset_reason();
    const bool crashed =
        resetReason == ESP_RST_PANIC || resetReason == ESP_RST_INT_WDT ||
        resetReason == ESP_RST_TASK_WDT || resetReason == ESP_RST_WDT ||
        resetReason == ESP_RST_BROWNOUT;
    if (rtcMagic != RTC_MAGIC || logLength > LOG_RING_BYTES ||
        logHead >= LOG_RING_BYTES) {
        rtcMagic = RTC_MAGIC;
        crashStreak = 0;
        logHead = 0;
        logLength = 0;
        pendingResultId = 0;
        checkAtStartMagic = 0;
        askMachineMagic = 0;
    }
    askMachineAddress[sizeof(askMachineAddress) - 1] = '\0';
    pendingResultDetail[sizeof(pendingResultDetail) - 1] = '\0';
    crashStreak = crashed ? crashStreak + 1 : 0;
    safeMode = crashStreak >= CRASHES_FOR_SAFE_MODE;
}

// ---------------------------------------------------------------------------
// Log capture. Warnings and errors are copied into a ring in RTC memory on
// their way to the serial port. When the ring is full the oldest text is
// overwritten: what led up to a crash matters more than what came before.
// ---------------------------------------------------------------------------
void ringWrite(const char *data, size_t length) {
    portENTER_CRITICAL_SAFE(&logMux);
    for (size_t index = 0; index < length; index++) {
        const char character = data[index];
        if (character == '\r') continue;
        logRing[logHead] = character;
        logHead = (logHead + 1) % LOG_RING_BYTES;
        if (logLength < LOG_RING_BYTES) logLength++;
    }
    portEXIT_CRITICAL_SAFE(&logMux);
}

const char *skipColor(const char *line) {
    if (line[0] == '\033' && line[1] == '[') {
        const char *end = strchr(line, 'm');
        if (end != nullptr) return end + 1;
    }
    return line;
}

bool isWarningOrError(const char *line) {
    const char *text = skipColor(line);
    return ((text[0] == 'E' || text[0] == 'W') && text[1] == ' ') ||
           strstr(text, "][E]") != nullptr || strstr(text, "][W]") != nullptr;
}

// Application code (and the Arduino core) prints through the ROM's ets_printf,
// one character at a time, not through the log system's vprintf. This keeps
// whole lines and passes them on to the ring and to the serial port.
char romLine[200];
size_t romLineLength = 0;

void IRAM_ATTR romPutc(char character) {
    esp_rom_output_tx_one_char(static_cast<uint8_t>(character));
    if (character == '\r') return;
    if (romLineLength < sizeof(romLine) - 1) romLine[romLineLength++] = character;
    if (character == '\n' || romLineLength >= sizeof(romLine) - 1) {
        romLine[romLineLength] = '\0';
        if (isWarningOrError(romLine)) ringWrite(romLine, romLineLength);
        romLineLength = 0;
    }
}

// Skips a leading terminal colour code, which IDF adds when colours are on.
int captureVprintf(const char *format, va_list arguments) {
    char line[200];
    va_list copy;
    va_copy(copy, arguments);
    const int written = vsnprintf(line, sizeof(line), format, copy);
    va_end(copy);
    hookCalls = hookCalls + 1;
    if (hookSample[0] == '\0' && written > 0) {
        strncpy(hookSample, line, sizeof(hookSample) - 1);
    }
    if (written > 0 && isWarningOrError(line)) {
        hookKept = hookKept + 1;
        ringWrite(line, min(static_cast<size_t>(written), sizeof(line) - 1));
    }
    return previousVprintf != nullptr ? previousVprintf(format, arguments)
                                      : vprintf(format, arguments);
}

// Copies the ring out as one text, oldest first, without clearing it.
String ringSnapshot() {
    String text;
    portENTER_CRITICAL(&logMux);
    const uint32_t length = logLength;
    const uint32_t start = (logHead + LOG_RING_BYTES - length) % LOG_RING_BYTES;
    text.reserve(length);
    for (uint32_t index = 0; index < length; index++) {
        text += logRing[(start + index) % LOG_RING_BYTES];
    }
    portEXIT_CRITICAL(&logMux);
    return text;
}

void ringClear() {
    portENTER_CRITICAL(&logMux);
    logHead = 0;
    logLength = 0;
    portEXIT_CRITICAL(&logMux);
}

// ---------------------------------------------------------------------------
// Identity: the remote creates its own random key once and keeps it. The
// server remembers it the first time it sees the remote, so nobody else can
// later connect under this remote's address.
// ---------------------------------------------------------------------------
String machineKey;
volatile bool fleetReady = false;

String deviceAddress() {
    const uint64_t mac = ESP.getEfuseMac();
    char text[18];
    snprintf(text, sizeof(text), "%02X:%02X:%02X:%02X:%02X:%02X",
             unsigned(mac & 255), unsigned((mac >> 8) & 255),
             unsigned((mac >> 16) & 255), unsigned((mac >> 24) & 255),
             unsigned((mac >> 32) & 255), unsigned((mac >> 40) & 255));
    return text;
}

void loadMachineKey() {
    Preferences preferences;
    if (!preferences.begin("fleet", false)) return;
    if (preferences.isKey("secret")) {
        machineKey = preferences.getString("secret", "");
    }
    if (machineKey.length() != 64) {
        uint8_t raw[32];
        esp_fill_random(raw, sizeof(raw));  // Wi-Fi is running: true random
        char hex[65];
        for (size_t index = 0; index < sizeof(raw); index++) {
            snprintf(hex + index * 2, 3, "%02x", raw[index]);
        }
        machineKey = hex;
        preferences.putString("secret", machineKey);
    }
    preferences.end();
}

// ---------------------------------------------------------------------------
// One HTTPS POST of a JSON body. Runs inside the check-in task.
// ---------------------------------------------------------------------------
bool postJson(const String &url, const String &body, int &status,
              String &response, String &error) {
    esp_http_client_config_t config = {};
    config.url = url.c_str();
    config.method = HTTP_METHOD_POST;
    config.timeout_ms = HTTP_TIMEOUT_MS;
    config.buffer_size = 2048;
    config.buffer_size_tx = 2048;
    config.crt_bundle_attach = FLEET_CRT_BUNDLE_ATTACH;

    status = 0;
    response = "";
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == nullptr) {
        error = "could not start the HTTPS client";
        return false;
    }
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Accept-Encoding", "identity");
    esp_http_client_set_header(client, "X-Fleet-Key", FLEET_KEY);
    esp_http_client_set_header(client, "X-Fleet-Secret", machineKey.c_str());

    bool ok = false;
    const esp_err_t opened = esp_http_client_open(client, body.length());
    if (opened != ESP_OK) {
        error = String("connect failed: ") + esp_err_to_name(opened);
    } else if (esp_http_client_write(client, body.c_str(), body.length()) !=
               static_cast<int>(body.length())) {
        error = "write failed";
    } else {
        esp_http_client_fetch_headers(client);
        status = esp_http_client_get_status_code(client);
        char buffer[512];
        int read = 0;
        while ((read = esp_http_client_read(client, buffer, sizeof(buffer))) >
               0) {
            if (response.length() + read > 8192) {
                error = "reply too large";
                response = "";
                break;
            }
            response.concat(buffer, read);
        }
        ok = error.isEmpty();
    }
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return ok;
}

// ---------------------------------------------------------------------------
// The check-in
// ---------------------------------------------------------------------------
struct Command {
    int id = 0;
    String type;
};

struct CheckinResult {
    bool ok = false;
    String error;
    std::vector<Command> commands;
    // What the dashboard says about this remote (shown on its Dashboard page).
    String name;
    String channel;
    String latestVersion;
    bool updateAvailable = false;
    // The dashboard's answer to "update my machine".
    bool answeredMachineRequest = false;
    String machineRequestMessage;
};

CheckinResult checkinResult;
volatile bool checkinDone = false;

void buildReport(JsonDocument &report, const String &logs) {
    report["deviceId"] = deviceAddress();
    report["deviceType"] = "radr";
    report["version"] = VERSION;
    report["buildSha"] = FIRMWARE_BUILD_SHA;
    report["track"] = FIRMWARE_TRACK;
    report["chip"] = ESP.getChipModel();
    report["state"] = "boot";
    report["uptimeSeconds"] =
        static_cast<uint32_t>(esp_timer_get_time() / 1000000LL);
    report["resetReason"] = resetReasonName(resetReason);
    report["logRingBytes"] = ringBytesAtStart;
    report["logHookCalls"] = hookCalls;
    report["logHookKept"] = hookKept;
    report["logSample"] = hookSample;
    report["crashes"] = crashStreak;
    report["safeMode"] = safeMode;

    const uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    report["heapFree"] = heap_caps_get_free_size(caps);
    report["heapLargest"] = heap_caps_get_largest_free_block(caps);
    report["heapMin"] = heap_caps_get_minimum_free_size(caps);
    report["psramBytes"] = ESP.getPsramSize();

    if (WiFi.status() == WL_CONNECTED) {
        report["wifiSsid"] = WiFi.SSID();
        report["wifiRssi"] = WiFi.RSSI();
        report["wifiIp"] = WiFi.localIP().toString();
    }

    // The outcome of the command that restarted or failed last time.
    if (pendingResultId > 0) {
        JsonObject result = report["results"].to<JsonArray>().add<JsonObject>();
        result["id"] = pendingResultId;
        result["ok"] = pendingResultOk != 0;
        result["detail"] = pendingResultDetail;
    }

    // A machine update asked for before the last restart.
    if (askMachineMagic == ASK_MACHINE_UPDATE && askMachineAddress[0] != '\0') {
        JsonObject request = report["requests"].to<JsonArray>().add<JsonObject>();
        request["type"] = "updateOssm";
        request["mac"] = askMachineAddress;
    }

    if (!logs.isEmpty()) {
        JsonArray lines = report["logs"].to<JsonArray>();
        int start = 0;
        while (start < static_cast<int>(logs.length()) && lines.size() < 150) {
            int end = logs.indexOf('\n', start);
            if (end < 0) end = logs.length();
            if (end > start) lines.add(logs.substring(start, end));
            start = end + 1;
        }
    }
}

void checkinTask(void *) {
    const String logs = ringSnapshot();
    JsonDocument report;
    buildReport(report, logs);
    String body;
    serializeJson(report, body);

    int status = 0;
    String response;
    String error;
    if (!postJson(String(FLEET_SERVER) + "/api/fleet/v1/checkin", body, status,
                  response, error) ||
        status != 200) {
        checkinResult.error = status != 0 ? "HTTP " + String(status) : error;
    } else {
        JsonDocument reply;
        if (deserializeJson(reply, response)) {
            checkinResult.error = "reply was not valid JSON";
        } else {
            checkinResult.ok = true;
            JsonObjectConst dashboard = reply["dashboard"].as<JsonObjectConst>();
            checkinResult.name = dashboard["name"] | "";
            checkinResult.channel = dashboard["channel"] | "";
            checkinResult.latestVersion = dashboard["latestVersion"] | "";
            checkinResult.updateAvailable = dashboard["updateAvailable"] | false;
            JsonObjectConst answer = reply["requestResults"][0].as<JsonObjectConst>();
            if (!answer.isNull()) {
                checkinResult.answeredMachineRequest = true;
                checkinResult.machineRequestMessage = answer["message"] | "";
            }
            for (JsonObjectConst item : reply["commands"].as<JsonArrayConst>()) {
                Command command;
                command.id = item["id"] | 0;
                command.type = item["cmd"] | item["type"] | "";
                checkinResult.commands.push_back(command);
            }
        }
    }
    if (checkinResult.ok) {
        ringClear();  // the server has them now
        pendingResultId = 0;
    }
    checkinDone = true;
    vTaskDelete(nullptr);
}

void setPendingResult(int id, bool ok, const String &detail) {
    pendingResultId = id;
    pendingResultOk = ok ? 1 : 0;
    strncpy(pendingResultDetail, detail.c_str(),
            sizeof(pendingResultDetail) - 1);
    pendingResultDetail[sizeof(pendingResultDetail) - 1] = '\0';
}

void runCommand(const Command &command) {
    ESP_LOGW(TAG, "Remote command %d: %s", command.id, command.type.c_str());
    if (command.type == "restart") {
        // The result goes to the server at the next check-in, after the restart.
        setPendingResult(command.id, true, "");
        ESP.restart();
    } else if (command.type == "update") {
        String detail;
        // Does not return when the update installs: the remote restarts on
        // the new firmware and the server sees the new build.
        if (!fleetInstallUpdate(detail)) {
            ESP_LOGW(TAG, "Update failed: %s", detail.c_str());
            setPendingResult(command.id, false, detail);
        }
    } else {
        setPendingResult(command.id, false, "unknown command");
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// USB setup channel, used by the setup page in the browser. One JSON line in
// (prefixed FLEET?), one JSON line out (prefixed FLEET!). The serial identity
// reader hands over every line it does not recognise.
// ---------------------------------------------------------------------------
namespace {

String fromHex(const char *hex) {
    String text;
    const size_t length = strlen(hex);
    for (size_t index = 0; index + 1 < length; index += 2) {
        char pair[3] = {hex[index], hex[index + 1], 0};
        text += static_cast<char>(strtol(pair, nullptr, 16));
    }
    return text;
}

void replySerial(const JsonDocument &reply) {
    String json;
    serializeJson(reply, json);
    const String line = "\nFLEET!" + json + "\n";
    Serial.write(reinterpret_cast<const uint8_t *>(line.c_str()), line.length());
    Serial.flush();
}

void handleSerial(const String &json) {
    JsonDocument request;
    if (deserializeJson(request, json)) return;
    const String op = request["op"] | "";
    JsonDocument reply;
    reply["op"] = op;
    bool restartAfter = false;

    if (op == "info") {
        reply["fleet"] = 1;
        reply["deviceType"] = "radr";
        reply["mac"] = deviceAddress();
        reply["version"] = VERSION;
        reply["buildSha"] = FIRMWARE_BUILD_SHA;
        reply["ready"] = static_cast<bool>(fleetReady);
        if (fleetReady) reply["secret"] = machineKey;
        JsonObject wifi = reply["wifi"].to<JsonObject>();
        wifi["connected"] = WiFi.status() == WL_CONNECTED;
        if (WiFi.status() == WL_CONNECTED) {
            wifi["ssid"] = WiFi.SSID();
            wifi["ip"] = WiFi.localIP().toString();
        }
    } else if (!fleetReady) {
        reply["ok"] = false;
        reply["error"] = "starting";
    } else if (op == "scan") {
        JsonArray networks = reply["networks"].to<JsonArray>();
        const int found = WiFi.scanNetworks();
        for (int index = 0; index < found && networks.size() < 15; index++) {
            const String ssid = WiFi.SSID(index);
            bool seen = ssid.isEmpty();
            for (JsonVariantConst known : networks) {
                if (ssid == known.as<const char *>()) seen = true;
            }
            if (!seen) networks.add(ssid);
        }
        WiFi.scanDelete();
        reply["ok"] = true;
    } else if (op == "wifi") {
        // The reader only takes plain ASCII, so the page sends hex.
        const String ssid = fromHex(request["ssidHex"] | "");
        const String password = fromHex(request["passHex"] | "");
        bool ok = false;
        if (!ssid.isEmpty()) {
            WiFi.begin(ssid.c_str(), password.c_str());  // saved to flash
            for (int waited = 0; waited < 150 && WiFi.status() != WL_CONNECTED;
                 waited++) {
                delay(100);
            }
            ok = WiFi.status() == WL_CONNECTED;
        }
        reply["ok"] = ok;
        if (ok) reply["ip"] = WiFi.localIP().toString();
        // Restart so the check-in at start-up registers the remote right away.
        restartAfter = ok;
    } else {
        reply["ok"] = false;
        reply["error"] = "unknown";
    }
    replySerial(reply);
    if (restartAfter) {
        delay(1500);
        ESP.restart();
    }
}

// One command at a time, each in a short-lived task. A task that stayed
// resident would keep its stack for good, and the remote's update needs a
// large free block of memory once Bluetooth is shut down (see update.cpp).
volatile bool serialBusy = false;

void serialWorker(void *argument) {
    char *line = static_cast<char *>(argument);
    handleSerial(String(line));
    free(line);
    serialBusy = false;
    vTaskDelete(nullptr);
}

}  // namespace

bool fleetSerialCommand(const char *line) {
    if (strncmp(line, "FLEET?", 6) != 0) return false;
    if (serialBusy) return true;  // the page asks again if it got no reply
    char *copy = strdup(line + 6);
    if (copy == nullptr) return true;
    serialBusy = true;
    if (createInternalTask(serialWorker, "fleetSerial", 6144, copy, 1, nullptr,
                           0) != pdPASS) {
        free(copy);
        serialBusy = false;
    }
    return true;
}

void fleetRequestMachineUpdate(const char *machineAddress) {
    if (machineAddress == nullptr || machineAddress[0] == '\0') {
        // Nothing to ask about: say so on the Dashboard page, do not restart.
        Preferences state;
        if (state.begin("fleet", false)) {
            state.putString("dmach", "Could not read the machine's address.");
            state.end();
        }
        return;
    }
    strncpy(askMachineAddress, machineAddress, sizeof(askMachineAddress) - 1);
    askMachineAddress[sizeof(askMachineAddress) - 1] = '\0';
    askMachineMagic = ASK_MACHINE_UPDATE;
    xTaskCreate(
        [](void *) {
            vTaskDelay(pdMS_TO_TICKS(1500));
            ESP.restart();
        },
        "fleetRestart", 2048, nullptr, 1, nullptr);
}

void fleetRequestStartupCheck() {
    checkAtStartMagic = CHECK_AT_START;
    // Let the screen show the message before the restart.
    xTaskCreate(
        [](void *) {
            vTaskDelay(pdMS_TO_TICKS(1500));
            ESP.restart();
        },
        "fleetRestart", 2048, nullptr, 1, nullptr);
}

String fleetDashboardDescription() {
    String name, channel, latest, last, check, machine;
    bool available = false;
    Preferences state;
    if (state.begin("fleet", false)) {  // read-write: creates the namespace
        name = state.getString("dname", "");
        channel = state.getString("dchan", "");
        latest = state.getString("dlatest", "");
        last = state.getString("dlast", "");
        check = state.getString("dcheck", "");
        machine = state.getString("dmach", "");
        available = state.getBool("davail", false);
        state.end();
    }

    String text = String(FLEET_HOST) + "\n";
    if (last.isEmpty()) {
        text += "Not checked in yet.\nSet up WiFi first.";
        return text;
    }
    text += name.isEmpty() ? String("Not named yet") : "Name: " + name;
    text += "\nUpdates: " + (channel.isEmpty() ? String("manual") : channel);
    text += "\nLast check-in: " + last;
    if (!check.isEmpty()) text += "\nUpdate check: " + check;
    if (!machine.isEmpty()) text += "\nMachine: " + machine;
    if (available && !latest.isEmpty()) {
        text += "\nNew firmware " + latest + " is waiting.\nChoose Check for updates.";
    } else if (last == "OK") {
        text += "\nFirmware is up to date.";
    }
    return text;
}

void fleetEarlyInit() {
    evaluateLastReset();
    previousVprintf = esp_log_set_vprintf(captureVprintf);
    esp_rom_install_channel_putc(1, romPutc);
}

void fleetBootCheckin() {
    loadMachineKey();
    fleetReady = true;

    Preferences state;
    uint32_t failures = 0;
    uint32_t boots = 0;
    if (state.begin("fleet", false)) {
        failures = state.getUInt("fails", 0);
        boots = state.getUInt("boots", 0) + 1;
        state.putUInt("boots", boots);
        state.end();
    }

    ESP_LOGW(TAG, "Start: %s, %u crash(es) in a row, safe mode %s",
             resetReasonName(resetReason), static_cast<unsigned>(crashStreak),
             safeMode ? "on" : "off");

    // Nothing to do without Wi-Fi details; and after repeated failures (no
    // network around) only try every few starts so a start is never slowed.
    if (WiFi.SSID().isEmpty()) return;
    if (failures >= FAILURES_BEFORE_BACKOFF && boots % BACKOFF_EVERY != 0) {
        return;
    }

    for (uint32_t waited = 0;
         waited < WIFI_WAIT_MS && WiFi.status() != WL_CONNECTED; waited += 100) {
        delay(100);
    }
    bool success = false;
    if (WiFi.status() == WL_CONNECTED) {
        checkinDone = false;
        checkinResult = CheckinResult();
        if (createInternalTask(checkinTask, "fleetCheckin", 16384, nullptr, 1,
                               nullptr, 0) == pdPASS) {
            for (uint32_t waited = 0; !checkinDone && waited < TASK_WAIT_MS;
                 waited += 100) {
                delay(100);
            }
        }
        // The finished task's 16 KB stack is freed by the idle task; let it
        // happen before Bluetooth starts allocating around it.
        delay(300);
        success = checkinDone && checkinResult.ok;
        ESP_LOGW(TAG, "Check-in %s (%u command(s))",
                 success ? "ok" : ("failed: " + checkinResult.error).c_str(),
                 static_cast<unsigned>(checkinResult.commands.size()));
    } else {
        ESP_LOGW(TAG, "Check-in skipped: Wi-Fi did not connect");
    }

    if (askMachineMagic == ASK_MACHINE_UPDATE) {
        askMachineMagic = 0;  // asked once, whatever the outcome
        if (state.begin("fleet", false)) {
            state.putString("dmach", success && checkinResult.answeredMachineRequest
                                         ? checkinResult.machineRequestMessage
                                         : String("The dashboard could not be reached."));
            state.end();
        }
    }
    if (state.begin("fleet", false)) {
        state.putUInt("fails", success ? 0 : failures + 1);
        // What the Dashboard page shows. Kept from the last start, so it also
        // answers when this start could not check in.
        if (success) {
            state.putString("dname", checkinResult.name);
            state.putString("dchan", checkinResult.channel);
            state.putString("dlatest", checkinResult.latestVersion);
            state.putBool("davail", checkinResult.updateAvailable);
            state.putString("dlast", "OK");
        } else if (WiFi.status() == WL_CONNECTED) {
            state.putString("dlast", "Failed: " + checkinResult.error);
        } else if (!WiFi.SSID().isEmpty()) {
            state.putString("dlast", "No Wi-Fi at start");
        }
        state.end();
    }

    ESP_LOGW(TAG, "Before Bluetooth: internal free=%u largest=%u",
             static_cast<unsigned>(
                 heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(
                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));

    if (success) {
        for (const Command &command : checkinResult.commands) {
            runCommand(command);
        }
    }

    // The menu asked for a check: look for new firmware now and install it.
    // Does not return when something installs (the remote restarts on it).
    const bool checkRequested = checkAtStartMagic == CHECK_AT_START;
    checkAtStartMagic = 0;
    if (state.begin("fleet", false)) {
        // After an install the new firmware says so; otherwise keep the last result.
        if (!checkRequested && state.getString("dcheck", "") == "Installing") {
            state.putString("dcheck", String("Updated to ") + VERSION);
        }
        if (checkRequested) state.putString("dcheck", "Installing");  // replaced below
        state.end();
    }
    if (checkRequested) {
        String detail;
        fleetInstallUpdate(detail);  // restarts on success
        if (state.begin("fleet", false)) {
            state.putString("dcheck", detail == "server offered no update for this remote"
                                          ? String("Up to date")
                                          : "Failed: " + detail);
            state.end();
        }
    }
}

#else

String fleetDashboardDescription() { return ""; }
void fleetRequestStartupCheck() {}
void fleetRequestMachineUpdate(const char *) {}
void fleetEarlyInit() {}
void fleetBootCheckin() {}
bool fleetSerialCommand(const char *) { return false; }

#endif
