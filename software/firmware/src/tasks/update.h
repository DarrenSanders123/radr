
#ifndef RADR_UPDATE_H
#define RADR_UPDATE_H
#include <Arduino.h>

#include <task.h>

extern bool isSoftwareUpdateAvailable;
extern bool isFilesystemUpdateAvailable;

// Why the last self-update step stopped. Empty means "no update available"
// (up to date); anything else is shown on the Update Failed page.
extern String updateFailureReason;

bool isUpdateAvailable();
bool confirmRunningFirmware();

#ifdef FLEET_SERVER
// Fleet builds only: install the update the fleet server offers right now.
// Returns false (with the reason in `detail`) if it did not install.
bool fleetInstallUpdate(String &detail);
#endif

// FREERTOS Task

extern TaskHandle_t updateTaskHandle;
constexpr char updateTaskName PROGMEM[] = "updateTask";
extern TaskHandle_t updateFilesystemTaskHandle;
constexpr char updateFilesystemTaskName PROGMEM[] = "updateFilesystemTask";
extern TaskHandle_t updateSoftwareTaskHandle;
constexpr char updateSoftwareTaskName PROGMEM[] = "updateSoftwareTask";

void updateTask(void *pvParameters);
void updateFilesystemTask(void *pvParameters);
void updateSoftwareTask(void *pvParameters);
#endif
