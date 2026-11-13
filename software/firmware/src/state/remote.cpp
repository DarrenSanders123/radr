#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include "remote.h"

#include <devices/device.h>
#include <utils/psramTask.h>

#include "devices/researchAndDesire/ossm/ossm_state.h"

StateLogger stateLogger;
// Static pointer to hold the state machine instance
sml::sm<ossm_remote_state, sml::thread_safe<ESP32RecursiveMutex>, sml::logger<StateLogger>> *stateMachine = nullptr;

// While the remote is simply connected to an OSSM (menu or control screens),
// look at the machine's state now and then. If the machine starts an update on
// its own (the dashboard asked it to, or it checks at start-up) the remote
// shows "updating, do not restart" and pulses its LED, so nobody pulls the
// plug halfway. The screens that already follow the OSSM do their own reading.
static void ossmWatchTask(void *)
{
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (stateMachine == nullptr || device == nullptr || !device->isConnected)
            continue;
        if (!(stateMachine->is("main_menu"_s) ||
              stateMachine->is("device_draw_control"_s) ||
              stateMachine->is("simple_penetration_control"_s)))
            continue;
        const std::string json = device->readRawState();
        if (!json.empty())
            ingestOssmStateJson(json.c_str(), json.size());
    }
}

void initStateMachine()
{
    if (stateMachine == nullptr)
    {
        stateMachine = new sml::sm<ossm_remote_state, sml::thread_safe<ESP32RecursiveMutex>, sml::logger<StateLogger>>(stateLogger);

        stateMachine->process_event(done{});
        // The stack lives in PSRAM: internal RAM is what Bluetooth needs.
        createPsramTask(ossmWatchTask, "ossmWatch", 8 * configMINIMAL_STACK_SIZE,
                        nullptr, 1, nullptr, 1);
    }
}

void fireStateMachineDoneEvent() {
    if (stateMachine) {
        stateMachine->process_event(done{});
    }
}

// Deferred to the timer task: this is fired from inside on_entry actions
// (task creation happens there), and SML must not re-enter process_event.
static void postTaskFailedEvent(void *, uint32_t) {
    if (stateMachine) {
        stateMachine->process_event(task_failed_event{});
    }
}

void fireStateMachineTaskFailedEvent() {
    xTimerPendFunctionCall(postTaskFailedEvent, nullptr, 0, pdMS_TO_TICKS(10));
}

void fireStateMachineOssmStateEvent() {
    if (stateMachine) {
        stateMachine->process_event(ossm_state_event{});
    }
}

void fireStateMachineOssmNoWifiEvent() {
    if (stateMachine) {
        stateMachine->process_event(ossm_no_wifi_event{});
    }
}

void fireStateMachineOssmUnsupportedEvent() {
    if (stateMachine) {
        stateMachine->process_event(ossm_unsupported_event{});
    }
}

void fireStateMachineOssmLinkLostEvent() {
    if (stateMachine) {
        stateMachine->process_event(disconnected_event{});
    }
}
