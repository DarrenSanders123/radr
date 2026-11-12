#pragma once

#include <string>

// Asks the connected OSSM to check for and install its own firmware update
// (BLE go:update). Progress arrives as ossm_state_event; see machine.h.
void startOssmUpdateRequest();

// The connected OSSM's address, as read from its pairing info the last time an
// update was requested. Empty if it was never read. Never does BLE I/O.
std::string ossmMacAddress();

// Draws the Install / Cancel page with the version the OSSM was offered.
void drawOssmUpdateAvailableFromState();

// Second go:update = confirm the install on the OSSM.
void confirmOssmInstall();
