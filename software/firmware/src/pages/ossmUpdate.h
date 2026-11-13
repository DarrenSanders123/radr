#pragma once

#include <string>

// Asks the connected OSSM to check for and install its own firmware update
// (BLE go:update). Progress arrives as ossm_state_event; see machine.h.
void startOssmUpdateRequest();

// The connected OSSM's address, as read from its pairing info the last time an
// update was requested. Empty if it was never read. Never does BLE I/O.
std::string ossmMacAddress();

// After an OSSM drops the link to update and restart: scan for the same OSSM
// and connect to it again by itself (connected_event moves the machine on).
// Gives up after a few minutes with done. Cancelled by stopOssmReconnect.
void startOssmReconnect();
void stopOssmReconnect();

// Draws the Install / Cancel page with the version the OSSM was offered.
void drawOssmUpdateAvailableFromState();

// Second go:update = confirm the install on the OSSM.
void confirmOssmInstall();
