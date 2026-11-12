#ifndef RADR_SERVICES_FLEET_H
#define RADR_SERVICES_FLEET_H

#include <Arduino.h>

/**
 * Fleet client for the wireless remote: lets the builder's own server see the
 * remote's status and logs, restart it and update it, and lets the browser
 * setup page register it and set up its Wi-Fi over USB.
 *
 * Only active in builds that define FLEET_SERVER and FLEET_KEY (the `fleet`
 * PlatformIO environment, see fleet_build.py); in every other build these
 * functions do nothing.
 *
 * Unlike the OSSM, the remote cannot keep a secure connection open: NimBLE
 * and a TLS handshake do not fit in internal RAM together (see AGENTS.md,
 * "Memory Rules"). So the remote checks in once per start, before Bluetooth
 * is initialised, while there is plenty of memory:
 *
 *   - it reports its status, and the log lines kept from its last session
 *     (including the lines written just before a crash),
 *   - it receives restart and update commands and runs them straight away,
 *   - then it continues booting as normal.
 *
 * A check-in is skipped, never delayed for long: at most about six seconds
 * to join Wi-Fi, and after three failed attempts only every fifth start.
 */

// First thing in setup(): log capture and the USB setup channel.
void fleetEarlyInit();

// In setup(), after Wi-Fi is started and before initBLE().
void fleetBootCheckin();

// The text of the remote's "Dashboard" menu page: which dashboard it talks
// to, its name there, its update channel, how the last check-in went and
// whether newer firmware is waiting. Empty in builds without the fleet client.
String fleetDashboardDescription();

// Called by the serial identity reader for every line it cannot interpret.
// Returns true when the line was meant for the fleet client.
bool fleetSerialCommand(const char *line);

#endif  // RADR_SERVICES_FLEET_H
