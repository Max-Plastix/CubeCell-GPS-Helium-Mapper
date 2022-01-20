// Version
#define APP_NAME "Cube Mapper"
#define APP_VERSION "MaxP v2.3.5"

// -----------------------------------------------------------------------------
// CONFIGURATION
// Stuff you might reasonably want to change is here:
// TODO: Not all of this is implemented yet!
// -----------------------------------------------------------------------------

// The Mapper is always sending when it either moves some distance, or waits some time:
#define MIN_DIST_M 70.0       // Minimum distance in meters from the last sent location before we can send again. A hex is about 340m.
#define MAX_TIME_S (10 * 60)  // If no minimum movement, uplink will still be sent every N seconds

// After a while, it still watches GPS, but turns the display off and sends slower:
#define REST_WAIT_S (30 * 60)  // If we still haven't moved in this many seconds, start sending even slower
#define REST_TIME_S (20 * 60)  // Slow resting uplink frequency in seconds
#define REST_LOW_VOLTAGE 3.6   // Below this voltage, send more slowly

// After a long while, shut down GPS & screen and go into deeper sleep, missing the initial movement.
#define SLEEP_WAIT_S (4 * 60 * 60)  // If we are not moving (or no GPS) for this long, go into deeper sleep
#define SLEEP_TIME_S (20 * 60)      // Wake up this often to power on the GPS and check for movement
#define SLEEP_GPS_TIMEOUT_S 60      // How long to wait for a GPS fix each time we wake
#define SLEEP_LOW_VOLTAGE 3.4       // Below this voltage, stay in deep sleep

#define USB_POWER_VOLTAGE 4.15     // Above this voltage, assume we have unlimited power (4.1 is typical)
#define GPS_LOST_WAIT_S (5 * 60)   // How long after losing GPS do we call it lost?
#define GPS_LOST_TIME_S (15 * 60)  // How often to send Lost GPS packets?

#define JOIN_TIMEOUT_S (1 * 60)      // How long to spend trying to Join/re-Join
#define JOIN_RETRY_TIME_S (1 * 60)  // Join Timeout/Retry sleep interval

// Deadzone defines a circular area where no map packets will originate.
// Set Radius to zero to disable, or leave it enabled to select center position from menu.
// (Thanks to @Woutch for the name)
// Set it here or in your platformio.ini
#ifndef DEADZONE_LAT
#define DEADZONE_LAT 34.5678
#endif
#ifndef DEADZONE_LON
#define DEADZONE_LON -123.4567
#endif
#define DEADZONE_RADIUS_M 500  // meters

// -----------------------------------------------------------------------------
// Less-common Configuration items, but feel free to adjust..
// -----------------------------------------------------------------------------
#define VBAT_CORRECTION 1.004                       // Edit this for calibrating your battery voltage
#define MENU_TIMEOUT_MS 3000                        // Menu timeout milliseconds
#define LONG_PRESS_MS 500                           // How long to hold the button to Select
#define BATTERY_UPDATE_RATE_MS (15 * 1000)          // How often to sample battery voltage
#define SCREEN_UPDATE_RATE_MS 500                   // Refresh OLED screen this often
#define SCREEN_ON_TIME_MS (MENU_TIMEOUT_MS + 5000)  // Keep screen on this long after button
#define FPORT_MAPPER 2                              // FPort for Mapper Uplink messages -- must match Helium Console Decoder script!
#define FPORT_LOST_GPS 5                            // FPort for no-GPS uplink messages -- do not feed to Mapper