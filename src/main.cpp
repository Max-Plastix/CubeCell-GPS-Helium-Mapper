/*
 * Max-Plastix Mapper build for
 * Heltec CubeCell GPS-6502 HTCC-AB02S
 *
 * Based on the CubeCell GPS example from
 * libraries\LoRa\examples\LoRaWAN\LoRaWAN_Sensors\LoRaWan_OnBoardGPS_Air530\LoRaWan_OnBoardGPS_Air530.ino
 * and on Jas Williams version from
 * https://github.com/jas-williams/CubeCell-Helium-Mapper.git with GPS Distance
 * and improvements from https://github.com/hkicko/CubeCell-GPS-Helium-Mapper
 *
 * Apologies for inconsistent style and naming; there were many hands in this code and Heltec has quite the Platform library.
 *
 */

#include "Arduino.h"
/* CubeCell bundlesfonts into their standard Platform library.
 * Unfortunately, it declares them right in the.h Header file itself, and in their library binary, 
 * so we have to tolerate getting TWO copies of each font included in flash. */
#include "HT_Display.h"
#include "HT_SSD1306Wire.h"   // HelTec's old version of SSD1306Wire.h, pulls in HT_DisplayFonts.h

#include "GPS_Air530Z.h"  // Bastard plagerized Heltec packed-in library
#include "LoRaWan_APP.h"      // Global values for IDs and LoRa state
#include "configuration.h"    // User configuration
#include "credentials.h"      // Helium LoRaWAN credentials
#include "cyPm.h"             // For deep sleep
#include "hw.h"               // for CyDelay()
#include "timeServer.h"       // Timer handlers

//#include "TinyGPS++.h"        // More recent/superior library

#include "images.h"  // For Satellite icon

extern SSD1306Wire display;     // Defined in LoRaWan_APP.cpp (!)
extern uint8_t isDispayOn;      // [sic] Defined in LoRaWan_APP.cpp
#define isDisplayOn isDispayOn  // Seriously.  It's wrong all over LoRaWan_APP.

extern uint32_t UpLinkCounter;    // FCnt, Frame Count, Uplink Sequence Number
extern bool wakeByUart;           // Declared in no-source Heltec binary that was
// \cores\asr650x\projects\CubeCellLib.a(AT_Command.o):D:\lib\650x/AT_Command.c:54
extern HardwareSerial GPSSerial;  // Defined in GPSTrans.cpp as UART_NUM_1
Air530ZClass AirGPS;              // Just for Init.  Has some Air530Z specials.
// TinyGPSPlus GPS;                  // Avoid the Heltec library as much as we can
#define GPS AirGPS

#if defined(REGION_EU868)
/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
#else
uint16_t userChannelsMask[6] = {0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
#endif

/* To represent battery voltage in one payload byte we scale it to hundredths of a volt,
 * with an offset of 2.0 volts, giving a useable range of 2.0 to 4.56v, perfect for any
 * Lithium battery technology. */
#define ONE_BYTE_BATTERY_V(mV) ((uint8_t)(((mV + 5) / 10) - 200) & 0xFF)

/* Unset APB stuff, but CubeCellLib.a requires that we declare them */
uint8_t nwkSKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appSKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint32_t devAddr = 0;

/* LoRaWAN Configuration.  Edit in platformio.ini */
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t loraWanClass = LORAWAN_CLASS;

/* Application data transmission duty cycle.  value in [ms]. */
uint32_t appTxDutyCycle = 1000 - 100;  // Must be a bit less than GPS update rate!

/* OTAA or ABP.  (Helium only supports OTAA) */
bool overTheAirActivation = LORAWAN_NETMODE;

/* ADR enable */
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash,
 * when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port.  NOTE: Name is fixed, as LoRa library will extern ours (!) */
uint8_t appPort = 0;

/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:
  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)
  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

static TimerEvent_t BatteryUpdateTimer;
static TimerEvent_t ScreenUpdateTimer;
static TimerEvent_t KeyDownTimer;
static TimerEvent_t MenuIdleTimer;
static TimerEvent_t DeepSleepTimer;
static TimerEvent_t ScreenOnTimer;
static TimerEvent_t JoinFailTimer;

float min_dist_moved = MIN_DIST_M;             // Meters of movement that count as moved
uint32_t max_time_ms = MAX_TIME_S * 1000;      // Time interval of Map uplinks
uint32_t rest_time_ms = REST_TIME_S * 1000;    // slower time interval (screen off)
uint32_t sleep_time_ms = SLEEP_TIME_S * 1000;  // slowest time interval (deep sleep)
uint32_t tx_time_ms = max_time_ms;             // The currently active send interval

// Deadzone (no uplink) location and radius
double deadzone_lat = DEADZONE_LAT;
double deadzone_lon = DEADZONE_LON;
double deadzone_radius_m = DEADZONE_RADIUS_M;
boolean in_deadzone = false;

boolean in_menu = false;         // Menu currently displayed?
const char *menu_prev;           // Previous menu name
const char *menu_cur;            // Highlighted menu name
const char *menu_next;           // Next menu name
boolean is_highlighted = false;  // highlight the current menu entry
int menu_entry = 0;              // selected item
boolean go_menu_select = false;  // Do the menu thing(in main context)

boolean justSendNow = false;  // Send an uplink right now

boolean key_down = false;    // User Key pressed right now?
uint32_t keyDownTime;        // how long was it pressed for
void userKeyIRQ(void);       // (soft) interrupt handler for key press
volatile boolean keyIRQ = false;      // IRQ flag for key event
boolean long_press = false;  // did this count as a Long press?

uint32_t last_fix_ms = 0;    // When did we last get a good GPS fix?
double last_send_lat = 0.0;  // Last sent Latitude
double last_send_lon = 0.0;  // Last sent Longitude
uint32_t last_send_ms;       // time of last uplink
boolean is_joined = false;   // True after Join complete

boolean need_light_sleep = false;  // Should we be in low-power state?
uint32_t need_deep_sleep_s = 0;    // Should we be in lowest-power state?
boolean in_light_sleep = false;    // Are we presently in low-power
boolean in_deep_sleep = false;     // Are we in deepest sleep?
boolean hold_screen_on = false;    // Are we holding the screen on from key press?
boolean stay_on = false;           // Has the user asked the screen to STAY on?

boolean is_gps_lost = false;    // No GPS Fix?
uint32_t last_lost_gps_ms = 0;  // When did we last cry about no GPS
uint32_t last_moved_ms = 0;     // When did we last notice significant movement
uint16_t battery_mv;            // Last measured battery voltage in millivolts

char buffer[40];  // Scratch string buffer for display strings

void onDeepSleepTimer(void);
void onJoinFailTimer(void);
void deepest_sleep(uint32_t sleepfor_s);

// SK6812 (WS2812).  DIN is IO13/GP13/Pin45
void testRGB(void) {
  for (uint32_t i = 0; i <= 30; i++) {
    turnOnRGB(i << 16, 10);
  }
  for (uint32_t i = 0; i <= 30; i++) {
    turnOnRGB(i << 8, 10);
  }
  for (uint32_t i = 0; i <= 30; i++) {
    turnOnRGB(i, 10);
  }
  turnOnRGB(0, 0);
}

// RGB LED power on (Vext to SK5812 pin 4)
void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

// RGB LED power off
void VextOFF(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}

void printGPSInfo(void) {
  // Serial.print("Date/Time: ");
  if (GPS.date.isValid()) {
    Serial.printf("%d-%02d-%02d", GPS.date.year(), GPS.date.month(), GPS.date.day());
  } else {
    Serial.print("INVALID");
  }

  if (GPS.time.isValid()) {
    Serial.printf(" %02d:%02d:%02d.%02d", GPS.time.hour(), GPS.time.minute(), GPS.time.second(), GPS.time.centisecond());
  } else {
    Serial.print(" INVALID");
  }
  Serial.println();

#if 1
  Serial.print(" LAT: ");
  Serial.print(GPS.location.lat(), 6);
  Serial.print(", LON: ");
  Serial.print(GPS.location.lng(), 6);
  Serial.print(", ALT: ");
  Serial.print(GPS.altitude.meters());

  Serial.print(", HDOP: ");
  Serial.print(GPS.hdop.hdop());
  Serial.print(", AGE: ");
  Serial.print(GPS.location.age());
  Serial.print(", COURSE: ");
  Serial.print(GPS.course.deg());
  Serial.print(", SPEED: ");
  Serial.print(GPS.speed.kmph());
#endif
  Serial.print(", Sats: ");
  Serial.print(GPS.satellites.value());

  Serial.println();
}

void displayLogoAndMsg(String msg, uint32_t wait_ms) {
  display.clear();
  display.drawXbm(0, 0, 128, 42, helium_logo_bmp);
  // displayBatteryLevel();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 54 - 16 / 2, msg);
  Serial.println(msg);
  display.display();

  if (wait_ms) {
    delay(wait_ms);
  }
}

boolean screenOffMode = false;

void switchScreenOffMode() {
  screenOffMode = true;
  VextOFF();
  display.stop();
  isDisplayOn = 0;
}

void switchScreenOnMode() {
  screenOffMode = false;
  VextON();
  isDisplayOn = 1;
  display.init();
  display.clear();
  display.display();
}

// Fetch Data Rate? (DR_0 to DR_5)
int8_t loraDataRate(void) {
  MibRequestConfirm_t mibReq;
  LoRaMacStatus_t status;
  int8_t ret = -1;

  mibReq.Type = MIB_CHANNELS_DATARATE;
  status = LoRaMacMibGetRequestConfirm(&mibReq);
  if (status == LORAMAC_STATUS_OK) {
    ret = mibReq.Param.ChannelsDatarate;
  }

  return ret;
}

#define SCREEN_HEADER_HEIGHT 24
uint8_t _screen_line = SCREEN_HEADER_HEIGHT - 1;
SSD1306Wire *disp;

void draw_screen(void);

void onScreenUpdateTimer(void) {
  draw_screen();
  TimerReset(&ScreenUpdateTimer);
  TimerStart(&ScreenUpdateTimer);
}

void screen_print(const char *text, uint8_t x, uint8_t y, uint8_t alignment) {
  if (isDisplayOn) {
    disp->setTextAlignment((DISPLAY_TEXT_ALIGNMENT)alignment);
    disp->drawString(x, y, text);
  }
}

void screen_print(const char *text, uint8_t x, uint8_t y) {
  screen_print(text, x, y, TEXT_ALIGN_LEFT);
}

void screen_print(const char *text) {
  Serial.printf(">>> %s", text);

  if (isDisplayOn) {
    disp->print(text);
    if (_screen_line + 8 > disp->getHeight()) {
      // scroll
    }
    _screen_line += 8;
  }
}

void screen_setup() {
  // Display instance
  disp = &display;
  disp->setFont(ArialMT_Plain_10);

  // Scroll buffer
  disp->setLogBuffer(4, 40);
}

uint32_t gps_start_time;

void configure_gps(void) {
  // Adjust the Green 1pps LED to have shorter blinks.
  const uint8_t cmdbuf[] = {0xBA, 0xCE, 0x10, 0x00, 0x06, 0x03, 0x40, 0x42, 0x0F, 0x00, 0x10, 0x27, 0x00,
                            0x00, 0x03, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x63, 0x69, 0x15, 0x0B};
  /* CFG-TX Time Pulse: 10mS width (vs 100), only if fixed, UTC Time, Automatic source */
  GPSSerial.write(cmdbuf, sizeof(cmdbuf));
  delay(50);

  AirGPS.setmode(MODE_GPS_BEIDOU_GLONASS);
  delay(50);

  AirGPS.setNMEA(NMEA_RMC | NMEA_GGA);  // Eliminate unused message traffic like SV
  delay(50);

#if 0
  // GPSSerial.write("$PCAS02,500*1A\r\n"); /* 500mS updates */
  GPSSerial.write("$PCAS02,1000*2E\r\n"); /* 1S updates */
  GPSSerial.flush();
  delay(50);
#endif
}

void start_gps(void) {
  Serial.println("Starting GPS:");
  AirGPS.begin(115200);  // Faster messages; less CPU, more idle.
  configure_gps();
  gps_start_time = millis();
}

void fast_start_gps() {
  Serial.println("(Wake GPS)");
  pinMode(GPIO14, OUTPUT);
  digitalWrite(GPIO14, LOW);
  GPSSerial.begin(115200);  // Faster messages; less CPU, more idle.
  gps_start_time = millis();
  while (GPS.getNMEA() == "0") {
    if (millis() - gps_start_time > SLEEP_GPS_TIMEOUT_S * 1000)
      return;
  }
  configure_gps();
  gps_start_time = millis();
}

void update_gps() {
  static boolean firstfix = true;
  uint32_t now_fix_count;
  static uint32_t last_fix_count = 0;

  while (GPSSerial.available()) {
    int c = GPSSerial.read();
    if (c > 0) {
      char c8 = c;
      // Serial.print(c8);
      GPS.encode(c8);
    }
  }

  now_fix_count = GPS.sentencesWithFix();
  if (now_fix_count != last_fix_count && GPS.location.isValid() && GPS.time.isValid() && GPS.date.isValid()) {
    last_fix_ms = millis();
    if (firstfix) {
      firstfix = false;
      snprintf(buffer, sizeof(buffer), "GPS fix: %d sec\n", (last_fix_ms - gps_start_time) / 1000);
      screen_print(buffer);
      printGPSInfo();
    }
  }
}

void stopGPS() {
  AirGPS.end();
}

/* In Heltec's brilliance, they put the User Key button input on top of the Battery ADC with a complicated switch,
 * instead of using one of the many other GPIOs that would not conflict at all.  Because of this, we should
 * ignore fake keypress inputs while sampling the battery ADC.  The getBatteryVoltage() call will also turn the
 * ADC Control switch over to Battery with pinMode(VBAT_ADC_CTL,OUTPUT) and restore it after.
 * Note that battery measurement takes significant time, so should be done infrequently and outside of any critical
 * LoRa timing.
 */
void update_battery_mv(void) {
  detachInterrupt(USER_KEY);  // ignore phantom button pushes
  battery_mv = (getBatteryVoltage() * (1024.0 * VBAT_CORRECTION)) / 1024;
  attachInterrupt(USER_KEY, userKeyIRQ, BOTH);
  Serial.printf("Bat: %d mV\n", battery_mv);
}

void onBatteryUpdateTimer(void) {
  update_battery_mv();
  TimerReset(&BatteryUpdateTimer);
  TimerStart(&BatteryUpdateTimer);
}

/* Populate appData/appDataSize with Mapper frame */
boolean prepare_map_uplink(uint8_t port) {
  uint32_t lat, lon;
  int alt, speed, sats;

  unsigned char *puc;

  appDataSize = 0;

  if (!GPS.location.isValid())
    return false;

  last_send_lat = GPS.location.lat();
  last_send_lon = GPS.location.lng();

  lat = ((last_send_lat + 90) / 180.0) * 16777215;
  lon = ((last_send_lon + 180) / 360.0) * 16777215;

  alt = (uint16_t)GPS.altitude.meters();
  // course = GPS.course.deg();
  speed = (uint16_t)GPS.speed.kmph();
  sats = GPS.satellites.value();
  // hdop = GPS.hdop.hdop();

  puc = (unsigned char *)(&lat);
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[0];

  puc = (unsigned char *)(&lon);
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[0];

  puc = (unsigned char *)(&alt);
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[0];

  puc = (unsigned char *)(&speed);
  appData[appDataSize++] = puc[0];

  appData[appDataSize++] = ONE_BYTE_BATTERY_V(battery_mv);

  appData[appDataSize++] = (uint8_t)(sats & 0xFF);
  return true;
}

void gps_passthrough(void) {
  Serial.println("GPS Passthrough forever...");
  int c;
  while (1) {
    c = GPSSerial.read();
    if (c != -1)
      Serial.write(c);

    c = Serial.read();
    if (c != -1) {
      GPSSerial.write(c); }   
  }
}

struct menu_item {
  const char *name;
  void (*func)(void);
};

void menu_send_now(void) {
  justSendNow = true;
}

void menu_power_off(void) {
  screen_print("\nPOWER OFF...\n");
  delay(1000);  // Give some time to read the screen
  VextOFF();
  pinMode(Vext, ANALOG);
  pinMode(ADC, ANALOG);
  display.sleep();
  display.displayOff();
  display.stop();
  detachInterrupt(RADIO_DIO_1);
  isDisplayOn = 0;
  AirGPS.end();

  Radio.Sleep();
  TimerStop(&KeyDownTimer);
  TimerStop(&BatteryUpdateTimer);
  TimerStop(&ScreenUpdateTimer);
  TimerStop(&MenuIdleTimer);
  TimerStop(&DeepSleepTimer);
  TimerStop(&ScreenOnTimer);

  // Must Reset to exit
  wakeByUart = false;
  while (1) lowPowerHandler();
}

void menu_distance_plus(void) {
  min_dist_moved += 10;
}
void menu_distance_minus(void) {
  min_dist_moved -= 10;
  if (min_dist_moved < 10)
    min_dist_moved = 10;
}
void menu_time_plus(void) {
  max_time_ms += 60 * 1000;
}
void menu_time_minus(void) {
  max_time_ms -= 60 * 1000;
  if (max_time_ms < 60 * 1000)
    max_time_ms = 60 * 1000;
}
void menu_gps_passthrough(void) {
  gps_passthrough();
  // Does not return!
}
void menu_deadzone_here(void) {
  if (GPS.location.isValid()) {
    deadzone_lat = GPS.location.lat();
    deadzone_lon = GPS.location.lng();
  }
}
void menu_stay_on(void) {
  stay_on = !stay_on;
}

void menu_experiment(void) {
  screen_print("\nExperiment..\n");

  delay(2000);
  keyIRQ = false;
  deepest_sleep(5);

  screen_print("Done\n");

  // uint32 reason = CySysGetResetReason();
}

struct menu_item menu[] = {{"Send Now", menu_send_now},           {"Power Off", menu_power_off},     {"Distance +10", menu_distance_plus},
                           {"Distance -10", menu_distance_minus}, {"Time +60", menu_time_plus},      {"Time -60", menu_time_minus},
                           {"Deadzone Here", menu_deadzone_here}, {"USB GPS", menu_gps_passthrough}, {"Stay ON", menu_stay_on},
                           //{"Experiment", menu_experiment}
                           };
#define MENU_ENTRIES (sizeof(menu) / sizeof(menu[0]))

void onMenuIdleTimer(void) {
  in_menu = false;
  is_highlighted = false;
}

void onScreenOnTimer(void) {
  hold_screen_on = false;
}

void menu_press(void) {
  if (in_menu)
    menu_entry = (menu_entry + 1) % MENU_ENTRIES;
  else
    in_menu = true;

  menu_prev = menu[(menu_entry - 1) % MENU_ENTRIES].name;
  menu_cur = menu[menu_entry].name;
  menu_next = menu[(menu_entry + 1) % MENU_ENTRIES].name;

  draw_screen();
  TimerReset(&MenuIdleTimer);
  TimerStart(&MenuIdleTimer);
}

void menu_selected(void) {
  if (in_menu) {
    is_highlighted = true;
    draw_screen();
    menu[menu_entry].func();  // Might not return.
  } else {
    in_menu = true;
    draw_screen();
  }
}

void menu_deselected(void) {
  is_highlighted = false;
  draw_screen();
}

void onKeyDownTimer(void) {
  // Long Press!
  long_press = true;
  go_menu_select = true;
}

// Interrupt handler for button press
void userKeyIRQ(void) {
  keyIRQ = true;
}

void userKeyIRQ_process(void) {
  if (!keyIRQ)
    return;
  keyIRQ = false;

  if (!key_down && digitalRead(USER_KEY) == LOW) {
    // Key Pressed
    key_down = true;
    long_press = false;
    keyDownTime = millis();
    TimerReset(&KeyDownTimer);
    TimerStart(&KeyDownTimer);
  }

  if (key_down && digitalRead(USER_KEY) == HIGH) {
    // Key Released
    key_down = false;
    if (long_press) {
      menu_deselected();
    } else {
      TimerStop(&KeyDownTimer);  // Cancel timer
      menu_press();
    }
#if 1
    uint32_t press_ms = millis() - keyDownTime;
    Serial.printf("[Key Pressed %d ms.]\n", press_ms);
#endif
  }
  TimerReset(&MenuIdleTimer);

  hold_screen_on = true;
  TimerStop(&ScreenOnTimer);
  TimerReset(&ScreenOnTimer);
  TimerStart(&ScreenOnTimer);
}

void gps_time(char *buffer, uint8_t size) {
  snprintf(buffer, size, "%02d:%02d:%02d", GPS.time.hour(), GPS.time.minute(), GPS.time.second());
}

void screen_header(void) {
  uint32_t sats;
  uint32_t now = millis();
  char bat_level = '?';

  sats = GPS.satellites.value();

  // Cycle display every 3 seconds
  if (millis() % 6000 < 3000) {
    // 2 bytes of Device EUI with Voltage and Current
    snprintf(buffer, sizeof(buffer), "#%04X", (devEui[6] << 8) | devEui[7]);
    disp->setTextAlignment(TEXT_ALIGN_LEFT);
    disp->drawString(0, 2, buffer);

    snprintf(buffer, sizeof(buffer), "%d.%02dV", battery_mv / 1000, (battery_mv % 1000) / 10);
  } else {
    if (!GPS.time.isValid() || sats < 3)
      snprintf(buffer, sizeof(buffer), "*** NO GPS ***");
    else
      gps_time(buffer, sizeof(buffer));
  }

  disp->setTextAlignment(TEXT_ALIGN_CENTER);
  disp->drawString(disp->getWidth() / 2, 2, buffer);

  // Satellite count
  disp->setTextAlignment(TEXT_ALIGN_RIGHT);
  disp->drawString(disp->getWidth() - SATELLITE_IMAGE_WIDTH - 4, 2, itoa(sats, buffer, 10));
  disp->drawXbm(disp->getWidth() - SATELLITE_IMAGE_WIDTH, 0, SATELLITE_IMAGE_WIDTH, SATELLITE_IMAGE_HEIGHT, SATELLITE_IMAGE);

  if (battery_mv > USB_POWER_VOLTAGE * 1000)
    bat_level = 'U';
  else if (battery_mv > REST_LOW_VOLTAGE * 1000)
    bat_level = 'H';
  else if (battery_mv > SLEEP_LOW_VOLTAGE * 1000)
    bat_level = 'L';
  else
    bat_level = 'Z';

  // Second status row:
  snprintf(buffer, sizeof(buffer), "%ds / %ds   %dm  %c%c%c%c\n",
           (now - last_send_ms) / 1000,  // Time since last send
           tx_time_ms / 1000,            // Interval Time
           (int)min_dist_moved,          // Interval Distance
           bat_level,                    // U for Unlimited Power (USB), Hi, Low, Zero
           stay_on ? 'S' : '-',          // S for Screen Stay ON
           in_deadzone ? 'D' : '-',      // D for Deadzone
           !is_joined ? 'X' : '-'        // X for Not Joined Yet
  );
  disp->setTextAlignment(TEXT_ALIGN_LEFT);
  disp->drawString(0, 12, buffer);

  // disp->setTextAlignment(TEXT_ALIGN_RIGHT);
  // disp->drawString(disp->getWidth(), 12, cached_sf_name);

  disp->drawHorizontalLine(0, SCREEN_HEADER_HEIGHT, disp->getWidth());
}

#define MARGIN 15

void draw_screen(void) {
  disp->setFont(ArialMT_Plain_10);
  disp->clear();
  screen_header();

  if (in_menu) {
    disp->setTextAlignment(TEXT_ALIGN_CENTER);
    disp->drawString(disp->getWidth() / 2, SCREEN_HEADER_HEIGHT + 5, menu_prev);
    disp->drawString(disp->getWidth() / 2, SCREEN_HEADER_HEIGHT + 28, menu_next);
    // if (is_highlighted)
    //    disp->clear();
    disp->drawHorizontalLine(MARGIN, SCREEN_HEADER_HEIGHT + 16, disp->getWidth() - MARGIN * 2);
    snprintf(buffer, sizeof(buffer), is_highlighted ? ">>> %s <<<" : "%s", menu_cur);
    disp->drawString(disp->getWidth() / 2, SCREEN_HEADER_HEIGHT + 16, buffer);
    disp->drawHorizontalLine(MARGIN, SCREEN_HEADER_HEIGHT + 28, disp->getWidth() - MARGIN * 2);
    disp->drawVerticalLine(MARGIN, SCREEN_HEADER_HEIGHT + 16, 28 - 16);
    disp->drawVerticalLine(disp->getWidth() - MARGIN, SCREEN_HEADER_HEIGHT + 16, 28 - 16);
  } else {
    disp->drawLogBuffer(0, SCREEN_HEADER_HEIGHT);
  }
  disp->display();
}

void setup() {
  boardInitMcu();

  Serial.begin(115200);

  if (!(devEui[0] || devEui[1] || devEui[2] || devEui[3] || devEui[4] || devEui[5] || devEui[6] || devEui[7]))
    LoRaWAN.generateDeveuiByChipID();  // Overwrite devEui with chip-unique value

#if (STAGING_CONSOLE)
  devEui[0] = 0xBB;
#endif

#if (AT_SUPPORT)
  enableAt();
#endif

  isDisplayOn = 1;
  display.init();  // displayMcuInit() will init the display, but if we want to
                   // show our logo before that, we need to init ourselves.
  displayLogoAndMsg(APP_VERSION, 100);

  start_gps();  // GPS takes the longest to settle.

  LoRaWAN.displayMcuInit();  // This inits and turns on the display
  deviceState = DEVICE_STATE_INIT;

  /* This will switch deviceState to DEVICE_STATE_SLEEP and schedule a SEND
   timer which will switch to DEVICE_STATE_SEND if saved network info exists
   and no new JOIN is necessary */
  /* Unfortunately, it uses the inscrutible checkNetInfo() binary-only function, which appears to have bugs. */
  // LoRaWAN.ifskipjoin();

  // Setup user button - this must be after LoRaWAN.ifskipjoin(), because the
  // button is used there to cancel stored settings load and initiate a new join
  pinMode(USER_KEY, INPUT);
  attachInterrupt(USER_KEY, userKeyIRQ, BOTH);

  screen_setup();
  screen_print(APP_VERSION "\n");

  TimerInit(&KeyDownTimer, onKeyDownTimer);
  TimerSetValue(&KeyDownTimer, LONG_PRESS_MS);

  TimerInit(&MenuIdleTimer, onMenuIdleTimer);
  TimerSetValue(&MenuIdleTimer, MENU_TIMEOUT_MS);

  TimerInit(&DeepSleepTimer, onDeepSleepTimer);
  //TimerSetValue(&DeepSleepTimer, SLEEP_TIME_S * 1000);

  TimerInit(&ScreenOnTimer, onScreenOnTimer);
  TimerSetValue(&ScreenOnTimer, SCREEN_ON_TIME_MS);

  TimerInit(&JoinFailTimer, onJoinFailTimer);
  // TimerSetValue(&JoinFailTimer, 2 * JOIN_TIMEOUT_S * 1000);

  TimerInit(&BatteryUpdateTimer, onBatteryUpdateTimer);
  TimerSetValue(&BatteryUpdateTimer, BATTERY_UPDATE_RATE_MS);
  TimerStart(&BatteryUpdateTimer);
  update_battery_mv();

  TimerInit(&ScreenUpdateTimer, onScreenUpdateTimer);
  TimerSetValue(&ScreenUpdateTimer, SCREEN_UPDATE_RATE_MS);
  TimerStart(&ScreenUpdateTimer);  // Let's Go!
}

boolean send_lost_uplink() {
  uint32 now = millis();
  Serial.printf("Lost GPS %ds ago\n", (now - last_fix_ms) / 1000);
  unsigned char *puc;

  // Use last-known location; might be zero
  double lat = ((last_send_lat + 90) / 180.0) * 16777215;
  double lon = ((last_send_lon + 180) / 360.0) * 16777215;
  uint8_t sats = GPS.satellites.value();
  uint16_t lost_minutes = MIN(0xFFFF, (now - last_fix_ms) / 1000 / 60);

  appPort = FPORT_LOST_GPS;
  appDataSize = 0;
  puc = (unsigned char *)(&lat);
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[0];

  puc = (unsigned char *)(&lon);
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[0];

  appData[appDataSize++] = ONE_BYTE_BATTERY_V(battery_mv);

  appData[appDataSize++] = (uint8_t)(sats & 0xFF);

  puc = (unsigned char *)(&lost_minutes);
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[0];

  snprintf(buffer, sizeof(buffer), "%d NO-GPS %dmin\n", UpLinkCounter, lost_minutes);
  screen_print(buffer);

  last_lost_gps_ms = now;
  LoRaWAN.send();

  return true;
}

boolean send_uplink(void) {
  uint32_t now = millis();

  if (is_gps_lost && (now - last_fix_ms < GPS_LOST_WAIT_S * 1000)) {
    // Recovered
    screen_print("Found GPS\n");
    is_gps_lost = false;
  }
  if (!is_gps_lost && (now - last_fix_ms > GPS_LOST_WAIT_S * 1000)) {
    // Haven't seen GPS in a while.  Send a non-Mapper packet
    screen_print("Lost GPS!\n");
    is_gps_lost = true;
    return send_lost_uplink();
  }
  if (is_gps_lost && (now - last_lost_gps_ms) > GPS_LOST_TIME_S * 1000) {
    // Still Lost.  Continue crying about it every GPS_LOST_TIME_S
    return send_lost_uplink();
  }
  if (is_gps_lost && (now - last_fix_ms) > SLEEP_WAIT_S * 1000) {
    // Been lost a really long time.  Can't tell if we're moving.  Sleep
    need_light_sleep = true;
    need_deep_sleep_s = GPS_LOST_TIME_S;
    return false;
  }

  // Shouldn't happen, but if it does.. can't compute distance
  if (!GPS.location.isValid())
    return false;

  double lat = GPS.location.lat();
  double lon = GPS.location.lng();

  double dist_moved = GPS.distanceBetween(last_send_lat, last_send_lon, lat, lon);
  double deadzone_dist = GPS.distanceBetween(deadzone_lat, deadzone_lon, lat, lon);
  in_deadzone = (deadzone_dist <= deadzone_radius_m);

#if 1
  Serial.printf("[Time %d / %ds, Moved %d / %dm in %ds %c %c %c]\n",
                (now - last_send_ms) / 1000,  // Time
                tx_time_ms / 1000,            // interval
                (int32_t)dist_moved,          // moved
                (int32_t)min_dist_moved,
                (now - last_moved_ms) / 1000,  // last movement ago
                in_deadzone ? 'D' : 'd', need_light_sleep ? 'S' : 's', need_deep_sleep_s != 0 ? 'Z' : 'z');
#endif

  /* Set tx interval (and screen state!) based on time since last movement: */
  if ((now - last_moved_ms < REST_WAIT_S * 1000) && (battery_mv > REST_LOW_VOLTAGE * 1000)) {
    // If we recently moved and battery is good.. keep the update rate high and screen on
    tx_time_ms = max_time_ms;
    need_light_sleep = false;
    need_deep_sleep_s = 0;
  } else if (battery_mv > USB_POWER_VOLTAGE * 1000) {
    // Don't slow down on USB power, or topped-off battery, ever
    tx_time_ms = max_time_ms;
    // However, OLED screens can burn-in, so do turn it off while stationary
    need_light_sleep = (now - last_moved_ms > REST_WAIT_S * 1000);
    need_deep_sleep_s = 0;
  } else if (now - last_moved_ms > SLEEP_WAIT_S * 1000) {
    // Been a really long time, Slowest interval, GPS OFF
    tx_time_ms = sleep_time_ms;
    need_light_sleep = true;
    need_deep_sleep_s = SLEEP_TIME_S;
  } else {
    // Parked/stationary or battery below REST_LOW_VOLTAGE
    need_light_sleep = true;
    need_deep_sleep_s = 0;  // Keep GPS on
    tx_time_ms = rest_time_ms;
  }

  // Last, there is User Override!
  if (stay_on) {
    need_light_sleep = false;
    need_deep_sleep_s = 0;
  }

  /* Do we send an uplink now? */

  // Deadzone means we don't send unless asked
  if (in_deadzone && !justSendNow)
    return false;

  // Don't send any mapper packets for time/distance without GPS fix
  if (is_gps_lost)
    return false;

  char because = '?';
  if (justSendNow) {
    justSendNow = false;
    Serial.println("** SEND_NOW");
    because = '>';
  } else if (dist_moved > min_dist_moved) {
    Serial.println("** MOVING");
    last_moved_ms = now;
    because = 'D';
  } else if (now - last_send_ms > tx_time_ms) {
    Serial.println("** TIME");
    because = 'T';
  } else {
    return false;  // Nothing to do, go home early
  }

  appPort = FPORT_MAPPER;
  if (!prepare_map_uplink(appPort))  // Don't send bad data
    return false;

  // The first distance-moved is crazy, since has no origin.. don't put it on screen.
  if (dist_moved > 1000000)
    dist_moved = 0;

  printGPSInfo();
  snprintf(buffer, sizeof(buffer), "%d %c %ds %dm\n", UpLinkCounter, because, (now - last_send_ms) / 1000, (int32_t)dist_moved);
  // Serial.print(buffer);
  screen_print(buffer);

  // Serial.println("send..");
  last_send_ms = now;
  LoRaWAN.send();
  // Serial.println("..sent");

  return true;
}

void enter_light_sleep(void) {
  Serial.println("ENTER light sleep");
  in_light_sleep = true;
  TimerStop(&ScreenUpdateTimer);
  VextOFF();  // No RGB LED or OLED power
  display.stop();
  isDisplayOn = false;
}

void exit_light_sleep(void) {
  Serial.println("EXIT light sleep");
  in_light_sleep = false;
  VextON();
  isDisplayOn = true;
  display.init();
  screen_setup();
  draw_screen();
  TimerReset(&ScreenUpdateTimer);
  TimerStart(&ScreenUpdateTimer);
}

boolean deep_sleep_wake = false;
uint32_t wake_count = 0;
void onDeepSleepTimer(void) {
  // Serial.println("WAKE Time");
  deep_sleep_wake = true;
}

void deepest_sleep(uint32_t sleepfor_s) {
  boolean was_in_light_sleep = in_light_sleep;

  if (!in_light_sleep)
    enter_light_sleep();  // Turn off screen
  in_deep_sleep = true;

  AirGPS.end();
  Radio.Sleep();
  TimerStop(&BatteryUpdateTimer);

  Serial.printf("Sleep %d s[\n", sleepfor_s);
  Serial.flush();
  delay(20);

  /* Set a Timer to wake */
  deep_sleep_wake = false;
  TimerSetValue(&DeepSleepTimer, sleepfor_s * 1000);
  TimerStart(&DeepSleepTimer);
  wake_count = 0;
  wakeByUart = false;
  do {
    lowPowerHandler();  // SLEEP
    wake_count++;
  } while (!deep_sleep_wake && !keyIRQ);
  TimerStop(&DeepSleepTimer);
  Serial.printf("]up; Woke %d times\n", wake_count);
  in_deep_sleep = false;

  fast_start_gps();
  last_fix_ms = 0;
  uint32_t gps_start_time = millis();
  do {
    update_gps();
  } while (!last_fix_ms && (millis() - gps_start_time) < SLEEP_GPS_TIMEOUT_S * 1000);
  if (!last_fix_ms)
    Serial.println("(Woke but no Fix)");
  else {
    Serial.print("Wake fix @ ");
    gps_time(buffer, sizeof(buffer));
    Serial.println(buffer);
  }
  TimerReset(&BatteryUpdateTimer);
  TimerStart(&BatteryUpdateTimer);

  if (!was_in_light_sleep)
    exit_light_sleep();  // Restore screen if it was on.
}

void onJoinFailTimer(void) {
  screen_print("Join timed out!\n");

  //need_deep_sleep_s = JOIN_RETRY_TIME_S;
  // Now try again

  TimerReset(&JoinFailTimer);
  TimerStart(&JoinFailTimer);
}

void loop() {
  static uint32_t lora_start_time;

  // Handle any pending key events
  userKeyIRQ_process();
  
  if (go_menu_select) {
    go_menu_select = false;
    menu_selected();
  }

  if (need_light_sleep && !in_light_sleep && !in_menu && !hold_screen_on) {
    enter_light_sleep();
  }
  if (in_light_sleep && (!need_light_sleep || in_menu || hold_screen_on)) {
    exit_light_sleep();
  }
  if (need_deep_sleep_s && !in_deep_sleep && !in_menu && !hold_screen_on) {
    deepest_sleep(need_deep_sleep_s);
    need_deep_sleep_s = 0;
  }

  // Serial.print(".");
  update_gps();  // Digest any pending bytes to update position

  switch (deviceState) {
    case DEVICE_STATE_INIT: {
      // Serial.print("[INIT] ");
      lora_start_time = millis();
#if (AT_SUPPORT)
      getDevParam();
#endif
      printDevParam();
      LoRaWAN.init(loraWanClass, loraWanRegion);
      // deviceState = DEVICE_STATE_JOIN;
      break;
    }
    case DEVICE_STATE_JOIN: {
      // Serial.print("[JOIN] ");
      TimerSetValue(&JoinFailTimer, JOIN_TIMEOUT_S * 1000);
      TimerStart(&JoinFailTimer);
      LoRaWAN.displayJoining();
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND: {
      // Serial.print("[SEND] ");
      TimerStop(&JoinFailTimer);
      if (!is_joined) {
        is_joined = true;
        snprintf(buffer, sizeof(buffer), "Joined Helium: %d sec\n", (millis() - lora_start_time) / 1000);
        screen_print(buffer);
        justSendNow = true;
      }
      send_uplink();
      deviceState = DEVICE_STATE_CYCLE;  // take a break if we sent or not.
      break;
    }
    case DEVICE_STATE_CYCLE: {
      // Serial.print("[CYCLE] ");
      LoRaWAN.cycle(appTxDutyCycle);  // Sets a timer to check state
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP: {
      // Serial.print("[SLEEP] ");
      wakeByUart = true;  // Without this, sleeps through GPS
      LoRaWAN.sleep();    // Causes serial port noise if it does sleep
      break;
    }
    default: {
      Serial.printf("Surprising state: %d\n", deviceState);
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}
