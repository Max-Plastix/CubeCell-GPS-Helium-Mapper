# Overview
Helium Mapper build for the **Heltec CubeCell GPS-6502 HTCC-AB02S**

A Mapper device helps determine the Helium network coverage area.  Learn more about Mapping in general here: https://docs.helium.com/use-the-network/coverage-mapping/

This software is based on the CubeCell GPS example from Heltec's examples, 
and on Jas Williams version from https://github.com/jas-williams/CubeCell-Helium-Mapper.git with GPS Distance and
improvements from https://github.com/hkicko/CubeCell-GPS-Helium-Mapper
 This build copies some look and behavior from my TTGO T-Beam build at https://github.com/Max-Plastix/tbeam-helium-mapper.

## Note on hardware
If you have not yet bought any Helium Mapper hardware, consider the LilyGo TTGO T-Beam instead of the Heltec CubeCell.
The cost is similar, but the Heltec uses **closed-source binaries** in their Platform libraries while TTGO is open-source with ESP32.  The TTGO also has a superior GPS antenna, more buttons, WiFi, Bluetooth, and a power management IC.  The Heltec CubeCell is physically smaller and lower power in sleep.  Study the tradeoffs.

### CubeCell Version
Heltec has released multiple versions of the CubeCell GPS units.  This software works only with the `CubeCell GPS 6502 v1.1`.  You can see the different versions [here](https://heltec-automation-docs.readthedocs.io/en/latest/cubecell/dev-board/htcc-ab02s/hardware_update_log.html#v1-1).  The v1.1 has been for sale since early 2021, and has an AIR530Z GPS module.  If you have one of the older 2020 boards, then this build won't work on it.

## Usage
The CubeCell only has one User Button, so a short press steps to the next menu entry, and a long press selects that entry.

------ Everything below is copied from Kicko's https://github.com/hkicko/CubeCell-GPS-Helium-Mapper
# Uploading the code

**Note: If you prefer to use Arduino IDE, just take the \src\main.cpp file and rename it to "something".ino (for example CubeCell_GPS_Helium_Mapper.ino)**

Install Serial Driver. Find directions [here.](https://heltec-automation-docs.readthedocs.io/en/latest/general/establish_serial_connection.html)

Install [Visual Studio Code](https://code.visualstudio.com/Download). If you are using Windows, make sure the pick the System installer, not the User installer.

(Optional) When the Get Started wizard prompts you to install language extensions - install the C/C++ extension.

Install Git from https://git-scm.com/downloads or https://github.com/git-guides/install-git

Reboot your computer for the path changes to take effect.

Install the GitHub Pull Requests and Issues extension from the link [here](https://code.visualstudio.com/docs/editor/github).

Install [PlatformIO IDE](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)

Once you are in Visual Studio Code, go to the Explorer and click Clone Repository. Paste the URL you got from GitHub, by clicking on the Code button. When prompted for location, if you want to use the default location (and you are using Windows) do the following - select your Documents folder, if there is no PlatformIO sub-folder in it - create one and enter it, then if there is no Projects sub-folder inside - create it and select it as the location for the cloned repository. So the final location would be %userprofile%\Documents\PlatformIO\Projects

Open the cloned folder

Open the main.cpp from src sub-folder and wait. Initially the #include directives at the top will have squiggly lines as unknown, but relatively soon (within 5 min) PlatformIO will detect and install the required platform and libraries. If you don't want to wait, open PlatformIO and go to Platforms and install "ASR Microelectronics ASR650x". You can do that as a step right after installing PlatformIO.

Comment out/uncomment the appropriate line for your board version (for GPS Air530 or Air530Z) in main.cpp.

Comment out/uncomment the #define lines for VIBR_SENSOR, VIBR_WAKE_FROM_SLEEP, MENU_SLEEP_DISABLE_VIBR_WAKEUP, MAX_GPS_WAIT, MAX_STOPPED_CYCLES and edit the values for the timers if desired.

Enter DevEUI(msb), AppEUI(msb), and AppKey(msb) from Helium Console, at the respective places in main.cpp. The values must be in MSB format. From console press the expand button to get the ID's as shown below.

![Console Image](https://gblobscdn.gitbook.com/assets%2F-M21bzsbFl2WA7VymAxU%2F-M6fLGmWEQ0QxjrJuvoC%2F-M6fLi5NzuMeWSzzihV-%2Fcubecell-console-details.png?alt=media&token=95f5c9b2-734a-4f84-bb88-523215873116)

```
uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
```

Modify platformio.ini if you need to change LoRaWAN settings like region.

Click the PlatformIO: Build button. Address any compile errors and repeat until you get a clean build.

Connect the CubeCell to the computer with USB cable.

Click the PlatformIO: Upload button.

# Debug using Serial connection via USB

(Optional) Uncomment the line enabling the DEBUG code and build again.
```
//#define DEBUG // Enable/Disable debug output over the serial console
```
Click the PlatformIO: Serial Monitor button

# Setting up Console

In [Helium Console](https://console.helium.com/) create a new function call it Heltec decoder => Type Decoder => Custom Script

Copy and paste the decoder into the custom script pane

```
// Decoder for MaxPlastix mappers
//
// 11 Byte payload: 
// 3 Lat, 3 Long, 2 Altitude (m), 1 Speed (km/hr), 1 Battery, 1 Sats.
// Accuracy is a dummy value required by some Integrations.
// Battery is 1/100 of a volt, offset by 2v for a range of 2.00 to 4.56 volts.
//
function Decoder(bytes, port) {
  var decoded = {};

  // All formats carry a lat & lon reading:
  var latitude = ((bytes[0] << 16) >>> 0) + ((bytes[1] << 8) >>> 0) + bytes[2];
  latitude = (latitude / 16777215.0 * 180) - 90;

  var longitude = ((bytes[3] << 16) >>> 0) + ((bytes[4] << 8) >>> 0) + bytes[5];
  longitude = (longitude / 16777215.0 * 360) - 180;

  switch (port) {
    case 2: // Mapper! (Cargo and Heatmap too)
      decoded.latitude = latitude;
      decoded.longitude = longitude;

      var altValue = ((bytes[6] << 8) >>> 0) + bytes[7];
      var sign = bytes[6] & (1 << 7);
      if (sign)
        decoded.altitude = 0xFFFF0000 | altValue;
      else
        decoded.altitude = altValue;

      decoded.speed = parseFloat((((bytes[8])) / 1.609).toFixed(2));
      decoded.battery = parseFloat((bytes[9] / 100 + 2).toFixed(2));
      decoded.sats = bytes[10];
      decoded.accuracy = 2.5; // Bogus Accuracy required by Cargo/Mapper integration
      break;
    case 5: // System status
      decoded.last_latitude = latitude;
      decoded.last_longitude = longitude;
      decoded.battery = parseFloat((bytes[6] / 100 + 2).toFixed(2));
      decoded.value = bytes[8];
      decoded.status = bytes[7];
      switch (bytes[7]) {
        case 1:
          decoded.status = "BOOT";
          break;
        case 2:
          decoded.status = "USB ON";
          break
        case 3:
          decoded.status = "USB OFF";
          break;
      }
      break;
    case 6: // Lost GPS
      decoded.last_latitude = latitude;
      decoded.last_longitude = longitude;
      decoded.battery = parseFloat((bytes[6] / 100 + 2).toFixed(2));
      decoded.sats = bytes[7];
      decoded.minutes = ((bytes[8] << 8) >>> 0) + bytes[9];
      break;
  }

  return decoded;
}
```

Create two integrations one for CARGO (optional) and one for MAPPERS.
For CARGO use the available prebuilt integration. 
For MAPPERS use a custom HTTP integration with POST Endpoint URL https://mappers.helium.com/api/v1/ingest/uplink

Go to Flows and from the Nodes menu add your device, decoder function and integrations. 
Connect the device to the decoder. 
Connect the decoder to the integrations.

Useful links:

[Mappers](http://mappers.helium.com) and [Cargo](https://cargo.helium.com)

[Integration information with Mappers](https://docs.helium.com/use-the-network/coverage-mapping/mappers-api/)

[Integration information for Cargo](https://docs.helium.com/use-the-network/console/integrations/cargo/)
