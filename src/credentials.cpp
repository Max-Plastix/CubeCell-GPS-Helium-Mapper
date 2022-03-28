/* Credentials definition */
#include "credentials.h"

#ifndef USE_OTAA
#error "Only OTAA is supported for Helium!"
#endif

/*
This is where you define the three key values that map your Device to the Helium Console.
All three values must match between the code and the Console.

There are two general ways to go about this:
1) Let the Console pick random values for one or all of them, and copy them here in the code.
-or-
2) Define values here in the code, and then copy them to the Console to match.

When the Mapper boots, it will show all three values in the Monitor console, like this:

+DevEui=00072441043ABCDE(For OTAA Mode)
+AppEui=6081F9BF908E2EA0(For OTAA Mode)
+AppKey=CE3204B816687881BC91D8AD30AF3F55(For OTAA Mode)

This format is suitable for copying the hex values from Terminal/Monitor and pasting directly into the Helium Console as-is.
These are all MSB in the terminal, same as the Console.

If you want to take the random Console values for a new device, and use them here, be sure to select (default):
   Device EUI: msb
   App EUI:    msb
   App Key:    msb
in the Console, then click the arrows to expand the values with comma separators, then paste them below.
*/

// The DevEUI will be generated automatically based on the device macaddr, if these are all zero
uint8_t devEui[8] = {
    0, 0, 0, 0, 0, 0, 0, 0
};

// This value is commonly shared between many devices of the same type or server.
uint8_t appEui[8] = {
    0x60, 0x81, 0xF9, 0xBF, 0x90, 0x8E, 0x2E, 0xA0
};

// The key shown here is the Semtech default key.  You should probably change it to a lesser-known (random) value
uint8_t appKey[16] = {
    0xCE, 0x32, 0x04, 0xB8, 0x16, 0x68, 0x79, 0x91, 0xBC, 0x91, 0xD8, 0xAD, 0x30, 0xAF, 0x3F, 0x55
};
