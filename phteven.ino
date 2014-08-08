/*
  TODO:
  - Scan for key presses using interrupt ISR
  - pairing mode function
  - keypress combination for virtual keyboard toggle ( short press pair button )
  - pairing mode keypress ( long press pair button )
  - factory reset function (send "SF,1" then run bluetoothSetup() or reset the MCU)
  - factory reset button combination ( Home + Pair for 5 seconds? )
  - get remote address regularly, store in eeprom for more reliable reconnecting with C,<ADDR>
  - enable status reports from module and read them regularly, watching for disconnection
  - periodically query module for connection status
  - don't set name if it's already what we want
  - status indicator lights
  - code efficiency, DRY
*/

#define DEBUG
#ifdef DEBUG  
  #define debug_out(msg) Serial.println(msg)
#else
  #define debug_out(msg) // no-op
#endif

#ifdef __AVR_ATmega328P__ // Duemilanove/Uno/Boarduino

  #define BLUETOOTH_RX_PIN 5
  #define BLUETOOTH_TX_PIN 4

  #define BLUETOOTH_ENABLE_PIN 6

  #define MATRIX_INTERRUPT_PIN 3
  #define MATRIX_A_PIN 8
  #define MATRIX_B_PIN 9
  #define MATRIX_C_PIN 10

#elif __AVR_ATmega32U4__ // Micro/Leonardo

  #define BLUETOOTH_RX_PIN 8
  #define BLUETOOTH_TX_PIN 7

  #define BLUETOOTH_ENABLE_PIN 6

  #define MATRIX_INTERRUPT_PIN 3
  #define MATRIX_A_PIN 9
  #define MATRIX_B_PIN 10
  #define MATRIX_C_PIN 11

#else
  #warning Unkown Processor Type
  // ATTiny84
  #define BLUETOOTH_RX_PIN 1
  #define BLUETOOTH_TX_PIN 0

  #define BLUETOOTH_ENABLE_PIN 3

  #define MATRIX_INTERRUPT_PIN 2
  #define MATRIX_A_PIN 8
  #define MATRIX_B_PIN 9
  #define MATRIX_C_PIN 10
#endif

#include <SoftwareSerial.h>

SoftwareSerial bluetooth = SoftwareSerial(BLUETOOTH_RX_PIN, BLUETOOTH_TX_PIN); // rx, tx

struct commandWithCallback {
  char cmd[20];
  char expected[20];
  uint8_t bufferSize;
  int8_t failure_index;
  void (*callback)();
};

struct keyMap {
  uint8_t key_mask;
  uint16_t key_code;
};

                       // LOW    MASK  TARGET  TARGET
                       // LINES  DEC   HEX     DEC     DESC
static struct keyMap keyMaps[] = {
  { 0b00000010, 128 }, // C      2  -> 0x80    128     Play/Pause
  { 0b00000100, 256 }, // B      4  -> 0x100   256     Scan Next Track
  { 0b00001000, 512 }, // A      8  -> 0x200   512     Scan Previous Track
  { 0b00000110, 16 },  // C B    6  -> 0x10    16      Volume Up
  { 0b00001110, 32 },  // C B A  14 -> 0x20    32      Volume Down
  { 0b00001100, 1 },   // B A    12 -> 0x1     1       Home
  { 0b00001010, 8 }    // C A    10 -> 0x8     8       Toggle Keyboard/Layout
};
#define KEY_MAPS_SIZE 7 // sizeof() can be problematic with struct arrays

char rxBuffer[32];  // was 64, but I doubt we need this

boolean key_pressed = false;

uint8_t matrix_pins[] = { MATRIX_A_PIN, MATRIX_B_PIN, MATRIX_C_PIN };

// Delay for bluetooth module after responding with "AOK"
#define BLUETOOTH_RESPONSE_DELAY 100  // delay in ms
#define BLUETOOTH_RESET_DELAY    2000 // delay in ms

#ifdef DEBUG
  // We don't need key descriptions outside of debug mode
  char * getKeyCodeDescription(uint16_t key_code) {

    struct keyCodeDescription {
      uint16_t key_code;
      char description[20];
    };

    static struct keyCodeDescription keyCodeDescriptions[] = {
      { 128, "Play/Pause" },
      { 256, "Scan Next Track" },
      { 512, "Scan Previous Track" },
      { 16, "Volume Up" },
      { 32, "Volume Down" },
      { 1, "Home" },
      { 8, "Keyboard Toggle" }
    };
    #define KEY_CODE_DESCRIPTIONS_SIZE 7 // sizeof() can be problematic with struct arrays

    for(uint8_t i = 0; i < KEY_CODE_DESCRIPTIONS_SIZE; i++) {
      if (keyCodeDescriptions[i].key_code == key_code) {
        return keyCodeDescriptions[i].description;
      }
    }

    debug_out("Description not found for key code: " + String(key_code, DEC));
    return 0; // unknown key code
  }
#endif

uint16_t keyMaskToKeyCode(uint8_t key_mask) {
  for(uint8_t i = 0; i < KEY_MAPS_SIZE; i++) {
    if (keyMaps[i].key_mask == key_mask) {
      debug_out("Key: " + String(getKeyCodeDescription(keyMaps[i].key_code)));
      return keyMaps[i].key_code;
    }
  }
  debug_out("Key not found for mask: " + String(key_mask, BIN));
  return 0;
}

void setup() {
  #ifdef DEBUG
  Serial.begin(9600);
  #endif

  bluetooth.begin(9600);

  pinMode(BLUETOOTH_ENABLE_PIN, OUTPUT);
  digitalWrite(BLUETOOTH_ENABLE_PIN, LOW);
  bluetoothSetup();

  pinMode(MATRIX_INTERRUPT_PIN, INPUT_PULLUP);

  for (uint8_t i = 0; i < sizeof(matrix_pins); i++) {
    pinMode(matrix_pins[i], INPUT_PULLUP);
  }
}

void loop() {
  if (!digitalRead(MATRIX_INTERRUPT_PIN)) {
    if (!key_pressed) {
      key_pressed = true;

      debug_out("Interrupt pin low");

      char key_mask = 0x0;
      for (uint8_t i = 0; i < sizeof(matrix_pins); i++) {

        #ifdef DEBUG
          if (!digitalRead(matrix_pins[i])) {
            debug_out("Pin " + String(matrix_pins[i], DEC) + " low");
          }
        #endif 
        key_mask |= !digitalRead(matrix_pins[i]);
        key_mask <<= 0x01;
      }

      debug_out("Key Mask: " + String(key_mask, BIN));

      send_consumer_key(keyMaskToKeyCode(key_mask));

    }
  } else {
    if (key_pressed == true) {
      debug_out("Key released\n");
      send_consumer_key(0x0);
      key_pressed = false;
    }
  }
  delay(10);
}

/* Command mode ($$$) commands:
SF,1 Reset to factory defaults
SM,6 Pairing Mode
SM,0 Slave Mode
SM,1 Master Mode
SM,2 Trigger Mode
SM,3 Auto-Connect Master Mode
SM,4 Auto-Connect DTR Mode
SM,5 Auto-Connect Any Mode
SM,6 Pairing Mode 
SN,<string> This command sets the device name,up to 20 alphanumeric characters.
S-,<string> Sets serialized friendly name of device with MAC
SP,<string> This command sets the security pin code (ex: SP,1234)
SU,<value> set uart baud rate (examples: SU,96 SU,57, SU,11
S~,6 Sets HID profile
C connect to last stored BT address stored with SR
SR,<mac address> Store the current bluetooth mac address
SA,<flag> set authentication method (0, Open, 1, Keyboard I/O Mode (Default), 2 "Just works"mode ,4 auth with pin code)
SH,<flag> Set HID flag register (default 0200), get with GH
R,1 Reboot
 */

void setHidMode() {
  bluetooth.flush();
  debug_out("Setting Hid Mode \"S~,6\"");
  bluetooth.print("S~,6");
  bluetooth.write('\r');
  delay(BLUETOOTH_RESPONSE_DELAY);
  bluetoothReceive(rxBuffer);
  bluetooth.flush();

  #ifdef DEBUG
    // check that the auth mode was set properly
    if (bluetoothCheckReceive(rxBuffer, "AOK", 3)) {
      debug_out("Auth mode set");
    } else {
      debug_out("\tError setting auth mode:");
      debug_out(rxBuffer);
      debug_out("\tEND");
    }
  
    if (bluetoothCheckReceive(rxBuffer, "AOK", 3)) {
      // reboot to complete mode change
      debug_out("Rebooting to change Hid mode");
      bluetooth.print("R,1");
      bluetooth.write('\r');
      delay(BLUETOOTH_RESPONSE_DELAY);
      bluetoothReceive(rxBuffer);
    
      if (bluetoothCheckReceive(rxBuffer, "Reboot!", 7)) {
        debug_out("\tReboot successful");
      } else {
        debug_out("\tError rebooting:");
        debug_out(rxBuffer);
        debug_out("\tEND");
      }
    }
  #elif
    /*
      if not in debug mode, we don't care about the response from the
      mode set command because we can't do anything about it so we
      try to reboot regardless.
    */
    bluetooth.print("R,1");
    bluetooth.write('\r');
  #endif

  // reboot takes longer to come back
  delay(BLUETOOTH_RESET_DELAY);

  // get back in to command mode
  bluetooth.flush();  // delete buffer contents
  debug_out("Returning to command mode");
  bluetooth.print("$$$");  // Command mode string
  bluetoothReceive(rxBuffer);
  #ifdef DEBUG
    if (bluetoothCheckReceive(rxBuffer, "CMD", 3)) {
        debug_out("\tBack in command mode");
    } else {
      debug_out("\tError re-entering command mode:");
      debug_out(rxBuffer);
      debug_out("\tEND");
    }
  #endif
}

void bluetoothSetup() {
  #ifdef DEBUG
    // additional time to open terminal
    delay(3000);
  #endif
  debug_out("Setting up bluetooth module");
  digitalWrite(BLUETOOTH_ENABLE_PIN, HIGH);
  delay(500);

  int timeout = 1000;  // timeout, in the rare case the module is unresponsive
  bluetooth.write((uint8_t) 0); // Disconnects, if connected
  delay(BLUETOOTH_RESPONSE_DELAY);
  
  bluetooth.flush();  // delete buffer contents
  bluetooth.print("$$$");  // Command mode string
  do // This gets the module out of state 3
  {  // continuously send \r until there is a response, usually '?'
    bluetooth.write('\r');  
    debug_out("-");  // Debug info for how many \r's required to get a respsonse
  } while ((!bluetooth.available()) && (timeout-- > 0));
  
  while (bluetooth.available())
    #ifdef DEBUG
      Serial.write(bluetooth.read());
    #else
      char c = bluetooth.read(); // useless, we just want all data read
    #endif
  delay(BLUETOOTH_RESPONSE_DELAY);

  static struct commandWithCallback cmds[] = {
    { "GM", "Pair", 4, 0 },
    { "GH", "0200", 4, 1 },
    { "GN", "PHTEVEN", 7, 2 },
    { "GA", "2", 1, 3 },
    { "G~", "6", 1, -1, &setHidMode },
    { "---", "END", 3 }
  };

  static struct commandWithCallback failureCmds[] = {
    { "SM,6", "AOK", 3 },
    { "SH,0200", "AOK", 3 }, // 0300 toggle virtual keyboard on connect
    { "S-,PHTEVEN", "AOK", 3 },
    { "SA,2", "AOK", 3 },
  };

  for(uint8_t i = 0; i < 6; i++) {
    if (!processCommand(cmds[i])) {
      if (cmds[i].failure_index > -1) {
        debug_out("executing failure command");
        processCommand(failureCmds[cmds[i].failure_index]);
      }
    }
  }

  debug_out("Setup complete");
}

uint8_t processCommand(struct commandWithCallback cmd) {
  bluetooth.flush();
  debug_out("Sending command '" + String(cmd.cmd) + "'");
  bluetooth.print(cmd.cmd);
  bluetooth.write('\r');
  delay(BLUETOOTH_RESPONSE_DELAY);
  bluetoothReceive(rxBuffer);
  if (bluetoothCheckReceive(rxBuffer, cmd.expected, cmd.bufferSize)) {
    debug_out("\tGot expected response: " + String(cmd.expected));
    return 1;
  } else {
    debug_out("\tERROR");
    debug_out(rxBuffer);
    debug_out("\tEND");
    return 0;
  }
}

void send_consumer_key(uint16_t keycode) {
  byte high_byte = highByte(keycode);
  byte low_byte = lowByte(keycode);
  debug_out("Sending code to bluetooth. High Byte: " + String(high_byte, HEX) + ", Low Byte: " + String(low_byte, HEX));
  bluetooth.write(0xFD); // Consumer Report
  bluetooth.write(0x03);
  bluetooth.write(0x03);
  bluetooth.write((byte)low_byte); // low byte
  bluetooth.write((byte)high_byte); // high byte
}

uint8_t bluetoothReceive(char * dest)
{
  int timeout = 1000;
  char c = 0; // avoid uninitialized warning
  int i = 0;

  while ((--timeout > 0) && (c != 0x0A))
  {
    if (bluetooth.available())
    {
      c = bluetooth.read();
      if (c != 0x0D)
        dest[i++] = c;
      timeout = 1000;	// reset timeout
    }
  }
  return timeout;
}

/* This function checks two strings against eachother. A 1 is returned if
   they're equal, a 0 otherwise. 
   This is used to verify the response from the RN-42 module. */
uint8_t bluetoothCheckReceive(char * src, char * expected, int bufferSize)
{
  int i = 0;
  // char c; // complier claims this is ununsed

  while ((src[i] != 0x0A) || (i < bufferSize))
  {
    if (src[i] != expected[i])
    {
      return 0;
    }
    i++;
  }
  return 1;
}
