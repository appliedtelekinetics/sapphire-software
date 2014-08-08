/*
  TODO:
  Scan for key presses using interrupt ISR
  get remote address regularly, store in eeprom for more reliable reconnecting with C,<ADDR>
  enable status reports from module and read them regularly, watching for disconnection
  periodically query module for connection status
  don't set name if it's already what we want

  "After first pairing the host to a device with the Bluetooth HID module, the host initiates a connection. However, if the initial connection is broken, as the case when the power is cycled, the device must re-connect to the host. (The host will not initiate a connec- tion.)
Using DTR mode 4 (default) or pairing mode 6 allows the module to auto-connect back to the last paired host. Alternatively, you can reconnect by sending the C command from command mode. See the following examples:

    SM,4 // Use GPIO6 to make and break connections
    SM,6 // Automatically make connections without using GPIO6

  status indicator lights
  code efficiency, DRY
*/

// #define DEBUG
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
  #error Unkown Processor Type
  // ATTiny84
  // #define BLUETOOTH_RX_PIN 1
  // #define BLUETOOTH_TX_PIN 0

  // #define BLUETOOTH_ENABLE_PIN 3

  // #define MATRIX_INTERRUPT_PIN 2
  // #define MATRIX_A_PIN 8
  // #define MATRIX_B_PIN 9
  // #define MATRIX_C_PIN 10
#endif

#include <SoftwareSerial.h>

SoftwareSerial bluetooth = SoftwareSerial(BLUETOOTH_RX_PIN, BLUETOOTH_TX_PIN); // rx, tx

typedef struct commandWithCallback {
  char cmd[20];
  char expected[20];
  uint8_t bufferSize;
  void (*callback)();
} commandWithCallback;

typedef struct keyMap {
  uint8_t key_mask;
  uint16_t key_code;
} keyMap;

                       // LOW    MASK  TARGET  TARGET
                       // LINES  DEC   HEX     DEC
static keyMap keyMaps[] = {
  { 0b00000010, 128 }, // C      2  -> 0x80    128
  { 0b00000100, 256 }, // B      4  -> 0x100   256
  { 0b00001000, 512 }, // A      8  -> 0x200   512
  { 0b00000110, 16 },  // C B    6  -> 0x10    16
  { 0b00001110, 32 },  // C B A  14 -> 0x20    32
  { 0b00001100, 1 },   // B A    12 -> 0x1     1
  { 0b00001010, 8 }    // C A    10 -> 0x8     8 
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
      { 8, "Keyboard Toggle" } // Keyboard Layout / iOS Virtual Keyboard Toggle
    };
    #define KEY_CODE_DESCRIPTIONS_SIZE 7 // sizeof() can be problematic with struct arrays

    for(uint8_t i = 0; i < KEY_CODE_DESCRIPTIONS_SIZE; i++) {
      if (keyCodeDescriptions[i].key_code == key_code) {
        return keyCodeDescriptions[i].description;
      }
    }

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

void setName() {
  bluetooth.flush();
  debug_out("Setting name \"S-,PHTEVEN\"");
  bluetooth.print("S-,PHTEVEN");
  bluetooth.write('\r');
  delay(BLUETOOTH_RESPONSE_DELAY);
  bluetoothReceive(rxBuffer);
  #ifdef DEBUG
    if (bluetoothCheckReceive(rxBuffer, "AOK", 3)) {
      debug_out("\tName set");
    } else {
      debug_out("\tError setting name:");
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
    Serial.write(bluetooth.read());
  delay(BLUETOOTH_RESPONSE_DELAY);

  static commandWithCallback cmds[] = {
    { "GM", "Pair", 4 },
    { "GH", "0200", 4 },
    { "GN", "PHTEVEN", 7, &setName },
    { "GA", "2", 1 },
    { "G~", "6", 1 },
    { "---", "END", 3 }
  };

  for(uint8_t i = 0; i < 6; i++) {
    bluetooth.flush();
    debug_out("Sending command '" + String(cmds[i].cmd) + "'");
    bluetooth.print(cmds[i].cmd);
    bluetooth.write('\r');
    delay(BLUETOOTH_RESPONSE_DELAY);
    bluetoothReceive(rxBuffer);
    if (bluetoothCheckReceive(rxBuffer, cmds[i].expected, cmds[i].bufferSize)) {
      debug_out("\tGot expected response: " + String(cmds[i].expected));
    } else {
      debug_out("\tERROR");
      debug_out(rxBuffer);
      debug_out("\tEND");
      if (cmds[i].callback) {
        debug_out("executing callback");
        cmds[i].callback();
      }
    }
  }

  // enterCommandMode();
  // bluetoothReceive(rxBuffer);
  // expectedResponse(rxBuffer, "CMD", 3);

  // sendCommand("GM");
  // bluetoothReceive(rxBuffer);
  // if (expectedResponse(rxBuffer, "Pair", 4)) {
  //   debug_out("Already in Pairing Mode");
  // } else {
  //   sendCommand("SM,6");
  //   bluetoothReceive(rxBuffer);
  //   expectedResponse(rxBuffer, "AOK", 3);
  // }

  // sendCommand("GH");
  // bluetoothReceive(rxBuffer);
  // if (expectedResponse(rxBuffer, "0200", 4)) {
  //   debug_out("Already using correct HID hash");
  // } else {
  //   sendCommand("SH,0200");
  //   //sendCommand("SH,0300"); // 0300 to toggle ios keyboard
  //   // sendCommand("SH,0203");
  //   bluetoothReceive(rxBuffer);
  //   expectedResponse(rxBuffer, "AOK", 3);
  // }

  // // Add a check with the GN command here so we don't needlessly set this
  // sendCommand("S-,PHTEVEN");
  // bluetoothReceive(rxBuffer);
  // expectedResponse(rxBuffer, "AOK", 3);
  
  // sendCommand("GA");
  // bluetoothReceive(rxBuffer);
  // if (expectedResponse(rxBuffer, "2", 1)) {
  //   debug_out("Already in correct auth mode");
  // } else {
  //   sendCommand("SA,2");
  //   bluetoothReceive(rxBuffer);
  //   expectedResponse(rxBuffer, "AOK", 3);
  // }

  // sendCommand("G~");
  // bluetoothReceive(rxBuffer);
  // if (expectedResponse(rxBuffer, "6", 1)) {
  //   debug_out("Already in HID Mode");
  // } else {
  //   sendCommand("S~,6");
  //   bluetoothReceive(rxBuffer);
  //   expectedResponse(rxBuffer, "AOK", 3);

  //   sendCommand("R,1"); // reboot to change profile
  //   bluetoothReceive(rxBuffer);
  //   delay(BLUETOOTH_RESET_DELAY);
  //   expectedResponse(rxBuffer, "Reboot!", 7);  
  // }
  
  // sendCommand("---");
  // bluetoothReceive(rxBuffer);
  // expectedResponse(rxBuffer, "END", 3);

  debug_out("Setup complete");
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
