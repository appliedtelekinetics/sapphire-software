/*
  TODO:
  - pairing mode function
  - pairing mode keypress ( long press keyboard toggle button )
  - Disable repeat on certain buttons (like volume control and play/pause)
  - factory reset function (send "SF,1"(fac reset) or "SR,Z" (erase stored mac) then run bluetoothSetup() or reset the MCU)
  - factory reset button combination ( Home + Pair for 5 seconds? )
  - get remote address regularly, store in eeprom for more reliable reconnecting with C,<ADDR>
  - enable status reports from module and read them regularly, watching for disconnection
  - periodically query module for connection status
  - don't set name if it's already what we want
  - status indicator lights
  - code efficiency, DRY
  - button hysteresis
  - bluetooth module power saving
*/

//#define DEBUG
  
#ifdef DEBUG  
  #define debug_out(msg) Serial.println(msg)
#else
  #define debug_out(msg) // no-op
#endif

#ifdef __AVR_ATmega328P__ // Duemilanove/Uno/Boarduino
  #define ENABLE_SLEEP
  #define SLEEP_METHOD SLEEP_MODE_PWR_SAVE
  /* ATmega sleep modes:
   SLEEP_MODE_IDLE     - least power savings
   SLEEP_MODE_ADC
   SLEEP_MODE_PWR_SAVE - good
   SLEEP_MODE_STANDBY
   SLEEP_MODE_PWR_DOWN - most power savings */

  #define BLUETOOTH_RX_PIN 5
  #define BLUETOOTH_TX_PIN 4

  #define BLUETOOTH_ENABLE_PIN 6

  // #define BUTTON_INTERRUPT 0
  // #define BUTTON_INTERRUPT_PIN 2

  // For matrix buttons
  #define MATRIX_A_PIN 8
  #define MATRIX_B_PIN 9
  #define MATRIX_C_PIN 10

#elif __AVR_ATmega32U4__ // Micro/Leonardo
  #define ENABLE_SLEEP
  #define SLEEP_METHOD SLEEP_MODE_ADC
  /* ATmega sleep modes:
   SLEEP_MODE_IDLE     - least power savings
   SLEEP_MODE_ADC
   SLEEP_MODE_PWR_SAVE - good
   SLEEP_MODE_STANDBY
   SLEEP_MODE_PWR_DOWN - most power savings */

  #define BLUETOOTH_RX_PIN 8 
  #define BLUETOOTH_TX_PIN 7

  #define BLUETOOTH_ENABLE_PIN 6

  // #define BUTTON_INTERRUPT 0
  // #define BUTTON_INTERRUPT_PIN 3

  // For matrix buttons
  #define MATRIX_A_PIN 9
  #define MATRIX_B_PIN 10
  #define MATRIX_C_PIN 11

#else
  #warning Unkown Processor Type
  // ATTiny84
  #define ENABLE_SLEEP
  #define SLEEP_METHOD SLEEP_MODE_PWR_DOWN
  /* ATtiny sleep modes are:
   SLEEP_MODE_IDLE     - least power savings
   SLEEP_MODE_ADC
   SLEEP_MODE_PWR_DOWN - most power savings
  */
  
  #define BLUETOOTH_RX_PIN 1
  #define BLUETOOTH_TX_PIN 0

  #define BLUETOOTH_ENABLE_PIN 3

  // #define BUTTON_INTERRUPT 0
  // #define BUTTON_INTERRUPT_PIN 2

  #define PAIRING_RESET_BUTTON 7

  #define STATUS_LED_PIN 4

  // For matrix buttons
  #define MATRIX_A_PIN 8
  #define MATRIX_B_PIN 9
  #define MATRIX_C_PIN 10

  #define MATRIX_A_PCI PCINT0
  #define MATRIX_B_PCI PCINT1
  #define MATRIX_C_PCI PCINT2

  #define PAIRING_RESET_PCI PCINT3

  #define PCI_VECTOR PCIE0
  #define PCI_INTERRUPT_SFR PCMSK0
  #define PCI_ENABLE_SFR GIMSK

#endif

#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <SoftwareSerial.h>

SoftwareSerial bluetooth = SoftwareSerial(BLUETOOTH_RX_PIN, BLUETOOTH_TX_PIN); // rx, tx

struct commandWithCallback {
  char cmd[12];
  char expected[8];
  uint8_t bufferSize;
  int8_t failure_index;
  void (*callback)();
};

struct keyMap {
  uint8_t key_mask;
  uint16_t key_code;
};

                       // LOW    MASK        TARGET  TARGET
                       // LINES  BINARY      HEX     DEC     DESC
const struct keyMap keyMaps[] = {
  { 2, 128 },          // C      00000010 -> 0x80    128     Play/Pause
  { 4, 256 },          // B      00000100 -> 0x100   256     Scan Next Track
  { 8, 512 },          // A      00001000 -> 0x200   512     Scan Previous Track
  { 6, 16 },           // C B    00000110 -> 0x10    16      Volume Up
  { 14, 32 },          // C B A  00001110 -> 0x20    32      Volume Down
  { 12, 1 },           // B A    00001100 -> 0x1     1       Home
  { 10, 8 }            // C A    00001010 -> 0x8     8       Toggle Keyboard/Layout
};
#define KEY_MAPS_SIZE 7 // sizeof() can be problematic with struct arrays

char rxBuffer[32];  // was 64, but I doubt we need this

uint16_t loop_delay_ts = millis();
#define LOOP_DELAY 1000

uint16_t key_code = 0x0;
boolean key_released = false;
boolean key_triggered = false;

volatile uint8_t sleep_loop_counter = 0;
#define SLEEP_COUNTER_TRIGGER 5

const uint8_t matrix_pins[] = { MATRIX_A_PIN, MATRIX_B_PIN, MATRIX_C_PIN };
#define MATRIX_PIN_COUNT 3

// Delay for bluetooth module after responding with "AOK"
#define BLUETOOTH_RESPONSE_DELAY 100  // delay in ms
#define BLUETOOTH_RESET_DELAY    2000 // delay in ms

#ifdef DEBUG
  // We don't need key descriptions outside of debug mode
  char * getKeyCodeDescription(uint16_t key_code) {

    struct keyCodeDescription {
      uint16_t key_code;
      char description[32];
    };

    struct keyCodeDescription keyCodeDescriptions[] = {
      { 128, "Play/Pause" },
      { 256, "Scan Next Track" },
      { 512, "Scan Previous Track" },
      { 16, "Volume Up" },
      { 32, "Volume Down" },
      { 1, "Home" },
      { 8, "Keyboard Toggle" }
    };

    for(uint8_t i = 0; i < 7; i++) {
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
  delay(500);
  digitalWrite(BLUETOOTH_ENABLE_PIN, HIGH);
  bluetoothSetup();

  for (uint8_t i = 0; i < sizeof(matrix_pins); i++) {
    pinMode(matrix_pins[i], INPUT_PULLUP);
  }

  pinMode(PAIRING_RESET_BUTTON, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);

  sbi(PCI_ENABLE_SFR,PCI_VECTOR); // Turn on Pin Change interrupt
  sbi(PCI_INTERRUPT_SFR,MATRIX_A_PCI); // Which pins are affected by the interrupt
  sbi(PCI_INTERRUPT_SFR,MATRIX_B_PCI);
  sbi(PCI_INTERRUPT_SFR,MATRIX_C_PCI);
  sbi(PCI_INTERRUPT_SFR,PAIRING_RESET_PCI);
}

void loop() {
  #ifdef ENABLE_SLEEP
    sleep_disable();
    sleep_loop_counter = 0;
  #endif

  if (digitalRead(PAIRING_RESET_BUTTON) == LOW) {
    uint16_t pairing_button_press_start = millis();
    uint16_t pairing_button_press_end;
    while (digitalRead(PAIRING_RESET_BUTTON) == LOW) {
      pairing_button_press_end = millis();
    }
    if (pairing_button_press_end - pairing_button_press_start > 1000) {
      for (uint8_t i = 0; i < 10; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
      }
    }
  }

  for (uint8_t i = 0; i < MATRIX_PIN_COUNT; i++) {
    if (digitalRead(matrix_pins[i]) == LOW) {
      key_triggered = true;
      break;
    }
  }

  // if (digitalRead(BUTTON_INTERRUPT_PIN) == LOW) {
  if (key_triggered) {
    key_triggered = false;
    if (key_code == 0x0) {
      char key_mask = 0x0;
      for (uint8_t i = 0; i < MATRIX_PIN_COUNT; i++) {
        if (digitalRead(matrix_pins[i]) == LOW) {
          key_mask |= 0x01;
        }
        key_mask <<= 0x01;
      }

      key_code = keyMaskToKeyCode(key_mask);
      debug_out("Key: " + String(key_code));
      send_consumer_key(key_code);
      key_code = 0x0;
      key_released = true;
    }
  } else {
    if (key_released) {
      debug_out("Key released.");
      send_consumer_key(0x0);
      key_released = false;
      sleep_loop_counter = 0;
    }
  }

  if (millis() > (loop_delay_ts + LOOP_DELAY)) {
    loop_delay_ts = millis();
    debug_out(millis());

    #ifdef ENABLE_SLEEP
      if (!key_released) { // dont sleep while button pressed
        sleep_loop_counter++;
        if (sleep_loop_counter >= SLEEP_COUNTER_TRIGGER) {
          debug_out("Going to sleep.");
          delay(100);
          sleep_loop_counter = 0;
          set_sleep_mode(SLEEP_METHOD);
          sleep_enable();
          sleep_cpu();
          sleep_disable();
        }
      }
    #endif
  }
}

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
  #else
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

  const struct commandWithCallback cmds[] PROGMEM = {
    { "GM", "Pair", 4, 0 },
    { "GH", "0200", 4, 1 },
    { "GN", "PHTEVEN", 7, 2 },
    { "GA", "2", 1, 3 },
    { "G~", "6", 1, -1, &setHidMode },
    { "---", "END", 3 }
  };

  const struct commandWithCallback failureCmds[] PROGMEM = {
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
      } else if (cmds[i].callback) {
        debug_out("executing failure callback");
        failureCmds[cmds[i].failure_index].callback();
      } else {
        debug_out("no failure action to take");
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
  debug_out("Sending code to bluetooth. High Byte: " + String(high_byte, HEX) + ", Low Byte: " + String(low_byte, HEX) + "\n");
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
