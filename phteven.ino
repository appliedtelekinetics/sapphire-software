/*
  TODO:

  bluetooth module stored config and reset
  bluetooth host autoconnect
  status indicator lights
*/

#define MCU_BOARDUINO 0
#define MCU_ARDUINO_MICRO 1
#define MCU_ATTINY84_8MHZ 2
#define TARGET_MCU MCU_BOARDUINO

#if TARGET_MCU == MCU_BOARDUINO

  #define BLUETOOTH_RX_PIN 5
  #define BLUETOOTH_TX_PIN 4

  #define BLUETOOTH_ENABLE_PIN 6

  #define MATRIX_INTERRUPT_PIN 3
  #define MATRIX_A_PIN 8
  #define MATRIX_B_PIN 9
  #define MATRIX_C_PIN 10

#elif TARGET_MCU == MCU_ARDUINO_MICRO

  #define BLUETOOTH_RX_PIN 5
  #define BLUETOOTH_TX_PIN 4

  #define BLUETOOTH_ENABLE_PIN 6

  #define MATRIX_INTERRUPT_PIN 3
  #define MATRIX_A_PIN 8
  #define MATRIX_B_PIN 9
  #define MATRIX_C_PIN 10

#elif TARGET_MCU == MCU_ARDUINO_MICRO
#endif

#include <SoftwareSerial.h>

SoftwareSerial bluetooth = SoftwareSerial(BLUETOOTH_RX_PIN, BLUETOOTH_TX_PIN); // rx, tx

boolean key_pressed = false;

uint8_t matrix_pins[] = { MATRIX_A_PIN, MATRIX_B_PIN, MATRIX_C_PIN };

void setup() {
  delay(1000);

  Serial.begin(9600);
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
  if (digitalRead(MATRIX_INTERRUPT_PIN) == LOW) {
    if (key_pressed == false) {
      key_pressed = true;

      Serial.println("Interrupt pin low");

      char key_mask = 0x0;
      for (uint8_t i = 0; i < sizeof(matrix_pins); i++) {
        if (digitalRead(matrix_pins[i]) == LOW) {
          Serial.println("Pin " + String(matrix_pins[i], DEC) + " low");
          key_mask |= 0x01;
        } 
        key_mask <<= 0x01;
      }

      Serial.println(String(key_mask, BIN));

      uint16_t keycode = 0;
      switch (key_mask) {
        case 0b00000010: // C
          keycode = 128; // HEX 0x80, DEC 128
          Serial.println("Play/Pause");
          break;
        case 0b00000100: // B
          keycode = 256; // HEX 0x100, DEC 256
          Serial.println("Scan Next Track");
          break;
        case 0b00001000: // A
          keycode = 512; // HEX 0x200, DEC 512
          Serial.println("Scan Previous Track");
          break;
        case 0b00000110:  // C + B
          keycode = 16; // HEX 0x10, DEC 16
          Serial.println("Volume Up");
          break;
        case 0b00001110:  // C + B + A
          keycode = 32; // HEX 0x20, DEC 32
          Serial.println("Volume Down");
          break;
        case 0b00001100:  // B + A
          keycode = 1; // HEX 0x1, DEC 1
          Serial.println("Home");
          break;
        case 0b00001010:  // C + A
          keycode = 8; // HEX 0x8, DEC 8
          Serial.println("Pair / Keyboard Layout (Virtual Apple Keyboard Toggle)");
          // enter_pairing_mode();
          break;
      }

      if (keycode > 0) {
        send_consumer_key(keycode);
      }
    }
  } else {
    if (key_pressed == true) {
      release_consumer_key();
      Serial.println("Key released\n");
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
void  bluetoothSetup() {
  delay(5000);
  Serial.println("Setting up bluetooth module");
  digitalWrite(BLUETOOTH_ENABLE_PIN, HIGH);
  delay(500);
  bluetooth.print("$$$"); // Command Mode 
  while (bluetooth.available()) { Serial.write(bluetooth.read()); }
  bluetooth.println("SF,1"); // Reset to factory defaults
  while (bluetooth.available()) { Serial.write(bluetooth.read()); }
  delay(200);
  bluetooth.println("SH,0207"); // connect to 3 stored hosts max (0200 default)
  // while (bluetooth.available()) { Serial.write(bluetooth.read()); }
  bluetooth.println("S-,PHTEVEN");
  while (bluetooth.available()) { Serial.write(bluetooth.read()); }
  bluetooth.println("SN,PHTEVEN");
  while (bluetooth.available()) { Serial.write(bluetooth.read()); }
  // bluetooth.println("SP,1234");
  // while (bluetooth.available()) { Serial.write(bluetooth.read()); }
  // bluetooth.println("SA,1");
  // while (bluetooth.available()) { Serial.write(bluetooth.read()); }
  // bluetooth.println("SM,0");
  // while (bluetooth.available()) { Serial.write(bluetooth.read()); }
  bluetooth.println("S~,6"); // HID Mode
  while (bluetooth.available()) { Serial.write(bluetooth.read()); }
  bluetooth.println("R,1"); // Reboot
  while (bluetooth.available()) { Serial.write(bluetooth.read()); }
  delay(500);
}

// void enter_pairing_mode() {
//   Serial.println("Pairing mode");
//   bluetooth.println("K,1"); // kill active connection
//   bluetooth.print("$$$");        // enter command mode
//   bluetooth.println("SM,6");   // trigger pairing mode (SM,6)
// }

void send_consumer_key(uint16_t keycode) {
  byte high_byte = highByte(keycode);
  byte low_byte = lowByte(keycode);
  Serial.println("Sending code to bluetooth. High Byte: " + String(high_byte, HEX) + ", Low Byte: " + String(low_byte, HEX));
  bluetooth.write(0xFD); // Consumer Report
  bluetooth.write(0x03);
  bluetooth.write(0x03);
  bluetooth.write((byte)low_byte); // low byte
  bluetooth.write((byte)high_byte); // high byte
}

void release_consumer_key() {
  Serial.println("Sending key release code to code to bluetooth");
  bluetooth.write(0xFD); // Consumer Report
  bluetooth.write(0x03);
  bluetooth.write(0x03);
  bluetooth.write((byte)0x00); // high byte
  bluetooth.write((byte)0x00); // low byte
} 
