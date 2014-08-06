/*
  TODO:
  Scan for key presses using interrupt ISR
  status indicator lights
  code efficiency, DRY
*/

#define DEBUGGING_MODE true
#ifdef DEBUGGING_MODE
  #define debug_out(msg) Serial.println(msg)
#else
  #define debug_out(msg) // no-op
#endif

#define MCU_BOARDUINO 0
#define MCU_ARDUINO_MICRO 1
#define TARGET_MCU MCU_ARDUINO_MICRO

#if TARGET_MCU == MCU_BOARDUINO

  #define BLUETOOTH_RX_PIN 5
  #define BLUETOOTH_TX_PIN 4

  #define BLUETOOTH_ENABLE_PIN 6

  #define MATRIX_INTERRUPT_PIN 3
  #define MATRIX_A_PIN 8
  #define MATRIX_B_PIN 9
  #define MATRIX_C_PIN 10

#elif TARGET_MCU == MCU_ARDUINO_MICRO

  #define BLUETOOTH_RX_PIN 8
  #define BLUETOOTH_TX_PIN 7

  #define BLUETOOTH_ENABLE_PIN 6

  #define MATRIX_INTERRUPT_PIN 3
  #define MATRIX_A_PIN 9
  #define MATRIX_B_PIN 10
  #define MATRIX_C_PIN 11

#endif

#include <SoftwareSerial.h>

SoftwareSerial bluetooth = SoftwareSerial(BLUETOOTH_RX_PIN, BLUETOOTH_TX_PIN); // rx, tx

boolean key_pressed = false;

uint8_t matrix_pins[] = { MATRIX_A_PIN, MATRIX_B_PIN, MATRIX_C_PIN };

// Delay for bluetooth module after responding with "AOK"
#define BLUETOOTH_RESPONSE_DELAY 100  // delay in ms
#define BLUETOOTH_RESET_DELAY  2000   // delay in ms

char rxBuffer[64];

void setup() {
  #ifdef DEBUGGING_MODE
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
  if (digitalRead(MATRIX_INTERRUPT_PIN) == LOW) {
    if (key_pressed == false) {
      key_pressed = true;

      debug_out("Interrupt pin low");

      char key_mask = 0x0;
      for (uint8_t i = 0; i < sizeof(matrix_pins); i++) {
        if (digitalRead(matrix_pins[i]) == LOW) {
          debug_out("Pin " + String(matrix_pins[i], DEC) + " low");
          key_mask |= 0x01;
        } 
        key_mask <<= 0x01;
      }

      debug_out("Key Mask: " + String(key_mask, BIN));

      uint16_t keycode = 0;
      switch (key_mask) {
        case 0b00000010: // C
          keycode = 128; // HEX 0x80, DEC 128
          debug_out("Play/Pause");
          break;
        case 0b00000100: // B
          keycode = 256; // HEX 0x100, DEC 256
          debug_out("Scan Next Track");
          break;
        case 0b00001000: // A
          keycode = 512; // HEX 0x200, DEC 512
          debug_out("Scan Previous Track");
          break;
        case 0b00000110:  // C + B
          keycode = 16; // HEX 0x10, DEC 16
          debug_out("Volume Up");
          break;
        case 0b00001110:  // C + B + A
          keycode = 32; // HEX 0x20, DEC 32
          debug_out("Volume Down");
          break;
        case 0b00001100:  // B + A
          keycode = 1; // HEX 0x1, DEC 1
          debug_out("Home");
          break;
        case 0b00001010:  // C + A
//          keycode = 8; // HEX 0x8, DEC 8
          debug_out("Pair / Keyboard Layout (Virtual Apple Keyboard Toggle)");
          enter_pairing_mode();
          break;
      }

      if (keycode > 0) {
        send_consumer_key(keycode);
      }
    }
  } else {
    if (key_pressed == true) {
      release_consumer_key();
      debug_out("Key released\n");
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
  delay(2000);
  debug_out("Setting up bluetooth module");
  digitalWrite(BLUETOOTH_ENABLE_PIN, HIGH);
  delay(500);
  bluetooth.write('$');
  bluetooth.write('$');
  bluetooth.write('$');
  delay(BLUETOOTH_RESPONSE_DELAY);
  bluetoothReceive(rxBuffer);

  bluetooth.print("SM,4");
  bluetooth.write('\r');
  delay(BLUETOOTH_RESPONSE_DELAY);
  bluetoothReceive(rxBuffer);
  
  bluetooth.print("SH,0200");
//  bluetooth.print("SH,0207");
  bluetooth.write('\r');
  delay(BLUETOOTH_RESPONSE_DELAY);
  bluetoothReceive(rxBuffer);
  
  bluetooth.print("S-,PHTEVEN");
  bluetooth.write('\r');
  delay(BLUETOOTH_RESPONSE_DELAY);
  bluetoothReceive(rxBuffer);
  
  bluetooth.print("SA,2");
  bluetooth.write('\r');
  delay(BLUETOOTH_RESPONSE_DELAY);
  bluetoothReceive(rxBuffer);
  
  bluetooth.print("S~,6");
  bluetooth.write('\r');
  delay(BLUETOOTH_RESPONSE_DELAY);
  bluetoothReceive(rxBuffer);
  
  bluetooth.print("R,1");
  bluetooth.write('\r');
  bluetoothReceive(rxBuffer);
  debug_out(rxBuffer);
  delay(BLUETOOTH_RESET_DELAY);

  bluetooth.write('$');
  bluetooth.write('$');
  bluetooth.write('$');
  delay(BLUETOOTH_RESPONSE_DELAY);
  bluetoothReceive(rxBuffer);
  
  debug_out("Trying to reconnect");

  bluetooth.print("C");
  bluetooth.write('\r');
  delay(BLUETOOTH_RESPONSE_DELAY);
  bluetoothReceive(rxBuffer);
  debug_out(String(rxBuffer));

  debug_out("Setup complete");
}

 void enter_pairing_mode() {
  debug_out("Pairing mode");
  digitalWrite(BLUETOOTH_ENABLE_PIN, LOW);
  delay(500);
  digitalWrite(BLUETOOTH_ENABLE_PIN, HIGH);
  bluetooth.write('$');
  bluetooth.write('$');
  bluetooth.write('$');
  delay(BLUETOOTH_RESPONSE_DELAY);
  bluetoothReceive(rxBuffer);
  bluetooth.print("SM,6");   // trigger pairing mode (SM,6)
  bluetooth.write('\r');
  
  bluetooth.print("R,1");
  bluetooth.write('\r');
  delay(BLUETOOTH_RESET_DELAY);
  debug_out("Pairing mode complete");
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

void release_consumer_key() {
  debug_out("Sending key release code to code to bluetooth");
  bluetooth.write(0xFD); // Consumer Report
  bluetooth.write(0x03);
  bluetooth.write(0x03);
  bluetooth.write((byte)0x00); // high byte
  bluetooth.write((byte)0x00); // low byte
} 

uint8_t bluetoothReceive(char * dest)
{
  int timeout = 1000;
  char c;
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
  char c;

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
