// This project uses the Arduino Nano which may require "ATmega328P (Old Bootloader)" to be set as the processor for successful upload
// This project uses the Serial Monitor to operate the program

/* Pins to SN74HC595 Shift Registers */
#define SHIFT_DATA_PIN 4 // Connect to the SER (Serial) Pin
#define SHIFT_LATCH_PIN 3 // Connect to the RCLK (Storage Register Clock) Pin
#define SHIFT_CLK_PIN 2 // Connect to the SRCLK (Shift Register Clock) Pin
int IO_PINS[] = {12,11,10,9,8,7,6,5}; // I/O0 to I/O7 going left to right

/* Pins to AT28C16 EEPROM */
#define WRITE_ENABLE_PIN 13 // Connected to capacitor pulse "switch"

/* Serial UI Variables */
char endMarker = '\n'; // For serial reading. Must be using Serial Monitor end marker set to New Line. Single quotes declare a char
String inputBuffer; // Stores what was input into Serial
String mode; // Programmer mode [read, write]
String modeReset; // Used if an invalid input is received
int firstBlock; // First address block [1 to 8]
int lastBlock; // Last address block [firstBlock to 8]
int sel; // Program select [1 to 16]

void setAddress(int address, bool outputEnable) {
  // Set the four unused shift register pins to 0
  address = (outputEnable ? (address & 0x7FFF) : (address | 0x8000)) & 0x87FF;
  // Shift first byte:
  shiftOut(SHIFT_DATA_PIN, SHIFT_CLK_PIN, LSBFIRST, address);
  // Shift second byte, set the MSB to ~OE, and set the unused shift register pins to 0:
  shiftOut(SHIFT_DATA_PIN, SHIFT_CLK_PIN, LSBFIRST, address >> 8 );
  // Latch data out from the shift register to the storage register which is tied to the output pins
  digitalWrite(SHIFT_LATCH_PIN, LOW);
  digitalWrite(SHIFT_LATCH_PIN, HIGH);
  digitalWrite(SHIFT_LATCH_PIN, LOW);
}

void readEnable() {
  // setAddress MUST be AFTER pinMode when READING the EEPROM to avoid both the microcontroller and EEPROM set to OUTPUT at the same time
  // The EEPROM address doesn't matter here, just setting the outputEnable pin
  for (int pin = 0; pin < 8; pin++) {
    pinMode(IO_PINS[pin], INPUT);
  }
  setAddress(0x0, true);
}

void writeEnable() {
  // setAddress MUST be BEFORE pinMode when WRITING to the EEPROM to avoid both the microcontroller and EEPROM set to OUTPUT at the same time
  // The EEPROM address doesn't matter here, just setting the outputEnable pin
  setAddress(0x0, false);
  for (int pin = 0; pin < 8; pin++) {
    pinMode(IO_PINS[pin], OUTPUT);
  }
}

byte readAddress(int address) { /* ENSURE THAT readEnable() HAS BEEN RUN BEFORE CALLING THIS FUNCITON */
  setAddress(address, true);
  byte data = 0;
  for (int pin = 7; pin >= 0; pin--) {
    data = (data << 1) + digitalRead(IO_PINS[pin]);
  }
  return data;
}

void writeData(int address, byte data) { /* ENSURE THAT writeEnable() HAS BEEN RUN BEFORE CALLING THIS FUNCTION */
  // Max address on the 16-bit shift register is 0xFFFF where 0x8000 is the output enable bit
  // Max address on a 16K EEPROM is the 11 bit address 0x7FF = 2047 in decimal
  // Max output on the EEPROM is the byte 0xFF
  setAddress(address, false); // Set address on shift register
  for (int pin = 0; pin < 8; pin++) { // Write data to each arduino pin using LSB mask (data & 1), then shifting out the LSB
    digitalWrite(IO_PINS[pin], data & 1);
    data = data >> 1;
  }
  // Save the data to the EEPROM
  digitalWrite(WRITE_ENABLE_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(WRITE_ENABLE_PIN, HIGH);
  delay(5);
}

void getInput() {
  inputBuffer = Serial.readStringUntil(endMarker);
  inputBuffer.trim();
  inputBuffer.toLowerCase();
}

bool readEEPROM() {
/* Serial UI */
  // First Address Block Input
  Serial.println("Enter the First Address Block: [1–8, all, cancel]");
  while (Serial.available() == 0) {}
  getInput();
  Serial.println(">> " + inputBuffer);
  if (inputBuffer == "cancel") return true;
  if (inputBuffer == "all") {
    firstBlock = 1;
    lastBlock = 8;
  }
  else {
    // First Address Block Processing
    firstBlock = inputBuffer.toInt(); // Anything not an integer will return as 0
    if ( !((firstBlock <= 8) && (firstBlock > 0)) ) {
      Serial.println(">> Invalid Number. Retry.");
      return false;
    }
    // Last Address Block
    Serial.println("Enter the Last Address Block: [" + String(firstBlock) + "–8, cancel]");
    while (Serial.available() == 0) {}
    getInput();
    Serial.println(">> " + inputBuffer);
    // Last Address Block Processing
    if (inputBuffer == "cancel") return true;
    lastBlock = inputBuffer.toInt(); // Anything not an integer will return as 0
    if ( !((lastBlock <= 8) && (lastBlock >= firstBlock)) ) {
      Serial.println(">> Invalid Number. Retry.");
      return false;
    }
  }

/* Read EEPROM */
  char buf[64];
  sprintf(buf, "\r\nReading EEPROM Addresses %03X to %03X...", (firstBlock - 1) * 256, (lastBlock * 256) - 1);
  Serial.println(buf);
  delay(400);
  readEnable();
  // Each Address Block = 256 bytes, Boundries: 1 <= firstBlock <= lastBlock <= 8
  int base = (firstBlock - 1) * 256;
  for (int block = firstBlock; block <= lastBlock; block++) {
    Serial.println("");
    for (base; base < block * 256; base += 16) { // Read the byte output of 16 addresses starting from the base address
      byte data[16];
      for (int i = 0; i <= 15; i += 1) {
        data[i] = readAddress(base + i);
      }
      char buf[64];
      sprintf(buf, "%03X:  %02X %02X %02X %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X %02X %02X %02X",
              base, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
              data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);
      Serial.println(buf); // Print the formatted line of 16 bytes
    }
  }
  Serial.println(">> Read Finished");
  return true;
}

bool writeEEPROM() {
/* Serial UI */
  Serial.println("Select Program: [1, 2, 15, 16, cancel]");
  while (Serial.available() == 0) {}
  getInput();
  Serial.println(">> " + inputBuffer);
  if (inputBuffer == "cancel") return true;
  sel = inputBuffer.toInt(); // Anything not an integer will return as 0
  if ( !(sel == 1 || sel == 2 || sel == 15 || sel == 16) ) {
    Serial.println(">> Invalid Number. Retry.");
    return false;
  }

/* Write EEPROM */
  writeEnable();
  Serial.println("\r\nWriting Program " + String(sel) + " to EEPROM...");
  long int t = millis();
  if (sel == 1) {
    /* Common Cathode 7-Segment Display Binary to Hex Decoder */
    byte program[] = {0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B, 0x77, 0x1F, 0x4E, 0x3D, 0x4F, 0x47};
    for (int address = 0; address < sizeof(program); address++) writeData(address, program[address]);
  }
  else if (sel == 2) {
    /* Common Anode 7-Segment Display Binary to Hex Decoder */
    byte program[] = {0x81, 0xCF, 0x92, 0x86, 0xCC, 0xA4, 0xA0, 0x8F, 0x80, 0x84, 0x88, 0xE0, 0xB1, 0xC2, 0xB0, 0xB8};
    for (int address = 0; address < sizeof(program); address++) writeData(address, program[address]);
  }
  else if (sel == 15) {
    /* Test Program */
    for (int address = 0x100; address <= 0x1FF; address++) writeData(address, address);
    for (int address = 0x700; address <= 0x7FF; address++) writeData(address, address);
  }
  else if (sel == 16) {
    /* Clear EEPROM */
    for (int address = 0; address <= 0x7FF; address++) writeData(address, 0xFF);
  } else Serial.println("Incorrect program selection value for writeEEPROM()");
  Serial.println(">> Write Finished in " + String(millis() - t) + " ms");
  return true;
}

void setup() {
/* Pin Mode Setup */
  pinMode(SHIFT_DATA_PIN, OUTPUT);
  pinMode(SHIFT_CLK_PIN, OUTPUT);
  pinMode(SHIFT_LATCH_PIN, OUTPUT);

  digitalWrite(WRITE_ENABLE_PIN, HIGH); // Set the default HIGH state of the ~WE pin
  pinMode(WRITE_ENABLE_PIN, OUTPUT); // then set ~WE pin as output

/* Program Setup */
  Serial.begin(57600);
  delay(100); // Give the arduino some time to start up and sync before running the program
  Serial.println("\r\n---------Programmer Connected---------\r\n");
  Serial.println("Enter Mode: [read, write]");
}

void loop() {
  if (Serial.available() || modeReset.equals("read") || modeReset.equals("write")) {
    getInput();
    if (inputBuffer.equals("read") || modeReset.equals("read")) {
      if (inputBuffer.equals("read")) Serial.println(">> read");
      if (readEEPROM() == true) {
        modeReset = "";
        Serial.println("\r\nEnter Mode: [read, write]");
      } else modeReset = "read";
    }
    else if (inputBuffer.equals("write") || modeReset.equals("write")) {
      if (inputBuffer.equals("write")) Serial.println(">> write");
      if (writeEEPROM() == true) {
        modeReset = "";
        Serial.println("\r\nEnter Mode: [read, write]");
      } else modeReset = "write";
    }
    else {
      Serial.println(">> " + inputBuffer);
      Serial.println(">> Invalid Mode. Retry.");
      delay(400);
      Serial.println("\r\nEnter Mode: [read, write]");
    }
  }
}

// Additional Notes:
// Shift Register Format = _OE,X,X,X, X,A10,A9,A8, A7,A6,A5,A4, A3,A2,A1,A0
// EEPROM Address Format = A10,A9,A8, A7,A6,A5,A4,A3,A2,A1,A0
// "bool outputEnable" in setAddress() will overwrite the MSB (outputEnable pin) of the 16 bit "int address" (which gets shifted to the 16 bit shift register), but remeber that the actual EEPROM address size is only 11 bits long
// The Output Enable pin is active low
// if outputEnable == true, then replace (outputEnable ? 0 : 1) with 0
// else, replace (outputEnable ? 0 : 1) with 1
// & 0x7F = bit-wise AND B01111111; Sets most significant bit to 0 without affecting the rest of the byte
// | 0x80 = bit-wise OR B10000000; Sets most significant bit to 1 without affecting the rest of the byte
// & 1 is the binary identity property which will always output the bit of "Data" that corresponds to the bit place that the 1 is located in. Also called "bitmasking"

// How to Program an AT28C16 EEPROM:
// Before any writing can begin:
//  Set EEPROM Output Enable pin (OE Active LOW) to HIGH
//  Set EEPROM Write Enable pin (WE Active LOW) to HIGH
// Setting an Address and Output:
//  Set all EEPROM address pins to the desired address and all EEPROM data pins to the corresponding output
// Writing to the EEPROM:
//  Set EEPROM Write Enable pin (WE Active LOW) to LOW, then to HIGH. The time that Write Enable is LOW must be between 100ns and 1µs
// Repeat "Setting an Address and Output" and "Writing to that Address" until all desired addresses are written