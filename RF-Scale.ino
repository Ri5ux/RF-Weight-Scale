/** RF Scale Controller (1/4/2021) **/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>
#include <HX711.h>

#define SDA_PORT PORTC
#define SDA_PIN 4 //4 within Port C
#define SCL_PORT PORTC
#define SCL_PIN 5 //5 within Port C
#define I2C_SLOWMODE 1

#include <SoftI2CMaster.h>

/** Battery I2c **/
#define bufferLen 32
byte address = 0;
uint8_t i2cBuffer[bufferLen];

#define MODE 0x03
#define VOLTAGE 0x09
#define TEMPERATURE 0x08
#define CURRENT 0x0a 
#define CAPACITY 0x10
#define TIME_TO_FULL 0x13
#define CHARGE 0x0d
#define CHARGE_VOLTAGE 0x15
#define CHARGE_CURRENT 0x14
#define TIME_TO_EMPTY 0x12
#define STATUS 0x16
#define CYCLE_COUNT 0x17
#define DESIGN_CAPACITY 0x18
#define DESIGN_VOLTAGE 0x19
#define MANUFACTURE_DATE 0x1b
#define SERIAL_NUMBER 0x1c
#define DEVICE_NAME 0x21
#define DEVICE_CHEMISTRY 0x22
#define MANUFACTURER_NAME 0x20
#define MANUFACTURER_DATA 0x23
#define MANUFACTURER_INFO 0x25
#define SPECIFICATION_INFO 0x1a
#define CELL4_VOLTAGE 0x3C
#define CELL3_VOLTAGE 0x3D
#define CELL2_VOLTAGE 0x3E
#define CELL1_VOLTAGE 0x3F

const int PIN_HX711_CLK = 2;
const int PIN_HX711_DAT = 3;

/** RF (NRF24L01) **/
RF24 radio(7, 8); //CE, CSN
const byte addresses[][6] = {"10001", "10002"};
int failedTransmissions = 0;

/** (HX711) **/
const int SCALE_SAMPLES = 4;
HX711 scale;
float calibration_factor = -5402.00; //-5212
float scaleValue = 0;
float scaleValueLast = 0;
long zero_factor;
boolean calibrationMode = false;

/** Display **/
int prevVals[6] = { };

/** Sequential reset function **/
int rsFlags[6] = { 0, 0, 0, 0, 0, 0 };
long timeRSTrigger = 0;
const int intervalReset = 5000;
const int seqWtDiffMax = 3;
const int seqWtDiffMin = 1;

/** Common variables **/
boolean reset = false;
long previousMillis = 0;
boolean sendPing = false;
boolean batteryDischarged = false;
uint8_t battAnimationState = 0;
uint8_t startAnimationState = 0;
boolean started = false;

void(* resetFunc) (void) = 0;

void setup() {
  Serial.begin(9600);
  printf_begin();
  Serial.println("RF Scale Controller");
  Serial.println("ASX Electronics");
  Serial.println("");
  Serial.println("[i2c] init");
  if (!i2c_init()) {
    Serial.println("[i2c] failed");
  } else {
    setAddress(11);
    Serial.println("[i2c] started");
  }

  Serial.println("[battery] init");
  printBatteryData();
  
  Serial.println("[radio] init");
  radio.begin();
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
  radio.setPALevel(RF24_PA_LOW);
  radio.printDetails();
  
  Serial.println("[scale] init");
  startScale();
  
  Serial.println("[init] done");
}

void loop() {
  handleRadio();
  
  if (Serial.available() > 0) {
   handleSerialCommands();
  } else {
    long currentMillis = millis();
  
    if(currentMillis - previousMillis > 100) {
      previousMillis = currentMillis;

      if (started) {
      if (!batteryDischarged) {
        scaleValueLast = scaleValue;
        scaleValue = scale.get_units(SCALE_SAMPLES);
        updateWeight();
        handleResetSequence();
      } else {
        if (battAnimationState == 0) updateDisplay(11, 15, 15, 15, 15, 15);
        if (battAnimationState == 1) updateDisplay(10, 11, 15, 15, 15, 15);
        if (battAnimationState == 2) updateDisplay(15, 10, 11, 15, 15, 15);
        if (battAnimationState == 3) updateDisplay(15, 15, 10, 11, 15, 15);
        if (battAnimationState == 4) updateDisplay(15, 15, 15, 10, 11, 15);
        if (battAnimationState == 5) updateDisplay(15, 15, 15, 15, 10, 11);
        if (battAnimationState == 6) updateDisplay(15, 15, 15, 15, 15, 10);
        if (battAnimationState == 7) updateDisplay(15, 15, 15, 15, 15, 15);
        battAnimationState++;

        if (battAnimationState > 15) {
          battAnimationState = 0;
        }
      }
      } else {
        if (startAnimationState == 0) updateDisplay(6, 15, 15, 15, 15, 15);
        if (startAnimationState == 1) updateDisplay(15, 6, 15, 15, 15, 15);
        if (startAnimationState == 2) updateDisplay(15, 15, 6, 15, 15, 15);
        if (startAnimationState == 3) updateDisplay(15, 15, 15, 6, 15, 15);
        if (startAnimationState == 4) updateDisplay(15, 15, 15, 15, 6, 15);
        if (startAnimationState == 5) updateDisplay(15, 15, 15, 15, 15, 6);
        if (startAnimationState == 6) updateDisplay(5, 15, 15, 15, 15, 6);
        if (startAnimationState == 7) updateDisplay(15, 5, 15, 15, 15, 6);
        if (startAnimationState == 8) updateDisplay(15, 15, 5, 15, 15, 6);
        if (startAnimationState == 9) updateDisplay(15, 15, 15, 5, 15, 6);
        if (startAnimationState == 10) updateDisplay(15, 15, 15, 15, 5, 6);
        if (startAnimationState == 11) updateDisplay(4, 15, 15, 15, 5, 6);
        if (startAnimationState == 12) updateDisplay(15, 4, 15, 15, 5, 6);
        if (startAnimationState == 13) updateDisplay(15, 15, 4, 15, 5, 6);
        if (startAnimationState == 14) updateDisplay(15, 15, 15, 4, 5, 6);
        if (startAnimationState == 15) updateDisplay(3, 15, 15, 4, 5, 6);
        if (startAnimationState == 16) updateDisplay(15, 3, 15, 4, 5, 6);
        if (startAnimationState == 17) updateDisplay(15, 15, 3, 4, 5, 6);
        if (startAnimationState == 18) updateDisplay(2, 15, 3, 4, 5, 6);
        if (startAnimationState == 19) updateDisplay(15, 2, 3, 4, 5, 6);
        if (startAnimationState == 20) updateDisplay(1, 2, 3, 4, 5, 6);
        if (startAnimationState > 20) updateDisplay(0, 0, 0, 0, 0, 0);
        
        startAnimationState++;

        if (startAnimationState > 25) {
          started = true;
        }
      }
    }
  }

  if (sendPing) {
    respondPing();
    sendPing = false;
  }

  if (reset) {
    resetFunc();
  }
}

float mvToV(int mv) {
  return mv / 1000.0;
}

void printKey(char* unit) {
  Serial.print(unit);
  Serial.print(':');
  Serial.print(' ');
}

void printValue(float value, char* unit) {
  Serial.print(value);
  Serial.print(' ');
  Serial.println(unit);
}

void setAddress(byte addr) {
  address = addr;
  Serial.print("Address Set: ");
  Serial.println(address);
}

int fetchWord(byte func) {
  i2c_start(address << 1 | I2C_WRITE);
  i2c_write(func);
  i2c_rep_start(address << 1 | I2C_READ);
  
  byte b1 = i2c_read(false);
  byte b2 = i2c_read(true);
  i2c_stop();
  
  return (int) b1| (((int) b2) << 8);
}

uint8_t readBlock(uint8_t command, uint8_t* blockBuffer, uint8_t blockBufferLen) {
    uint8_t x;
    uint8_t num_bytes;
    
    i2c_start((address << 1) + I2C_WRITE);
    i2c_write(command);
    i2c_rep_start((address << 1) + I2C_READ);
    
    num_bytes = i2c_read(false);
    num_bytes = constrain(num_bytes, 0, blockBufferLen - 2);
    
    for (x = 0; x < num_bytes - 1; x++) {
      blockBuffer[x] = i2c_read(false);
    }
    
    blockBuffer[x++] = i2c_read(true);
    blockBuffer[x] = 0;
    
    i2c_stop();
    
    return num_bytes;
}

String convertDateCode(int dateCode) {
  String date = "";
  
  int mday = dateCode & B00011111;
  int mmonth = dateCode >> 5 & B00001111;
  int myear = 1980 + (dateCode >> 9 & B01111111);
  
  date += mmonth;
  date += "-";
  date += mday;
  date += "-";
  date += myear;

  return date;
}

void printActiveStatusFlags(uint8_t flags) {
    if (flags & 1 << 4) Serial.print("DISCHARGED ");
    if (flags & 1 << 5) Serial.print("CHARGED ");
    if (flags & 1 << 6) Serial.print("DISCHARGING ");
    if (flags & 1 << 7) Serial.print("INIT ");
    if (flags & 1 << 8) Serial.print("REM_TIME_ALARM ");
    if (flags & 1 << 9) Serial.print("REM_CAPACITY_ALARM ");
    if (flags & 1 << 11) Serial.print("TERMINATE_DISCHARGE_ALARM ");
    if (flags & 1 << 12) Serial.print("OVERTEMP_ALARM ");
    if (flags & 1 << 14) Serial.print("TERMINATE_CHARGE_ALARM ");
    if (flags & 1 << 15) Serial.print("OVERCHARGE_ALARM ");
    Serial.println();
}

void printBatteryData() {
  uint8_t length_read = 0;

  printKey("Manufacturer");
  length_read = readBlock(MANUFACTURER_NAME, i2cBuffer, bufferLen);
  Serial.write(i2cBuffer, length_read);
  Serial.println();
  printKey("Device");
  length_read = readBlock(DEVICE_NAME, i2cBuffer, bufferLen);
  Serial.write(i2cBuffer, length_read);
  Serial.println();
  printKey("SN");
  Serial.println(fetchWord(SERIAL_NUMBER));
  printKey("Manufactured");
  int dateCode = fetchWord(MANUFACTURE_DATE);
  Serial.println(convertDateCode(dateCode));
  printKey("Type");
  length_read = readBlock(DEVICE_CHEMISTRY, i2cBuffer, bufferLen);
  Serial.write(i2cBuffer, length_read);
  Serial.println();
  printKey("DesignVoltage");
  printValue(mvToV(fetchWord(DESIGN_VOLTAGE)), "V");
  printKey("DesignCapacity");
  printValue(fetchWord(DESIGN_CAPACITY), "mAh");
  printKey("Status");
  printActiveStatusFlags(fetchWord(STATUS));
  printKey("Voltage");
  printValue(mvToV(fetchWord(VOLTAGE)), "V");
  printKey("Current");
  printValue(fetchWord(CURRENT), "ma");
  printKey("Capacity");
  printValue(fetchWord(CAPACITY), "mAh");
  printKey("C1");
  printValue(mvToV(fetchWord(CELL1_VOLTAGE)), "V");
  printKey("C2");
  printValue(mvToV(fetchWord(CELL2_VOLTAGE)), "V");
  printKey("C3");
  printValue(mvToV(fetchWord(CELL3_VOLTAGE)), "V");
  printKey("C4");
  printValue(mvToV(fetchWord(CELL4_VOLTAGE)), "V");
  printKey("Temp");
  printValue(((float) fetchWord(TEMPERATURE)) / 10.0 - 273.15, "C");
  printKey("Charge");
  printValue(fetchWord(CHARGE), "%");
  printKey("ChargeCurrent");
  printValue(fetchWord(CHARGE_CURRENT), "ma");
  printKey("ChargeVoltage");
  printValue(mvToV(fetchWord(CHARGE_VOLTAGE)), "V");
  printKey("Cycles");
  printValue(fetchWord(CYCLE_COUNT), "");
  printKey("TTF");
  printValue(fetchWord(TIME_TO_FULL), "minutes");
  printKey("TTE");
  printValue(fetchWord(TIME_TO_EMPTY), "minutes");
}

void monitorBattery() {
  uint8_t flags = fetchWord(STATUS);
  if (flags & 1 << 4) {
    batteryDischarged = true;
  }
}

void startScale() {
  scale.begin(PIN_HX711_DAT, PIN_HX711_CLK);
  scale.set_scale(calibration_factor);
  scale.tare();
  zero_factor = scale.read_average();
}

void handleRadio() {
  /** Unfortunately seems to be required **/
  delay(1);
  radio.startListening();
  
  if (radio.available()) {
    char text[32] = {0};
    radio.read(&text, sizeof(text));
    Serial.println(text);
    sendPing = true;
  }
  
  /** Unfortunately seems to be required **/
  delay(1);
  radio.stopListening();
}

void handleSerialCommands() {
  if (calibrationMode) {
    char incomingCharacter = Serial.read();
    switch (incomingCharacter) {
    case '+':
      calibration_factor += 1;
      printCalibrationFactor();
      break;
    case '-':
      calibration_factor -= 1;
      printCalibrationFactor();
      break;
    case '.':
      calibration_factor += 10;
      printCalibrationFactor();
      break;
    case ',':
      calibration_factor -= 10;
      printCalibrationFactor();
      break;
    case 'x':
      calibrationMode = false;
      Serial.println("Calibration disabled");
      break;
    }
    scale.set_scale(calibration_factor);
  } else {
    String command = Serial.readStringUntil('\n');

    if (command == "reset") {
      Serial.println("reset");
      reset = true;
      delay(100);
    } else if (command == "calibrate") {
      calibrationMode = true;
      Serial.println("Calibration mode (x: exit)");
      Serial.println("Use '+' or '-' to calibrate(+/- 1)");
      Serial.println("Use '.' or ',' to calibrate(+/- 10)");
    } else if (command == "battery") {
      printBatteryData();
    }
  }
}

void handleResetSequence() {
  if (scaleValueLast - scaleValue > seqWtDiffMax || 
      rsFlags[0] == 1 && scaleValueLast - scaleValue < seqWtDiffMin || 
      rsFlags[0] == 1 && rsFlags[1] == 1 && scaleValueLast - scaleValue > seqWtDiffMax || 
      rsFlags[0] == 1 && rsFlags[1] == 1 && rsFlags[2] == 1 && scaleValueLast - scaleValue < seqWtDiffMin || 
      rsFlags[0] == 1 && rsFlags[1] == 1 && rsFlags[2] == 1 && rsFlags[3] == 1 && scaleValueLast - scaleValue > seqWtDiffMax || 
      rsFlags[0] == 1 && rsFlags[1] == 1 && rsFlags[2] == 1 && rsFlags[3] == 1 && rsFlags[4] == 1 && scaleValueLast - scaleValue < seqWtDiffMin
      ) {
    if (rsFlags[0] == 0) {
      rsFlags[0] = 1;
      timeRSTrigger = millis();
      Serial.println("[seq] start");
    }
    if (rsFlags[0] == 1 && scaleValueLast - scaleValue < seqWtDiffMin && rsFlags[1] == 0) {
      rsFlags[1] = 1;
      timeRSTrigger = millis();
    }
    if (rsFlags[0] == 1 && rsFlags[1] == 1 && scaleValueLast - scaleValue > seqWtDiffMax && rsFlags[2] == 0) {
      rsFlags[2] = 1;
      timeRSTrigger = millis();
    }
    if (rsFlags[0] == 1 && rsFlags[1] == 1 && rsFlags[2] == 1 && scaleValueLast - scaleValue < seqWtDiffMin && rsFlags[3] == 0) {
      rsFlags[3] = 1;
      timeRSTrigger = millis();
    }
    if (rsFlags[0] == 1 && rsFlags[1] == 1 && rsFlags[2] == 1 && rsFlags[3] == 1 && scaleValueLast - scaleValue > seqWtDiffMax && rsFlags[4] == 0) {
      rsFlags[4] = 1;
      timeRSTrigger = millis();
    }
    if (rsFlags[0] == 1 && rsFlags[1] == 1 && rsFlags[2] == 1 && rsFlags[3] == 1 && rsFlags[4] == 1 && scaleValueLast - scaleValue < seqWtDiffMin && rsFlags[5] == 0) {
      rsFlags[5] = 1;
      timeRSTrigger = millis();
    }
  }

  if (millis() - timeRSTrigger < intervalReset) {
    if (rsFlags[0] == 1 && rsFlags[1] == 1 && rsFlags[2] == 1 && rsFlags[3] == 1 && rsFlags[4] == 1 && rsFlags[5] == 1) {
      reset = true;
      Serial.println("[seq] reset");
      updateDisplay(0, 0, 0, 0, 0, 0);
      delay(500);
    }
  }

  if (timeRSTrigger > 0 && millis() - timeRSTrigger > intervalReset) {
    timeRSTrigger = 0;
    rsFlags[0] = 0;
    rsFlags[1] = 0;
    rsFlags[2] = 0;
    rsFlags[3] = 0;
    rsFlags[4] = 0;
    rsFlags[5] = 0;
    Serial.println("[seq] timeout");
  }
}

void updateWeight() {
  double integral;
  double fractional = modf(scaleValue, &integral);
  int weightOunces = abs(floor(fractional * 16));
  int weight = floor(scaleValue);
  int d1 = 0, d2 = 0, d3 = 0, d4 = 0, d5 = 15, d6 = 0;

  if (weightOunces > 0) {
    d5 = (weightOunces / 10) % 10;
    d6 = weightOunces % 10;
  
    if (weightOunces < 9 || d5 == 0) {
      d5 = 15;
    }
  } else {
    d5 = 15;
    d6 = 0;
  }
  
  if (weight > 0) {
    d1 = (weight / 1000) % 10;
    d2 = (weight / 100) % 10;
    d3 = (weight / 10) % 10;
    d4 = weight % 10;
  }

  if (weight <= 999 && d1 == 0) { d1 = 15; }
  if (weight <= 99 && d2 == 0) { d2 = 15; }
  if (weight <= 9 && d3 == 0) { d3 = 15; }

  if (weight < 0) {
    d1 = 15;
    d2 = 15;
    d3 = 15;
    d4 = 0;
    d5 = 15;
    d6 = 0;
  }

  if (scaleValue < -0.2) {
    d1 = 15;
    d2 = 15;
    d3 = 15;
    d4 = 15;
    d5 = 15;
    d6 = 14;
  }
  
  updateDisplay(d1, d2, d3, d4, d5, d6);
}

void printCalibrationFactor() {
  Serial.print("Calibration set: ");
  Serial.println(calibration_factor);
}

void updateDisplay(int d1, int d2, int d3, int d4, int d5, int d6) {
  sendDisplayRF(d1, d2, d3, d4, d5, d6, 0);
}
boolean sendDisplayRF(int d1, int d2, int d3, int d4, int d5, int d6, int attempts) {
  float values[] = {d1, d2, d3, d4, d5, d6, scaleValue};

  if (!(values[0] == prevVals[0] && 
        values[1] == prevVals[1] && 
        values[2] == prevVals[2] &&
        values[3] == prevVals[3] && 
        values[4] == prevVals[4] && 
        values[5] == prevVals[5])) {
    if (!radio.write(&values, sizeof(values))) {
      /** Failed, try again **/
      if (attempts < 8) {
        sendDisplayRF(d1, d2, d3, d4, d5, d6, attempts + 1);
      } else {
        return false;
      }
      
      failedTransmissions++;
    } else {
      return true;
    }
  }
  
  prevVals[0] = values[0];
  prevVals[1] = values[1];
  prevVals[2] = values[2];
  prevVals[3] = values[3];
  prevVals[4] = values[4];
  prevVals[5] = values[5];
}

void respondPing() {
  const char text[] = "ACK";
  radio.write(&text, sizeof(text));
}
