#include <Arduino.h>
#include <Wire.h>

#define RM3100Address 0x23 // 0x20 (00)

//pin definitions
#define PIN_DRDY 9 //Set pin D9 to be the Data Ready Pin

//internal register values without the R/W bit
#define RM3100_REVID_REG 0x36 // Hexadecimal address for the Revid internal register
#define RM3100_POLL_REG 0x00 // Hexadecimal address for the Poll internal register
#define RM3100_CMM_REG 0x01 // Hexadecimal address for the CMM internal register
#define RM3100_STATUS_REG 0x34 // Hexadecimal address for the Status internal register
#define RM3100_CCX1_REG 0x04 // Hexadecimal address for Cycle Count X1 internal register
#define RM3100_CCX0_REG 0x05 // Hexadecimal address for the Cycle Count X0 internal register

//options
#define initialCC 400 // Set the cycle count
#define singleMode 0 //0 = use continuous measurement mode; 1 = use single measurement mode
#define useDRDYPin 0 //0 = not using DRDYPin ; 1 = using DRDYPin to wait for data

uint8_t revid;
uint16_t cycleCount;
float sens_gain[4];
static const uint8_t SENSOR_ADDRS[4] = {0x20, 0x21, 0x22, 0x23};
// Arrays to hold the raw or converted X/Y/Z for each sensor
long sensorX[4], sensorY[4], sensorZ[4];


void setup() {
  pinMode(PIN_DRDY, INPUT);
  Wire.begin(); // Initiate the Wire library
  Wire.setClock(400000);  //400000
  // Serial.begin(9600); //set baud rate to 9600
  Serial.begin(115200);
  // Serial.begin(230400);
  delay(100);

  // Init sensors
  sens_gain[0] = setupSens(0x20);
  sens_gain[1] = setupSens(0x21);
  sens_gain[2] = setupSens(0x22);
  sens_gain[3] = setupSens(0x23);
}

void loop() {
  // Read all sensors in one consolidated function call
  readAllSensors();

  // Print them out (or do it inside readAllSensors)
  for (int i = 0; i < 4; i++) {
    // Convert raw data to final scaled values
    float xVal = sensorX[i] / sens_gain[i];
    float yVal = sensorY[i] / sens_gain[i];
    float zVal = sensorZ[i] / sens_gain[i];

    // Print X/Y/Z for this sensor
    Serial.print(xVal);
    Serial.print(",");
    Serial.print(yVal);
    Serial.print(",");
    Serial.print(zVal);
    Serial.print(",");
  }
  Serial.println();
}

void readAllSensors() {
  for (int i = 0; i < 4; i++) {
    uint8_t address = SENSOR_ADDRS[i];
    long x = 0, y = 0, z = 0;
    uint8_t x2, x1, x0, y2, y1, y0, z2, z1, z0;

    // If using DRDY pin, you can check it once per sensor
    // if (useDRDYPin) {
    //   while (digitalRead(PIN_DRDY) == LOW);
    // } else {
    //   // Or read the status register (blocking) here
    //   while ((readReg(RM3100_STATUS_REG, address) & 0x80) != 0x80);
    // }

    // Set pointer to first measurement results register
    Wire.beginTransmission(address);
    Wire.write(0x24);
    Wire.endTransmission();

    // Request 9 bytes in one transaction
    Wire.requestFrom((int)address, 9);
    if (Wire.available() == 9) {
      x2 = Wire.read();
      x1 = Wire.read();
      x0 = Wire.read();

      y2 = Wire.read();
      y1 = Wire.read();
      y0 = Wire.read();

      z2 = Wire.read();
      z1 = Wire.read();
      z0 = Wire.read();

      // Construct signed 24-bit values
      if (x2 & 0x80) x = 0xFF;
      if (y2 & 0x80) y = 0xFF;
      if (z2 & 0x80) z = 0xFF;

      x = (x << 24) | ((int32_t)x2 << 16) | ((uint16_t)x1 << 8) | x0;
      y = (y << 24) | ((int32_t)y2 << 16) | ((uint16_t)y1 << 8) | y0;
      z = (z << 24) | ((int32_t)z2 << 16) | ((uint16_t)z1 << 8) | z0;
    }

    // Store results into global arrays for later processing
    sensorX[i] = x;
    sensorY[i] = y;
    sensorZ[i] = z;
  }
}

// void loop() {
//   readSens(0x20,sens_gain[0]);  
//   readSens(0x21, sens_gain[1]);  
//   readSens(0x22, sens_gain[2]);  
//   readSens(0x23, sens_gain[3]);  
//   Serial.println("");
// }

void readSens(int address, float gain)
{  
  long x = 0;
  long y = 0;
  long z = 0;
  uint8_t x2,x1,x0,y2,y1,y0,z2,z1,z0;
 
  // wait until data is ready using 1 of two methods (chosen in options at top of code)
  // if(useDRDYPin){
  //   while(digitalRead(PIN_DRDY) == LOW); //check RDRY pin
  // }
  // else{
  //   while((readReg(RM3100_STATUS_REG, address) & 0x80) != 0x80); //read internal status register
  // }

  Wire.beginTransmission(address);
  Wire.write(0x24); //request from the first measurement results register
  Wire.endTransmission();

  // Request 9 bytes from the measurement results registers
  Wire.requestFrom(address, 9);
  if(Wire.available() == 9) {
    x2 = Wire.read();
    x1 = Wire.read();
    x0 = Wire.read();
   
    y2 = Wire.read();
    y1 = Wire.read();
    y0 = Wire.read();
   
    z2 = Wire.read();
    z1 = Wire.read();
    z0 = Wire.read();
  }

  //special bit manipulation since there is not a 24 bit signed int data type
  if (x2 & 0x80){
      x = 0xFF;
  }
  if (y2 & 0x80){
      y = 0xFF;
  }
  if (z2 & 0x80){
      z = 0xFF;
  }

  //format results into single 32 bit signed value
  x = (x * 256 * 256 * 256) | (int32_t)(x2) * 256 * 256 | (uint16_t)(x1) * 256 | x0;
  y = (y * 256 * 256 * 256) | (int32_t)(y2) * 256 * 256 | (uint16_t)(y1) * 256 | y0;
  z = (z * 256 * 256 * 256) | (int32_t)(z2) * 256 * 256 | (uint16_t)(z1) * 256 | z0;

  // Sensor 1
  Serial.print(x/gain);
  Serial.print(",");
  Serial.print(y/gain);
  Serial.print(",");
  Serial.print(z/gain);
  Serial.print(",");
}

float setupSens(int address){
  revid = readReg(RM3100_REVID_REG, address);
 
    changeCycleCount(initialCC, address); //change the cycle count; default = 200 (lower cycle count = higher data rates but lower resolution)
 
    cycleCount = readReg(RM3100_CCX1_REG, address);
    cycleCount = (cycleCount << 8) | readReg(RM3100_CCX0_REG, address);
 
    double gain = (0.3671 * (float)cycleCount) + 1.5; //linear equation to calculate the gain from cycle count
 
    if (singleMode){
      //set up single measurement mode
      writeReg(RM3100_CMM_REG, 0, address);
      writeReg(RM3100_POLL_REG, 0x70, address);
    }
    else{
      // Enable transmission to take continuous measurement with Alarm functions off
      writeReg(RM3100_CMM_REG, 0x79, address);
    }
    return gain;
}

//addr is the 7 bit value of the register's address (without the R/W bit)
uint8_t readReg(uint8_t addr, int address){
  uint8_t data = 0;
 
  // Enable transmission to specific which register to read from
  Wire.beginTransmission(address);
  Wire.write(addr); //request from the REVID register
  Wire.endTransmission();

  //delay(100);

  // Request 1 byte from the register specified earlier
  Wire.requestFrom(address, 1);
  if(Wire.available() == 1) {
    data = Wire.read();
  }
  return data;
}

//addr is the 7 bit (No r/w bit) value of the internal register's address, data is 8 bit data being written
void writeReg(uint8_t addr, uint8_t data, int address){
  Wire.beginTransmission(address);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}

//newCC is the new cycle count value (16 bits) to change the data acquisition
void changeCycleCount(uint16_t newCC, int address){
  uint8_t CCMSB = (newCC & 0xFF00) >> 8; //get the most significant byte
  uint8_t CCLSB = newCC & 0xFF; //get the least significant byte
 
  Wire.beginTransmission(address);
  Wire.write(RM3100_CCX1_REG);
  Wire.write(CCMSB);  //write new cycle count to ccx1
  Wire.write(CCLSB);  //write new cycle count to ccx0
  Wire.write(CCMSB);  //write new cycle count to ccy1
  Wire.write(CCLSB);  //write new cycle count to ccy0
  Wire.write(CCMSB);  //write new cycle count to ccz1
  Wire.write(CCLSB);  //write new cycle count to ccz0    
  Wire.endTransmission();  
}