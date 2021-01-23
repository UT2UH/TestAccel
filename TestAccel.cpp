/*
 * TestAccel.cpp
 *
 *  Created on: 14.12.2020
 *      Author: frank
 *
 *  and code from
 *  2008 The Android Open Source Project
 *  updated by K. Townsend (Adafruit Industries)
 *
 *  and code from
 *  SparkFun_ADXL345.cpp
 *  E.Robert @ SparkFun Electronics
 *
 */

#include <Arduino.h>
#include <Wire.h>
#include <esp32/ulp.h>
#include <driver/rtc_io.h>
#include <soc/rtc_i2c_reg.h>
#include <soc/sens_reg.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "TestAccel.h"
#include "TestAccel_ULP.h"

#define ACC_SDA GPIO_NUM_15
#define ACC_SCL GPIO_NUM_4
#define ACC_FREQ 400000L

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

float X_out, Y_out, Z_out;  // Outputs
bool _ADXL345_status = true;
int  _ADXL345_error_code = ADXL345_OK;
int goodcount = 0;
int badcount  = 0;

void setup() {
  Serial.begin(115200); // Initiate serial communication for printing the results on the Serial monitor
  delay(100);
  //check and wake up sensor
  if (!accel.begin((uint8_t)ADXL345_DEFAULT_ADDRESS, ACC_SDA, ACC_SCL, ACC_FREQ)){
    // There was a problem detecting the ADXL345 ... check your connections
    Serial.println("Ooops, no ADXL345 detected ");// + String(_ID, HEX) + " Check your wiring!");
    while(1);
  }
  // Set the range to whatever is appropriate for your project
  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_25_HZ);
  _ADXL345_setFullResBit(false);

  accel.writeRegister(ADXL345_REG_OFSX,56);
  accel.writeRegister(ADXL345_REG_OFSZ,-127);


  //INIT_ACCEL_SOC();
  INIT_ACCEL_ULP(250000);
  delay(100);
}

void loop(){
  uint32_t takt1 = millis();

  //loopSOC();
  loopULP();


  //Serial.println("time needed [ms]: " + String(millis() - takt1));
  delay(200);
}
void loopULP(void){
#if I2C_DEBUG
  /*
  //simple debug
  uint8_t ack1 = (uint8_t)RTC_SLOW_MEM[I2C_DEBUG_MEM + 10];
  uint8_t ack2 = (uint8_t)RTC_SLOW_MEM[I2C_DEBUG_MEM + 20];
  uint8_t ack3 = (uint8_t)RTC_SLOW_MEM[I2C_DEBUG_MEM + 30];
  uint8_t byte = (uint8_t)RTC_SLOW_MEM[I2C_DEBUG_MEM + 39];
  if ((uint8_t)RTC_SLOW_MEM[I2C_DEBUG_MEM] != 0xaa){
    if ((ack1 == 0) && (ack2 == 0) && (ack2 == 0) && (byte != 0xaa)){
      goodcount++;
    } else {
      badcount++;
    }
    Serial.println("Good / bad: " + String(goodcount) + "/" + String(badcount) + "; " + String(ack1) + String(ack2) + String(ack3) + ", 0x" + String(byte, HEX));
  }
  Serial.print("BYTE: "); Serial.println("0x" + String((uint16_t)RTC_SLOW_MEM[1063], HEX));
  */
  //detailed debug
  int j = I2C_DEBUG_MEM+1;
  for(int k=1; k<4; k++){
    Serial.print("WR" + String(k) + " : "); Serial.println("0x" + String((uint16_t)RTC_SLOW_MEM[j++], HEX));
    Serial.print("BIT" + String(k) + ": ");
    Serial.print(String((uint8_t)RTC_SLOW_MEM[j++])); Serial.print(", ");
    Serial.print(String((uint8_t)RTC_SLOW_MEM[j++])); Serial.print(", ");
    Serial.print(String((uint8_t)RTC_SLOW_MEM[j++])); Serial.print(", ");
    Serial.print(String((uint8_t)RTC_SLOW_MEM[j++])); Serial.print(", ");
    Serial.print(String((uint8_t)RTC_SLOW_MEM[j++])); Serial.print(", ");
    Serial.print(String((uint8_t)RTC_SLOW_MEM[j++])); Serial.print(", ");
    Serial.print(String((uint8_t)RTC_SLOW_MEM[j++])); Serial.print(", ");
    Serial.println(String((uint8_t)RTC_SLOW_MEM[j++]) + "; " + String(j));
    Serial.print("ACK" + String(k) + ": ");  Serial.println(String((uint8_t)RTC_SLOW_MEM[j++]));
  }
  for(int k=1; k<3; k++){
    Serial.print("RD " + String(k) + ": ");
    Serial.print(String((uint8_t)RTC_SLOW_MEM[j++])); Serial.print(", ");
    Serial.print(String((uint8_t)RTC_SLOW_MEM[j++])); Serial.print(", ");
    Serial.print(String((uint8_t)RTC_SLOW_MEM[j++])); Serial.print(", ");
    Serial.print(String((uint8_t)RTC_SLOW_MEM[j++])); Serial.print(", ");
    Serial.print(String((uint8_t)RTC_SLOW_MEM[j++])); Serial.print(", ");
    Serial.print(String((uint8_t)RTC_SLOW_MEM[j++])); Serial.print(", ");
    Serial.print(String((uint8_t)RTC_SLOW_MEM[j++])); Serial.print(", ");
    Serial.println(String((uint8_t)RTC_SLOW_MEM[j++]) + "; " + String(j));
  }
  Serial.print("DATA: "); Serial.println((int16_t)RTC_SLOW_MEM[j++]);

  for(int i=I2C_DEBUG_MEM; i<I2C_DEBUG_MEM+64; i++){ RTC_SLOW_MEM[i] = 0xaa; };
#else
  uint8_t res  = (uint8_t)RTC_SLOW_MEM[I2C_TRARANSMISSION_RESULT];
  int16_t data = (int16_t)RTC_SLOW_MEM[I2C_READ_RESULT];
  if (res != 2){
    if ((res == I2C_SUCCESS) && (data != 0x00aa)){
      goodcount++;
    } else {
      badcount++;
    }
    Serial.println("Good / bad: " + String(goodcount) + "/" + String(badcount) + "; " + String(res) + ", " + String(data));
    RTC_SLOW_MEM[I2C_TRARANSMISSION_RESULT] = 2;
    RTC_SLOW_MEM[I2C_READ_RESULT] = 0xaa;
  }
#endif
}

void loopSOC(void){
  Wire.beginTransmission((uint8_t)ADXL345_DEFAULT_ADDRESS);
  Wire.write((uint8_t)ADXL345_REG_DEVID);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)ADXL345_DEFAULT_ADDRESS, (uint8_t)1);
  uint8_t _ID = Wire.read();
  Serial.print("ID: 0x"); Serial.print(_ID, HEX); Serial.print("  ");
  int16_t x1 = 1;
  int16_t x2 = -1;
  int ix = 0;
  while ((x1 != x2) && (ix < 5)){
    ix++;
    Wire.beginTransmission((uint8_t)ADXL345_DEFAULT_ADDRESS);
    Wire.write((uint8_t)ADXL345_REG_DATAX0);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)ADXL345_DEFAULT_ADDRESS, (uint8_t)1);
    uint8_t _x1 = (uint8_t)Wire.read();
    Wire.beginTransmission((uint8_t)ADXL345_DEFAULT_ADDRESS);
    Wire.write((uint8_t)ADXL345_REG_DATAX1);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)ADXL345_DEFAULT_ADDRESS, (uint8_t)1);
    uint8_t _x2 = (uint8_t)Wire.read();
    x1 = (int16_t)(_x1 | (_x2 << 8));
    Wire.beginTransmission((uint8_t)ADXL345_DEFAULT_ADDRESS);
    Wire.write((uint8_t)ADXL345_REG_DATAX0);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)ADXL345_DEFAULT_ADDRESS, (uint8_t)1);
    _x1 = (uint8_t)Wire.read();
    Wire.beginTransmission((uint8_t)ADXL345_DEFAULT_ADDRESS);
    Wire.write((uint8_t)ADXL345_REG_DATAX1);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)ADXL345_DEFAULT_ADDRESS, (uint8_t)1);
    _x2 = (uint8_t)Wire.read();
    x2 = (int16_t)(_x1 | (_x2 << 8));
  }

  Serial.print("XX: "); Serial.print(x1); Serial.print("["); Serial.print(ix); Serial.print("]    ");
  Serial.print("X : "); Serial.print(accel.getX()); Serial.print("  ");
  Serial.print("Y : "); Serial.print(accel.getY()); Serial.print("  ");
  Serial.print("Z : "); Serial.print(accel.getZ()); Serial.print("  ");
  Serial.println("");

}
void INIT_ACCEL_SOC(void){
  /* Initialize the sensor */

  Serial.println("wiring done");//: 0x" + String(_ID, HEX));
  delay(100);

  // Display some basic information on this sensor
  displaySensorDetails();

  // Display additional settings (outside the scope of sensor_t)
  displayDataRate();
  displayRange();
  Serial.println("SDA: " + String(ACC_SDA) + ", SCL: " + String(ACC_SCL));
  Serial.println("------------------------------------");
  Serial.println("");

}



void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
void displayDataRate(void)
{
  Serial.print  ("Data Rate:    ");

  switch(accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 ");
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 ");
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 ");
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 ");
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 ");
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 ");
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 ");
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 ");
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 ");
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 ");
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 ");
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 ");
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 ");
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 ");
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 ");
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 ");
      break;
    default:
      Serial.print  ("???? ");
      break;
  }
  Serial.println(" Hz");
}

void displayRange(void)
{
  Serial.print  ("Range:         +/- ");

  switch(accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 ");
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 ");
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 ");
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 ");
      break;
    default:
      Serial.print  ("?? ");
      break;
  }
  Serial.println(" g");
}

/*************************** BANDWIDTH ******************************/
/*                          ~ SET & GET                             */
byte _ADXL345_get_bw_code(){
  byte bw_code;
  bw_code = accel.readRegister(ADXL345_BW_RATE);
  return bw_code;
}

void _ADXL345_set_bw(byte bw_code){
  if((bw_code < ADXL345_BW_0_05) || (bw_code > ADXL345_BW_1600)){
    _ADXL345_status = false;
    _ADXL345_error_code = ADXL345_BAD_ARG;
  }
  else{
    accel.writeRegister(ADXL345_BW_RATE, bw_code);
  }
}

/************************* FULL_RES BIT STATE ***********************/
/*                           ~ GET & SET                            */
bool _ADXL345_getFullResBit() {
  return _getRegisterBit(ADXL345_DATA_FORMAT, 3);
}

/*  If Set (1) Device is in Full Resolution Mode: Output Resolution Increase with G Range
 *  Set by the Range Bits to Maintain a 4mg/LSB Scale Factor
 *  If Set (0) Device is in 10-bit Mode: Range Bits Determine Maximum G Range
 *  And Scale Factor
 */
void _ADXL345_setFullResBit(bool fullResBit) {
  _setRegisterBit(ADXL345_DATA_FORMAT, 3, fullResBit);
}

/*************************** RATE BITS ******************************/
/*                                                                  */
double _ADXL345_getRate(){
  byte _b;
  _b = accel.readRegister(ADXL345_BW_RATE);
  _b &= B00001111;
  return (pow(2,((int) _b)-6)) * 6.25;
}

void _ADXL345_setRate(double rate){
  byte _b,_s;
  int v = (int) (rate / 6.25);
  int r = 0;
  while (v >>= 1)
  {
    r++;
  }
  if (r <= 9) {
    _b = accel.readRegister(ADXL345_BW_RATE);
    _s = (byte) (r + 6) | (_b & B11110000);
    accel.writeRegister(ADXL345_BW_RATE, _b);
  }
}

void _setRegisterBit(byte regAdress, int bitPos, bool state) {
  byte _b;
  _b = accel.readRegister(regAdress);
  if (state) {
    _b |= (1 << bitPos);  // Forces nth Bit of _b to 1. Other Bits Unchanged.
  }
  else {
    _b &= ~(1 << bitPos); // Forces nth Bit of _b to 0. Other Bits Unchanged.
  }
  accel.writeRegister(regAdress, _b);
}

bool _getRegisterBit(byte regAdress, int bitPos) {
  byte _b;
  _b = accel.readRegister(regAdress);
  return ((_b >> bitPos) & 1);
}
