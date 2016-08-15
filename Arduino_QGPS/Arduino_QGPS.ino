/*******************************************
 Sample sketch that configures an HMC5883L 3 axis
 magnetometer to continuous mode and reads back 
 the three axis of data.
 Code compiles to a size of 1500 bytes
 Equivalent Wire Library code compiles to 2032 bytes
 *******************************************/

//#include <stdint.h>

#include <Wire.h>
#include "gps_registers.h"

#define QGPS_I2C_ADDRESS        0x20                      //7 bit address 0x40 write, 0x41 read

uint8_t version=0x1;
STATUS_REGISTER status;
GPS_COORDINATES location;

uint8_t gps_read_u8(uint8_t addr)
{
  Wire.beginTransmission(QGPS_I2C_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(QGPS_I2C_ADDRESS,1);
  return Wire.read();
}

STATUS_REGISTER gps_read_status()
{
  Wire.beginTransmission(QGPS_I2C_ADDRESS);
  Wire.write((uint8_t)I2C_GPS_STATUS_00);
  Wire.endTransmission();
  Wire.requestFrom(QGPS_I2C_ADDRESS,1);
  byte b = Wire.read();
  return *(STATUS_REGISTER*)&b;
}

bool gps_read_registers(uint8_t addr, byte* loc, int length)
{
  Wire.beginTransmission(QGPS_I2C_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(QGPS_I2C_ADDRESS,length);
  if(length <= Wire.available()) {
    for(int i=0; i<length; i++)
      *loc++ = Wire.read();
    return true;
  } else
    return false;
}

void setup()
{
  location.lat=location.lon=0;
  Wire.begin();
  Serial.begin(9600);

  Wire.beginTransmission(QGPS_I2C_ADDRESS);
  Wire.write(I2C_GPS_COMMAND);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();
}

unsigned long nextUpdate=0;

void loop()
{
  uint8_t u8;
  if(nextUpdate < millis()) {
    nextUpdate = nextUpdate + 1000;
    
    // read the version of the GPS, it should be 21
    //version = gps_read_u8(I2C_GPS_REG_VERSION);
    
    // read status (fix and number of satellites)
    status = gps_read_status();

    // read lat/lon location
    gps_read_registers(I2C_GPS_LOCATION, (byte*)&location, sizeof(GPS_COORDINATES));
    
    // convert the gps coordinates from fixed-decimal integer to double floating point
    // this is not efficient for our little 8-bit Atmel processor (but since we have so little to do...)
    double lat = (double)location.lat/10000000.0, lon = (double)location.lon/10000000.0;
    
    // print our coordinate fix state
    // 2D gets lat/lon coordinates, 3D means we have altitude as well
    if(status.gps3dfix)
      Serial.print("3D");
    else  if(status.gps2dfix)
      Serial.print("2D");
    else
      Serial.print("NO-FIX");
      
    // print the number of satellites
    Serial.print("  S");
    Serial.print(status.numsats);
    
    // print a Google maps link to our location
    Serial.print("   https://www.google.com/maps/@");
    Serial.print(lat,7);
    Serial.print(",");
    Serial.print(lon,7);
    Serial.println(",100m");
  } 
}


