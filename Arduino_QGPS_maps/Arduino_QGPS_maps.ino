/**********************************************************************************************************************************
 * QuadrinoGPS Simple Interface Sample for Arduino
 *
 * MIT License:
 *
 * Copyright (c) 2016 FlyingEinstein, LLC and Colin MacKenzie
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation 
 * files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, 
 * modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE 
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF  OR 
 * IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ***********************************************************************************************************************************/
 

#include <Wire.h>
#include "gps_registers.h"

#define QGPS_I2C_ADDRESS        0x20                      //7 bit address 0x40 write, 0x41 read

uint8_t version=0x1;
STATUS_REGISTER status;
GPS_COORDINATES location;
GPS_DATETIME utc;

int points=0; // reset every 60 points
unsigned long nextUpdate=0;

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

bool gps_read_registers(uint8_t addr, void* variable, int length)
{
  byte* loc = (byte*)variable;
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

inline bool gps_read_location(GPS_COORDINATES& location)
{
  return gps_read_registers(I2C_GPS_LOCATION, &location, sizeof(GPS_COORDINATES));
}

inline uint16_t gps_read_altitude()
{
  uint16_t alt;
  return (gps_read_registers(I2C_GPS_ALTITUDE, &alt, sizeof(alt))) ? alt : 0;
}

inline bool gps_read_utc_datetime(GPS_DATETIME& dt)
{
  return gps_read_registers(I2C_GPS_WEEK, &dt, sizeof(GPS_DATETIME  ));
}

inline uint32_t gps_read_utc_tow()
{
  uint32_t utc;
  return (gps_read_registers(I2C_GPS_TIME, &utc, sizeof(utc))) ? utc : 0;
}

inline uint16_t gps_read_utc_week()
{
  uint16_t utc;
  return (gps_read_registers(I2C_GPS_WEEK, &utc, sizeof(utc))) ? utc : 0;
}


void print_header()
{
  Serial.print("type,");
  if(version>=22)
    Serial.print("week,tow,");
  Serial.println("fix,satellites,latitude,longitude,alt");
}


void setup()
{
  location.lat=location.lon=0;
  utc.date = utc.time = 0;
  
  Wire.begin();
  Serial.begin(9600);

  Wire.beginTransmission(QGPS_I2C_ADDRESS);
  Wire.write(I2C_GPS_COMMAND);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();
  
  
  // read the version of the GPS, it should be 21
  do {
    delay(100);
    version = gps_read_u8(I2C_GPS_REG_VERSION);
  } while(version==0 || version==0xff);
}

void loop()
{
  uint8_t u8;
  if(nextUpdate < millis()) {
    nextUpdate = nextUpdate + 1000;
    
    if(points >=60) {
      print_header();
      points=0;
    }
    
    // read status (fix and number of satellites)
    status = gps_read_status();

    if(version >=22)
      gps_read_utc_datetime(utc);
    
    uint16_t alt = gps_read_altitude();

    // read lat/lon location
    gps_read_location(location);
    
    // convert the gps coordinates from fixed-decimal integer to double floating point
    // this is not efficient for our little 8-bit Atmel processor (but since we have so little to do...)
    double lat = (double)location.lat/10000000.0, lon = (double)location.lon/10000000.0;

    // GPS Visualizer likes the rows to start with T to indicate "track" type
    Serial.print("T,");

    if(version>=22) {
      //UTC Time
      Serial.print(utc.date);
      Serial.print(",");
      Serial.print(utc.time);
      Serial.print(",");
    }
    
    // print our coordinate fix state
    // 2D gets lat/lon coordinates, 3D means we have altitude as well
    if(status.gps3dfix)
      Serial.print("3D");
    else  if(status.gps2dfix)
      Serial.print("2D");
    else
      Serial.print("NO-FIX");
      
    // Number of satellites
    Serial.print(",");
    Serial.print(status.numsats);
    
    // Latitude, Longitude, Altitude
    Serial.print(",");
    Serial.print(lat,7);
    Serial.print(",");
    Serial.print(lon,7);
    Serial.print(",");
    Serial.print(alt);
    Serial.println();

    points++;
  } 
}


