


#if defined(NMEA)

// no initialization (though you may want to send config commands here to your GPS unit
inline void GPSModuleInit() {}


/* The latitude or longitude is coded this way in NMEA frames
  dddmm.mmmm   coded as degrees + minutes + minute decimal
  This function converts this format in a unique unsigned long where 1 degree = 10 000 000
  I increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
  with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased 
  resolution also increased precision of nav calculations
*/

#define DIGIT_TO_VAL(_x)	(_x - '0')
uint32_t GPS_coord_to_degrees(char* s)
{
	char *p, *q;
	uint8_t deg = 0, min = 0;
	unsigned int frac_min = 0;

	// scan for decimal point or end of field
	for (p = s; isdigit(*p); p++)
		;
	q = s;

	// convert degrees
	while ((p - q) > 2) {
		if (deg)
			deg *= 10;
		deg += DIGIT_TO_VAL(*q++);
	}
	// convert minutes
	while (p > q) {
		if (min)
			min *= 10;
		min += DIGIT_TO_VAL(*q++);
	}
	// convert fractional minutes
	// expect up to four digits, result is in
	// ten-thousandths of a minute
	if (*p == '.') {
		q = p + 1;
		for (char i = 0; i < 4; i++) {
			frac_min *= 10;
			if (isdigit(*q))
				frac_min += *q++ - '0';
		}
	}
	return deg * 10000000UL + (min * 1000000UL + frac_min*100UL) / 6;
}

/* This is am expandable implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output NMEA frames.
   It assumes there are some NMEA GGA, GSA and RMC frames to decode on the serial bus
   Using the following data :
   GGA
     - time
     - latitude
     - longitude
     - GPS fix 
     - GPS num sat (5 is enough to be +/- reliable)
     - GPS alt
   GSA
     - 3D fix (it could be left out since we have a 3D fix if we have more than 4 sats  
   RMC
     - GPS speed over ground, it will be handy for wind compensation (future)  
     
*/

#define NO_FRAME    0
#define GPGGA_FRAME 1
#define GPGSA_FRAME 2
#define GPRMC_FRAME 3

bool GPS_NMEA_newFrame(char c) {

  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, gps_frame = NO_FRAME;

  switch (c) {
    case '$': param = 0; offset = 0; parity = 0; 
              break;
    case ',':
    case '*':  string[offset] = 0;
                if (param == 0) { //frame identification
                  gps_frame = NO_FRAME;  
                  if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') gps_frame = GPGGA_FRAME;
                  if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'S' && string[4] == 'A') gps_frame = GPGSA_FRAME;
                  if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') gps_frame = GPRMC_FRAME;
                }
                
                switch (gps_frame)
                {
                  //************* GPGGA FRAME parsing
                  case GPGGA_FRAME: 
                    switch (param)
                     {
                      case 1: i2c_dataset.time = (atof(string)*1000);      //up to .000 s precision not needed really but the data is there anyway
                              break;
                      //case 2: i2c_dataset.gps_loc.lat = GPS_coord_to_degrees(string);
                      case 2: GPS_read[LAT] = GPS_coord_to_degrees(string);
                              break;
                      //case 3: if (string[0] == 'S') i2c_dataset.gps_loc.lat = -i2c_dataset.gps_loc.lat;
                      case 3: if (string[0] == 'S') GPS_read[LAT] = -GPS_read[LAT];
                              break;
                      //case 4: i2c_dataset.gps_loc.lon = GPS_coord_to_degrees(string);
                      case 4: GPS_read[LON] = GPS_coord_to_degrees(string);
                              break;
                      //case 5: if (string[0] == 'W') i2c_dataset.gps_loc.lon = -i2c_dataset.gps_loc.lon;
                      case 5: if (string[0] == 'W') GPS_read[LON] = -GPS_read[LON];
                              break;
                      case 6: i2c_dataset.status.gps2dfix = string[0]  > '0';
                              break;
                      case 7: i2c_dataset.status.numsats = atoi(string);
                              break;
                      case 9: i2c_dataset.altitude = atoi(string);
                              break;
                     }
                   break;         
                   //************* GPGSA FRAME parsing
                   case GPGSA_FRAME:
                     switch (param)
                     {
                      case 2: i2c_dataset.status.gps3dfix = string[0] == '3';
                      break;
                     }
                   break;
                   //************* GPGSA FRAME parsing
                   case GPRMC_FRAME:
                     switch(param)
                     {
                       case 7: i2c_dataset.ground_speed = (atof(string)*0.5144444)*10;      //convert to m/s*100
                               break; 
	               case 8: i2c_dataset.ground_course = (atof(string)*10);				//Convert to degrees *10 (.1 precision)
							   break;
                     }
                   
                   break;                   
                }
            
                param++; offset = 0;
                if (c == '*') checksum_param=1;
                else parity ^= c;
                break;
     case '\r':
     case '\n':  
                if (checksum_param) { //parity checksum
                  uint8_t checksum = 16 * ((string[0]>='A') ? string[0] - 'A'+10: string[0] - '0') + ((string[1]>='A') ? string[1] - 'A'+10: string[1]-'0');
                  if (checksum == parity) frameOK = 1;
                }
                checksum_param=0;
                break;
     default:
             if (offset < 15) string[offset++] = c;
             if (!checksum_param) parity ^= c;
             
  }
  return frameOK && (gps_frame == GPGGA_FRAME);
}
#endif 

