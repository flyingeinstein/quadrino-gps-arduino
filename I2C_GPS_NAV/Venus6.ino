//#include "Venus6.h"

#ifdef VENUS6

venus_message venus_ctx;

void VenusWriteImmediate()
{
  int pls=0;
  byte cs=venus_ctx.id;
  while(pls<venus_ctx.length)
    cs = cs ^ venus_ctx.body[pls++];
  pls++;
  VenusSerial.write(0xA0);
  VenusSerial.write(0xA1);
  VenusSerial.write((pls>>8)&0xff);
  VenusSerial.write(pls&0xff);
  VenusSerial.write(venus_ctx.id);
  for(pls=0; pls<venus_ctx.length; pls++)
    VenusSerial.write(venus_ctx.body[pls]);
  //Serial.write(body, length);
  VenusSerial.write(cs);
  VenusSerial.write(0x0D);
  VenusSerial.write(0x0A);
}

#if 0
void VenusWriteNull()
{
  venus_ctx.id=VENUS_NACK;
  venus_ctx.length=1;
  venus_ctx.body[0]=0;
  VenusWriteImmediate(); 
}
#endif

//inline void VenusWriteImmediate(byte msgid, byte a) { venus_ctx.length=1; venus_ctx.id=msgid; venus_ctx.body[0]=a; VenusWriteImmediate(); }
//inline void VenusWriteImmediate(byte msgid, byte a, byte b) { venus_ctx.length=1; venus_ctx.id=msgid; venus_ctx.body[0]=a; venus_ctx.body[1]=b; VenusWriteImmediate(); }



#if 0
#define SWAP16(x) ((x&0xff)<<8 | (x>>8))
#define SWAP32(x) ((x&0xff)<<24 | ((x&0xff00)<<8) | ((x&0xff0000)>>8) | ((x&0xff000000)>>24))
#else
uint16_t SWAP16(uint16_t x) { return ((x&0xff)<<8 | (x>>8)); }
uint32_t SWAP32(uint32_t x) { return ((x&0xff)<<24 | ((x&0xff00)<<8) | ((x&0xff0000)>>8) | ((x&0xff000000)>>24)); }
#endif

void VenusFixLocationEndianess()
{
  venus_ctx.location.gps_week=SWAP16(venus_ctx.location.gps_week);
  venus_ctx.location.gps_tow = SWAP32(venus_ctx.location.gps_tow);
  venus_ctx.location.latitude = SWAP32(venus_ctx.location.latitude);
  venus_ctx.location.longitude = SWAP32(venus_ctx.location.longitude);
  venus_ctx.location.ellipsoid_alt = SWAP32(venus_ctx.location.ellipsoid_alt);
  venus_ctx.location.sealevel_alt = SWAP32(venus_ctx.location.sealevel_alt);
  venus_ctx.location.gdop = SWAP16(venus_ctx.location.gdop);
  venus_ctx.location.pdop = SWAP16(venus_ctx.location.pdop);
  venus_ctx.location.hdop = SWAP16(venus_ctx.location.hdop);
  venus_ctx.location.vdop = SWAP16(venus_ctx.location.vdop);
  venus_ctx.location.tdop = SWAP16(venus_ctx.location.tdop);
  venus_ctx.location.ecef.x = SWAP32(venus_ctx.location.ecef.x);
  venus_ctx.location.ecef.y = SWAP32(venus_ctx.location.ecef.y);
  venus_ctx.location.ecef.z = SWAP32(venus_ctx.location.ecef.z);
  venus_ctx.location.vel.x = SWAP32(venus_ctx.location.vel.x);
  venus_ctx.location.vel.y = SWAP32(venus_ctx.location.vel.y);
  venus_ctx.location.vel.z = SWAP32(venus_ctx.location.vel.z);
}

char VenusProcessInput(int c)
{
  static byte state=0;
  static char n=0;
  static int cr=0;
  
  switch(state) {
    case 0: if(c==0xA0) state++; break;
    case 1: if(c==0xA1) state++; else state=0; break;
    case 2: venus_ctx.length=c<<8; state++; break;  // read payload length (2 bytes)
    case 3: venus_ctx.length|=c; state++; break;
    case 4: 
      venus_ctx.id=cr=c; n=0; 
      if(--venus_ctx.length>0) state++; else state=6; // if no payload then skip next state
      break;
    case 5: // read bytes of the payload
      if(n<VENUS_MAX_PAYLOAD)
        venus_ctx.body[n]=c;
      n++;
      cr ^= c;  // adjust checksum
      if(n>=venus_ctx.length) state++;
      break;
    case 6: 
      if(c==cr) 
        state++; 
      else {
        state=0; 
        return VENUS_BADCR; 
      } break; // check checksum, abort if not-equal
    case 7: if(c==0x0d) state++; break;
    case 8: 
      state=0;
      if(venus_ctx.id==VENUS_GPS_LOCATION)
        VenusFixLocationEndianess();
      return (c==0x0A) ? ((n<=VENUS_MAX_PAYLOAD)?VENUS_OK:VENUS_CLIPPED): VENUS_INCOMPLETE; 
    default: 
      state=0; 
      return VENUS_INCOMPLETE;
  }
  return VENUS_MORE;
}

// reads the next binary message
char VenusRead(int timeout)
{
  char result;
  // read immediate or return timeout
#if 0
  if(timeout==1)
    if (VenusSerial.available() && (result=VenusProcessInput(VenusSerial.read()))==VENUS_OK || result==VENUS_CLIPPED)
      return result;
    else
      return VENUS_TIMEOUT;
#endif
  // read with timeout or infinite
  unsigned long long stopat = millis() + timeout;
  while(timeout==0 || millis()<stopat) {
    if (VenusSerial.available() && (result=VenusProcessInput(VenusSerial.read()))==VENUS_OK || result==VENUS_CLIPPED)
        // a complete message was received
        return result;
    bargraph_update();
  }
  return VENUS_TIMEOUT;
}

#if 0
char VenusAsyncRead()
{
  char result;
  if (VenusSerial.available() && (result=VenusProcessInput(VenusSerial.read()))==VENUS_OK || result==VENUS_CLIPPED)
        // a complete message was received
        return result;
  return VENUS_MORE;
}
#endif

char VenusWrite(int timeout)
{
  byte msgid = venus_ctx.id;
  unsigned long long stopat = millis() + timeout;
  VenusWriteImmediate();      // write msg
  while(millis() < stopat) {    // now wait for ack
    VenusRead(stopat - millis());
    if(venus_ctx.id==VENUS_ACK && venus_ctx.body[0]==msgid)
      return VENUS_OK;
    else if(venus_ctx.id==VENUS_NACK && venus_ctx.body[0]==msgid)
      return VENUS_NACKED;
    //else
    //  VenusDispatchMessage();  // dispatch this message through the normal channels
    bargraph_update();
  }
  return VENUS_TIMEOUT;
}

/* conflicts with VenusWrite(timeout)
bool VenusWrite(byte queryid)
{
    venus_ctx.id = queryid;
    venus_ctx.length = 0;
    return VenusWrite(1000);
}*/

// return the response msgid for a given query msgid
byte VenusWhatIsResponseMsgIdOf(byte msgid)
{
  switch(msgid) {
         case VENUS_QUERY_SW_VERSION: return VENUS_REPORT_SW_VERSION; 
    case VENUS_QUERY_GPS_UPDATE_RATE: return VENUS_REPORT_GPS_UPDATE_RATE; 
            case VENUS_GET_EPHEMERIS: return VENUS_REPORT_EPHEMERIS; 
              case VENUS_QUERY_DATUM: return VENUS_GPS_DATUM; 
               case VENUS_QUERY_WAAS: return VENUS_GPS_WAAS_STATUS; 
        case VENUS_QUERY_GPS_PINNING: return VENUS_GPS_PINNING_STATUS; 
           case VENUS_QUERY_NAV_MODE: return VENUS_GPS_NAV_MODE; 
          case VENUS_QUERY_1PPS_MODE: return VENUS_GPS_1PPS_MODE; 
          default: return 0;
  }
}

// send a query to the GPS module and expect/waitfor a reply
char VenusQuery(int timeout)
{
  int orig_timeout = timeout;
  byte msgid=venus_ctx.id, reply_msgid=VenusWhatIsResponseMsgIdOf(venus_ctx.id);
  unsigned long long stopat = millis() + timeout;
  bool acked=false;
  char result;
  VenusWriteImmediate();      // write msg
  while(millis() < stopat) {    // now wait for ack
    if((result=VenusRead(stopat - millis())) == VENUS_TIMEOUT)
      goto timedout;
    else if(venus_ctx.id==VENUS_ACK && venus_ctx.body[0]==msgid) {
      acked=true;  // got the ack, now the response
      timeout = orig_timeout;
    } else if(venus_ctx.id==VENUS_NACK && venus_ctx.body[0]==msgid) {
      return VENUS_NACKED;
    } else if(venus_ctx.id==reply_msgid) {
      return result;
    } //else
      ///VenusDispatchMessage();  // dispatch this message through the normal channels
    bargraph_update();
  }
timedout:
  return acked ? VENUS_ACK_NOREPLY : VENUS_TIMEOUT;
}

char VenusSetOption(byte option, byte value, bool flash)
{
    venus_ctx.id = option;
    venus_ctx.length = 2;
    venus_ctx.body[0] = value;
    venus_ctx.body[1] = flash?VENUS_FLASH:VENUS_SRAM;
    return VenusWrite(VENUS_DEFAULT_TIMEOUT);
}

inline char VenusSetUpdateRate(byte updaterate, bool flash)
{ return VenusSetOption(VENUS_CONFIG_GPS_UPDATE_RATE, updaterate, flash); }

inline char VenusSetOutput(byte outputmode, bool flash)
{ return VenusSetOption(VENUS_CONFIG_OUTPUT_MSG_FORMAT, outputmode, flash); }

inline char VenusSetWAAS(char enable, bool flash)
{ return VenusSetOption(VENUS_CONFIG_WAAS, enable, flash); }

inline char VenusSetPowerMode(char enable, bool flash)
{ return VenusSetOption(VENUS_CONFIG_POWER_MODE, enable, flash); }

inline char VenusSetNavMode(char car_or_pedestrial, bool flash)
{ return VenusSetOption(VENUS_CONFIG_NAV_MODE, car_or_pedestrial, flash); }

inline char VenusSetPositionPinning(char enable, bool flash)
{ return VenusSetOption(VENUS_CONFIG_GPS_PINNING, enable, flash); }


#if 0
// this could be computed to save memory!
const unsigned long VenusSupportedRates[] = {4800,9600,19200,38400,57600,115200}; //,230400};
//const uint16_t VenusSupportedRates[] = {48,96,1152}; //,230400};
const byte VenusSupportedRatesCount = (sizeof(VenusSupportedRates)/sizeof(unsigned long));

unsigned long VenusGetBaudRate(byte n)
{
  return (n<VenusSupportedRatesCount) ? VenusSupportedRates[n] : 4800;
}

byte VenusGetBaudRateOrdinal(unsigned long baud)
{
  for(byte i=0; i<VenusSupportedRatesCount; i++)
    if(VenusSupportedRates[i] == baud)
      return i;
}
#else
const byte VenusSupportedRatesCount = 3;
uint32_t VenusGetBaudRate(byte n)
{
  switch(n) {
    case 0: return 4800;
    case 1: return 9600;
    default: return 115200;
  }
}

byte VenusGetBaudRateOrdinal(uint32_t baud)
{
  if(baud==4800)
    return 0;
  else if(baud==9600)
    return 1;
  else
    return 5;
}
#endif

char VenusSetBaudRate(uint32_t baudrate, bool flash)
{
  char result;
  venus_ctx.id = VENUS_CONFIG_SERIAL_PORT;
  venus_ctx.length = 3;
  venus_ctx.body[0] = 0;  // Venus device's COM1
  venus_ctx.body[1] = VenusGetBaudRateOrdinal(baudrate);
  venus_ctx.body[2] = flash?VENUS_FLASH:VENUS_SRAM;
  if((result=VenusWrite(VENUS_DEFAULT_TIMEOUT)) == VENUS_OK) {
    VenusSerial.begin(baudrate);
    return result;
  } else {
    return result;
  }
}

void VenusScan(unsigned long desiredBaud, byte desiredUpdateRate, byte desiredMessageFormat)
{
  // detect the baud rate
  char result;
  uint32_t baud;
  char actualUpdateRate=0;
  char i = -1;
  while(1) {
    if(i<0) i=VenusSupportedRatesCount-1;
    baud = VenusGetBaudRate(i);
    bargraph_upto(i+1);
    
    digitalWrite(2, LOW);
    delay(250);
    digitalWrite(2, HIGH);
    delay(1000);
    
    // try to communicate at this baud rate
    VenusSerial.begin(baud);
    //VenusWriteNull();
    venus_ctx.id=VENUS_QUERY_GPS_UPDATE_RATE;
    venus_ctx.length=0;
    if((result=VenusQuery(VENUS_DEFAULT_TIMEOUT)) == VENUS_OK) {
      actualUpdateRate = venus_ctx.body[0];
      bargraph_upto(3);  // enable all red lights
      
      if(baud != desiredBaud) {
        if(VenusSetBaudRate(desiredBaud, true)!=VENUS_OK)
          bargraph_displayerror(STARTUP_BAUDRATE);
        else
          delay(4000);
      }
      bargraph_setled(STARTUP_BAUDRATE);

      if(VenusSetOutput(desiredMessageFormat, false)!=VENUS_OK)
        bargraph_displayerror(STARTUP_MSGMODE);
      else
        bargraph_setled(STARTUP_MSGMODE);

      if(actualUpdateRate!=desiredUpdateRate && VenusSetUpdateRate(desiredUpdateRate, true)!=VENUS_OK)
        bargraph_displayerror(STARTUP_UPDATERATE);
      else
        bargraph_setled(STARTUP_UPDATERATE);

      return;
    }
    i--;
  }
}

void GPSModuleInit()
{
  // enable the 3V3B regulator thus enabling the GPS
  bargraph_upto(0);
  digitalWrite(2, LOW);   // set GPS device OFF
  digitalWrite(A1, HIGH);   // enable pullup on A1
  pinMode(2, OUTPUT);     
  
  if(!digitalRead(A1)) {
    // go into HiZ state
    pinMode(1, INPUT);
    digitalWrite(0,LOW);  // disable pullups
    digitalWrite(1,LOW);
    digitalWrite(2, HIGH);   // set GPS on
    while(!digitalRead(A1)) {
      digitalWrite(3, HIGH);   // set the LED on
      delay(300);              // wait for a second
      digitalWrite(3, LOW);    // set the LED off
      delay(700);              // wait for a second
    }
    pinMode(1, OUTPUT);     
  }
  
  
  VenusScan(GPS_SERIAL_SPEED, 20, VENUS_OUTPUT_MODE);

  char params[] = {VENUS_QUERY_WAAS,VENUS_QUERY_NAV_MODE};//,VENUS_QUERY_GPS_PINNING};  
  char values[sizeof(params)];

  /* Warning: This error checking code adds almost 100 code bytes */
  char errors[3] = {0,0,0}, e=0;
  

  // query options state
  venus_ctx.length = 0;
  for(char i=0; i<sizeof(params); i++) {
    venus_ctx.id = params[i];
    if(VenusWrite(VENUS_DEFAULT_TIMEOUT)==VENUS_OK)
      values[i] = venus_ctx.body[0];
  }
  bargraph_setled(STARTUP_QUERYOPTIONS);
  
  // enable WAAS  
  if(values[0] || VenusSetWAAS(1,1)==VENUS_OK)
    bargraph_setled(STARTUP_WAAS);
  else
    bargraph_displayerror(STARTUP_WAAS);
  
  // configure NAV MODE
  if(values[1] || VenusSetNavMode(1,1)==VENUS_OK)
    errors[0]=1;
    else e=1;

#if 0
  // disable POSITION PINNING
  if(!values[2] || VenusSetPositionPinning(0,1)==VENUS_OK)
    errors[1]=1;
    else e=1;
#else
  errors[1]=1;
#endif

  // NORMAL power mode
  if(VenusSetPowerMode(0,1)==VENUS_OK)
    errors[2]=1;
    else e=1;
    
  for(char i=0; i<3; i++) {
    bargraph_setled(i,errors[i]);
  }
  if(e)
    bargraph_displayerror(STARTUP_OPTIONS);
  else
    bargraph_setled(STARTUP_OPTIONS);
}

#endif  // ifdef VENUS6

