
#if defined(VENUS6)

#define VenusSerial Serial

#define VENUS_DEFAULT_TIMEOUT    3000
#define VENUS_MAX_PAYLOAD        60
#define VENUS_MAX_BAUDRATE       115200 //230400
#define VENUS_MIN_BAUDRATE       4800

/* Venus6 message codes */
        // Input System Messages
#define VENUS_SYS_RESTART                 0x01
#define VENUS_QUERY_SW_VERSION            0x02
#define VENUS_QUERY_SW_CRC                0x03
#define VENUS_SET_FACTORY_DEFAULTS        0x04
#define VENUS_CONFIG_SERIAL_PORT          0x05
#define VENUS_CONFIG_NMEA                 0x08
#define VENUS_CONFIG_OUTPUT_MSG_FORMAT    0x09
#define VENUS_CONFIG_POWER_MODE           0x0C
#define VENUS_CONFIG_GPS_UPDATE_RATE      0x0E
#define VENUS_QUERY_GPS_UPDATE_RATE       0x10
        // Input GPS Messages
#define VENUS_CONFIG_DATUM                0x29
#define VENUS_QUERY_DATUM                 0x2D
#define VENUS_GET_EPHEMERIS               0x30
#define VENUS_SET_EPHEMERIS               0x31
#define VENUS_CONFIG_WAAS                 0x37
#define VENUS_QUERY_WAAS                  0x38
#define VENUS_CONFIG_GPS_PINNING          0x39
#define VENUS_QUERY_GPS_PINNING           0x3A
#define VENUS_CONFIG_GPS_PINNING_PARAMS   0x3B
#define VENUS_CONFIG_NAV_MODE             0x3C
#define VENUS_QUERY_NAV_MODE              0x3D
#define VENUS_CONFIG_1PPS_MODE            0x3E
#define VENUS_QUERY_1PPS_MODE             0x3F
        // Output System Messages
#define VENUS_REPORT_SW_VERSION           0x80
#define VENUS_REPORT_SW_CRC               0x81
#define VENUS_REPORT_GPS_UPDATE_RATE      0x86
#define VENUS_REPORT_EPHEMERIS            0xB1
        // Output GPS Messages
#define VENUS_GPS_LOCATION                0xA8
#define VENUS_GPS_DATUM                   0xAE
#define VENUS_GPS_WAAS_STATUS             0xB3
#define VENUS_GPS_PINNING_STATUS          0xB4
#define VENUS_GPS_NAV_MODE                0xB5
#define VENUS_GPS_1PPS_MODE               0xB6
        // Request Acknowledge
#define VENUS_ACK                         0x83
#define VENUS_NACK                        0x84

#define VENUS_INT16(h,l)   ((h<<8)|l)

// Read/write errors (will be char)
#define VENUS_OK           0  // read successful
#define VENUS_TIMEOUT     -1  // no data received within timeout period
#define VENUS_MORE        -2  // more data remaining
#define VENUS_NACKED      -3  // message reply was a nack
#define VENUS_ACK_NOREPLY -4  // message was incomplete
#define VENUS_BADCR       -5  // checksum failure
#define VENUS_INCOMPLETE  -6  // message was incomplete
#define VENUS_CLIPPED     -7  // message buffer too small, response payload was clipped



// typical SRAM or FLASH attribute
#define VENUS_SRAM            0
#define VENUS_FLASH           1

// message output mode
#define VENUS_OUTPUT_NONE        0
#define VENUS_OUTPUT_NMEA        1
#define VENUS_OUTPUT_BINARY      2

#if defined(NMEA)
#define VENUS_OUTPUT_MODE VENUS_OUTPUT_NMEA
#else
#define VENUS_OUTPUT_MODE VENUS_OUTPUT_BINARY
#endif


typedef struct {
  int32_t x;
  int32_t y;
  int32_t z;
} xyz32_t;

typedef struct {
  uint8_t  fixmode;
  uint8_t  sv_count;  // satellites
  uint16_t gps_week;
  uint32_t gps_tow;
  int32_t latitude;
  int32_t longitude;
  uint32_t ellipsoid_alt;
  uint32_t sealevel_alt;
  uint16_t gdop, pdop, hdop, vdop, tdop;
  xyz32_t ecef;
  xyz32_t vel;
} venus_location;

typedef struct _venus_message {
  unsigned char length;       // payload length;
  byte id;    // message id
  union {
    byte body[VENUS_MAX_PAYLOAD];
    venus_location location;
  };
} venus_message;

extern venus_message venus_ctx;

void VenusScan(unsigned long desiredBaud=57600, byte desiredUpdateRate=1, byte desiredMessageFormat=VENUS_OUTPUT_BINARY);
char VenusSetUpdateRate(byte updaterate, bool flashtrue);

#endif
