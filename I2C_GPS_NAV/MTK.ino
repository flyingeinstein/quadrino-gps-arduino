
#if defined(MTK)
    struct diyd_mtk_msg {
        int32_t		latitude;
        int32_t		longitude;
        int32_t		altitude;
        int32_t		ground_speed;
        int32_t		ground_course;
        uint8_t		satellites;
        uint8_t		fix_type;
        uint32_t	utc_date;
        uint32_t	utc_time;
        uint16_t	hdop;
    };
// #pragma pack(pop)
    enum diyd_mtk_fix_type {
        FIX_NONE = 1,
        FIX_2D = 2,
        FIX_3D = 3
    };

    enum diyd_mtk_protocol_bytes {
        PREAMBLE1 = 0xd0,
        PREAMBLE2 = 0xdd,
    };

    // Packet checksum accumulators
    uint8_t 	_ck_a;
    uint8_t 	_ck_b;

    // State machine state
    uint8_t 	_step;
    uint8_t		_payload_counter;

    // Time from UNIX Epoch offset
    long		_time_offset;
    bool		_offset_calculated;

    // Receive buffer
    union {
        diyd_mtk_msg	msg;
        uint8_t			bytes[];
    } _buffer;

inline long _swapl(const void *bytes)
{
    const uint8_t	*b = (const uint8_t *)bytes;
    union {
        long	v;
        uint8_t b[4];
    } u;

    u.b[0] = b[3];
    u.b[1] = b[2];
    u.b[2] = b[1];
    u.b[3] = b[0];

    return(u.v);
}

void GPSModuleInit()
{
  /* Using the AXN 1.51 firmware which defaults at 1Hz/38400 but supports binary protocol
   * First connect to it with 38400, then set the speed to 115200 
   * and set the update rate to 10Hz
   * and finally switch to Binary mode
   */
  
  Serial.begin(38400);
  delay(1500); //let it init
  Serial.write("$PMTK251,115200*1F\r\n");
  delay(300);
  Serial.end();
  
  Serial.begin(115200);
  Serial.write("$PGCMD,16,0,0,0,0,0*6A\r\n");
  delay(1000);
  Serial.write("$PGCMD,16,0,0,0,0,0*6A\r\n");
  delay(300);
  Serial.write("$PMTK220,100*2F\r\n");
  return true;
}


bool GPS_MTK_newFrame(uint8_t data)
{
       bool parsed = false;

restart:
        switch(_step) {

            // Message preamble, class, ID detection
            //
            // If we fail to match any of the expected bytes, we
            // reset the state machine and re-consider the failed
            // byte as the first byte of the preamble.  This
            // improves our chances of recovering from a mismatch
            // and makes it less likely that we will be fooled by
            // the preamble appearing as data in some other message.
            //
        case 0:
            if(PREAMBLE1 == data)
                _step++;
            break;
        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
            goto restart;
        case 2:
            if (sizeof(_buffer) == data) {
                _step++;
                _ck_b = _ck_a = data;				// reset the checksum accumulators
                _payload_counter = 0;
            } else {
                _step = 0;							// reset and wait for a message of the right class
                goto restart;
            }
            break;

            // Receive message data
            //
        case 3:
            _buffer.bytes[_payload_counter++] = data;
            _ck_b += (_ck_a += data);
            if (_payload_counter == sizeof(_buffer))
                _step++;
            break;

            // Checksum and message processing
            //
        case 4:
            _step++;
            if (_ck_a != data) {
                _step = 0;
            }
            break;
        case 5:
            _step = 0;
            if (_ck_b != data) {
                break;
            }

            i2c_dataset.status.gps3dfix 			= _buffer.msg.fix_type == FIX_3D;
            GPS_read[LAT]		= _buffer.msg.latitude * 10;	// XXX doc says *10e7 but device says otherwise
            GPS_read[LON]		= _buffer.msg.longitude * 10;	// XXX doc says *10e7 but device says otherwise
            i2c_dataset.altitude		= _buffer.msg.altitude /100;
            i2c_dataset.ground_speed	                = _buffer.msg.ground_speed;
            i2c_dataset.ground_course	        = _buffer.msg.ground_course;
            i2c_dataset.status.numsats		        = _buffer.msg.satellites;
            //GPS_hdop			= _buffer.msg.hdop;
            parsed = true;
            //GPS_Present = 1;
        }
    return parsed;
}
#endif //MTK

