#if defined(UBLOX)

	struct ubx_header {
		uint8_t preamble1;
		uint8_t preamble2;
		uint8_t msg_class;
		uint8_t msg_id;
		uint16_t length;
	};

    struct ubx_nav_posllh {
        uint32_t	time;				// GPS msToW
        int32_t		longitude;
        int32_t		latitude;
        int32_t		altitude_ellipsoid;
        int32_t		altitude_msl;
        uint32_t	horizontal_accuracy;
        uint32_t	vertical_accuracy;
    };
    struct ubx_nav_status {
        uint32_t	time;				// GPS msToW
        uint8_t		fix_type;
        uint8_t		fix_status;
        uint8_t		differential_status;
        uint8_t		res;
        uint32_t	time_to_first_fix;
        uint32_t	uptime;				// milliseconds
    };
    struct ubx_nav_solution {
        uint32_t	time;
        int32_t		time_nsec;
        int16_t		week;
        uint8_t		fix_type;
        uint8_t		fix_status;
        int32_t		ecef_x;
        int32_t		ecef_y;
        int32_t		ecef_z;
        uint32_t	position_accuracy_3d;
        int32_t		ecef_x_velocity;
        int32_t		ecef_y_velocity;
        int32_t		ecef_z_velocity;
        uint32_t	speed_accuracy;
        uint16_t	position_DOP;
        uint8_t		res;
        uint8_t		satellites;
        uint32_t	res2;
    };
    struct ubx_nav_velned {
        uint32_t	time;				// GPS msToW
        int32_t		ned_north;
        int32_t		ned_east;
        int32_t		ned_down;
        uint32_t	speed_3d;
        uint32_t	speed_2d;
        int32_t		heading_2d;
        uint32_t	speed_accuracy;
        uint32_t	heading_accuracy;
    };

    enum ubs_protocol_bytes {
        PREAMBLE1 = 0xb5,
        PREAMBLE2 = 0x62,
        CLASS_NAV = 0x01,
        CLASS_ACK = 0x05,
        CLASS_CFG = 0x06,
		MSG_ACK_NACK = 0x00,
		MSG_ACK_ACK = 0x01,
        MSG_POSLLH = 0x2,
        MSG_STATUS = 0x3,
        MSG_SOL = 0x6,
        MSG_VELNED = 0x12,
        MSG_CFG_PRT = 0x00,
        MSG_CFG_RATE = 0x08,
        MSG_CFG_SET_RATE = 0x01,
		MSG_CFG_NAV_SETTINGS = 0x24
    };
    enum ubs_nav_fix_type {
        FIX_NONE = 0,
        FIX_DEAD_RECKONING = 1,
        FIX_2D = 2,
        FIX_3D = 3,
        FIX_GPS_DEAD_RECKONING = 4,
        FIX_TIME = 5
    };
    enum ubx_nav_status_bits {
        NAV_STATUS_FIX_VALID = 1
    };

    // Packet checksum accumulators
    static uint8_t		_ck_a;
    static uint8_t		_ck_b;

    // State machine state
    static uint8_t		_step;
    static uint8_t		_msg_id;
    static uint16_t	_payload_length;
    static uint16_t	_payload_counter;

    static bool        next_fix;
    
    static uint8_t     _class;

	// do we have new position information?
	static bool		_new_position;

	// do we have new speed information?
	static bool		_new_speed;

	static uint8_t	    _disable_counter;

    // Receive buffer
    static union {
        ubx_nav_posllh		posllh;
        ubx_nav_status		status;
        ubx_nav_solution	solution;
        ubx_nav_velned		velned;
        uint8_t	bytes[];
    } _buffer;

void _update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b)
{
	while (len--) {
		ck_a += *data;
		ck_b += ck_a;
		data++;
	}
}

bool GPS_UBLOX_newFrame(uint8_t data)
{
       bool parsed = false;

        switch(_step) {

        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
        case 0:
            if(PREAMBLE1 == data) _step++;
            break;

        case 2:
            _step++;
	    _class = data;
	    _ck_b = _ck_a = data;			// reset the checksum accumulators
            break;
        case 3:
            _step++;
            _ck_b += (_ck_a += data);			// checksum byte
            _msg_id = data;
            break;
        case 4:
            _step++;
            _ck_b += (_ck_a += data);			// checksum byte
            _payload_length = data;				// payload length low byte
            break;
        case 5:
            _step++;
            _ck_b += (_ck_a += data);			// checksum byte

            _payload_length += (uint16_t)(data<<8);
			if (_payload_length > 512) {
				_payload_length = 0;
				_step = 0;
			}
            _payload_counter = 0;				// prepare to receive payload
            break;
        case 6:
            _ck_b += (_ck_a += data);			// checksum byte
			if (_payload_counter < sizeof(_buffer)) {
				_buffer.bytes[_payload_counter] = data;
			}
            if (++_payload_counter == _payload_length)
                _step++;
            break;
        case 7:
            _step++;
            if (_ck_a != data) _step = 0;						// bad checksum
            break;
        case 8:
            _step = 0;
            if (_ck_b != data)  break; 							// bad checksum
 	    if (UBLOX_parse_gps())  { parsed = true; }
        } //end switch
   return parsed;
}

bool UBLOX_parse_gps(void)
{

    switch (_msg_id) {
    case MSG_POSLLH:
        i2c_dataset.time	        = _buffer.posllh.time;
        GPS_read[LON]	                = _buffer.posllh.longitude;
        GPS_read[LAT]	                = _buffer.posllh.latitude;
        i2c_dataset.altitude  	        = _buffer.posllh.altitude_msl / 10 /100;      //alt in m
	i2c_dataset.status.gps3dfix	= next_fix;
	_new_position = true;
	break;
    case MSG_STATUS:
        next_fix	= (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
	if (!next_fix) i2c_dataset.status.gps3dfix = false;
        break;
    case MSG_SOL:
        next_fix	= (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
	if (!next_fix) i2c_dataset.status.gps3dfix = false;
        i2c_dataset.status.numsats	= _buffer.solution.satellites;
        //GPS_hdop		= _buffer.solution.position_DOP;
        //debug[3] = GPS_hdop;
        break;
    case MSG_VELNED:
        //speed_3d	= _buffer.velned.speed_3d;				// cm/s
        i2c_dataset.ground_speed = _buffer.velned.speed_2d;				// cm/s
        i2c_dataset.ground_course = (uint16_t)(_buffer.velned.heading_2d / 10000);	// Heading 2D deg * 100000 rescaled to deg * 10
	_new_speed = true;
        break;
    default:
        return false;
    }

	// we only return true when we get new position and speed data
	// this ensures we don't use stale data
	if (_new_position && _new_speed) {
		_new_speed = _new_position = false;
		return true;
	}
	return false;
}


#endif //UBLOX
