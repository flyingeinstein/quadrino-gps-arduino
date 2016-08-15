//////////////////////////////////////////////////////////////////////////////
// bargraph display definitions
//


#define BARGRAPH_NONE               0    // no animation, just display the value
#define BARGRAPH_UPTO               1    // no animation, just display the value
#define BARGRAPH_KIT                2    // kit the car
//#define BARGRAPH_SINGLESHOT         3    // single up bullet
//#define BARGRAPH_MULTISHOT          4    // same as single shot but repeats
//#define BARGRAPH_SINGLESHOT_DOWN    5    // single up bullet
//#define BARGRAPH_MULTISHOT_DOWN     6    // same as single shot but repeats
//#define BARGRAPH_FANOUT             7    // middle LEds to outer LEDs
//#define BARGRAPH_FANIN              8    // both outer to LEDs inward
//#define BARGRAPH_SHIFTER            9    // every second light on, then shift
#define BARGRAPH_LASER              10    // turn on all leds in sequence, then all off in sequence
#define BARGRAPH_LAST               BARGRAPH_LASER

// you can lower this to increase the animation speed but it takes more CPU
#define BARGRAPH_PERIOD              35


// During startup the LEDs are switched on as each step in the process completes
#define STARTUP_BAUDRATE          3
#define STARTUP_MSGMODE           4
#define STARTUP_UPDATERATE        5
#define STARTUP_QUERYOPTIONS      6
#define STARTUP_WAAS              7
#define STARTUP_OPTIONS           8
#define STARTUP_SDCARD            9



extern const byte bargraph_pins[];// = {9,8,7,6,5,A2,A0,A3,4,3};
extern uint32_t reenable_satdisplay;


//void bargraph_displayerror(byte code, int blinks=5, int interdelay=250);

inline void bargraph_setled(byte led, bool state=true)
{
    digitalWrite(bargraph_pins[led], state ? LOW : HIGH);
}

void bargraph_toggleled(byte led, bool state)
{
  byte pin = bargraph_pins[led];
  digitalWrite(pin, !digitalRead(pin));
}

inline void suppressSatDisplay(int _millis)
{
  reenable_satdisplay = millis()+_millis;
}

