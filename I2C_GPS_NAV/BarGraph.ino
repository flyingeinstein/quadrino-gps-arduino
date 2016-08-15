

#ifdef BARGRAPH_DISPLAY

const byte bargraph_pins[] = {9,8,7,6,5,A2,A0,A3,4,3};
const byte bargraph_dots = sizeof(bargraph_pins)/sizeof(bargraph_pins[0]);

uint32_t reenable_satdisplay=0;


byte bargraph_value=0xff;
byte bargraph_anim=BARGRAPH_NONE;
byte bargraph_status=0;
unsigned long long bargraph_timer=0;

void bargraph_init()
{
  for(char i=0; i<bargraph_dots; i++) {
    pinMode(bargraph_pins[i], OUTPUT);
    digitalWrite(bargraph_pins[i], LOW);
  }
  bargraph_value=0;
  bargraph_timer=0;
}

void bargraph_animate(byte anim)
{
  bargraph_anim=anim;
}

void bargraph_set(byte anim, byte value)
{
  bargraph_anim=anim;
  bargraph_value=value;
}

#if 0
void bargraph_nextAnimation()
{
  bargraph_anim++;
  if(bargraph_anim>BARGRAPH_LAST) bargraph_anim=1;
  bargraph_reset();
}  

inline byte bargraph_getAnimation()
{
  return bargraph_anim;
}
#endif

//void bargraph_displayerror(byte code, int blinks, int interdelay)
void bargraph_displayerror(byte code)
{
  for(char i=0; i<5; i++) {
    bargraph_setled(code, true);
    delay(250);
    bargraph_setled(code, false);
    delay(250);
  }  
}

void bargraph_reset()
{
  switch(bargraph_anim)
  {
#if defined(BARGRAPH_SINGLESHOT_DOWN) || defined(BARGRAPH_MULTISHOT_DOWN)
#ifdef BARGRAPH_SINGLESHOT_DOWN
    case BARGRAPH_SINGLESHOT_DOWN:
#endif
#ifdef BARGRAPH_MULTISHOT_DOWN
    case BARGRAPH_MULTISHOT_DOWN:
#endif
      bargraph_value=bargraph_dots;
      break;
#endif
    default:
      bargraph_value=0;
      break;
  }
}

void  bargraph_single(byte n)
{
  for(char i=0; i<bargraph_dots; i++)
    digitalWrite(bargraph_pins[i], (i==n) ? LOW : HIGH);
}

#if 0
void  bargraph_mirrored(byte n)
{
  byte dots = bargraph_dots>>1;
  for(char i=0,j=bargraph_dots-1; i<dots; i++,j--) {
    digitalWrite(bargraph_pins[i], (i==n) ? LOW : HIGH);
    digitalWrite(bargraph_pins[j], (i==n) ? LOW : HIGH);
  }
}

void  bargraph_alternate(byte n)
{
  if(n) {
    for(char i=0; i<bargraph_dots; i++)
      digitalWrite(bargraph_pins[i], (i&1) ? LOW : HIGH);
  } else {
    for(char i=0; i<bargraph_dots; i++)
      digitalWrite(bargraph_pins[i], (i&1) ? HIGH : LOW);
  }
}
#endif

void  bargraph_upto(byte n)
{
  if(n>=10) {
    //bargraph_upto_invert(n-10);
    n-=10;
    for(char i=0; i<bargraph_dots; i++)
      digitalWrite(bargraph_pins[i], (i<n) ? HIGH : LOW);
  } else {
    for(char i=0; i<bargraph_dots; i++)
      digitalWrite(bargraph_pins[i], (i<n) ? LOW : HIGH);
  }
}

# if 0
void  bargraph_upto_invert(byte n)
{
  for(char i=0; i<bargraph_dots; i++)
    digitalWrite(bargraph_pins[i], (i<n) ? HIGH : LOW);
}
#endif

void bargraph_update()
{
  if(bargraph_timer<millis()) {
    bargraph_timer = millis()+BARGRAPH_PERIOD;
    switch(bargraph_anim) {
      case BARGRAPH_NONE: 
        break;

      case BARGRAPH_UPTO: 
        bargraph_upto(bargraph_value); // nothing
        break;

#ifdef BARGRAPH_KIT
      case BARGRAPH_KIT:
        bargraph_value++;
        if(bargraph_value < bargraph_dots)
          bargraph_single(bargraph_value); 
        else if(bargraph_value < bargraph_dots<<1)
          bargraph_single((bargraph_dots<<1) - bargraph_value-1); 
        else
          bargraph_single(bargraph_value=0);
        break;
#endif
/*
#ifdef BARGRAPH_SINGLESHOT
      case BARGRAPH_SINGLESHOT:
        bargraph_value++;
        if(bargraph_value < bargraph_dots)
          bargraph_single(bargraph_value); 
        else
          bargraph_single(0xff);
        break;
#endif
      
#ifdef BARGRAPH_MULTISHOT
      case BARGRAPH_MULTISHOT:
        bargraph_value++;
        if(bargraph_value < bargraph_dots)
          bargraph_single(bargraph_value); 
        else
          bargraph_single(bargraph_value=0);
        break;
#endif

#ifdef BARGRAPH_SINGLESHOT_DOWN
      case BARGRAPH_SINGLESHOT_DOWN:
        if(bargraph_value >0)
          bargraph_single(--bargraph_value); 
        else
          bargraph_single(0xff); 
        break;
#endif

#ifdef BARGRAPH_MULTISHOT_DOWN
      case BARGRAPH_MULTISHOT_DOWN:
        if(bargraph_value >0)
          bargraph_single(--bargraph_value);
        else
          bargraph_single(bargraph_value=bargraph_dots); 
        break;
#endif
*/
#ifdef BARGRAPH_FANOUT
      case BARGRAPH_FANOUT:
        bargraph_value++;
        if(bargraph_value > bargraph_dots>>1)
          bargraph_value=0;
        bargraph_mirrored(bargraph_value);
        break;
#endif

#ifdef BARGRAPH_FANIN
      case BARGRAPH_FANIN:
        if(bargraph_value == 0)
          bargraph_value=bargraph_dots>>1;
        else
          bargraph_value--;
        bargraph_mirrored(bargraph_value);
        break;
#endif

#ifdef BARGRAPH_SHIFTER
      case BARGRAPH_SHIFTER:
        bargraph_alternate(bargraph_value = !bargraph_value);
        break;
#endif

#ifdef BARGRAPH_LASER
      case BARGRAPH_LASER:
        bargraph_value++;
#if 0
        if(bargraph_value < bargraph_dots)
          bargraph_upto(bargraph_value); 
        else if(bargraph_value < bargraph_dots<<1)
          bargraph_upto_invert(bargraph_value-bargraph_dots); 
#else
        if(bargraph_value <= bargraph_dots<<1)
          bargraph_upto(bargraph_value); 
#endif
        else
          bargraph_single(bargraph_value=0);
        break;
#endif

#if 0
    default: // no anim handler, inc to the next anim
      bargraph_anim++;
      if(bargraph_anim>BARGRAPH_LAST)
        bargraph_anim=0;
      break;
#endif
    }
  }
}

#endif

