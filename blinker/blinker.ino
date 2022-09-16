
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <FastLED.h>
#include <math.h>
#include <EEPROM.h>

/* 
 *  
 * PWM output of left eye brightness
 * LEDs chained left, then right
 * Parameters:
 *     Frequency
 *     Waveform
 *     On/Off ratio
 *     left to right phase
 *     Brightness red
 *     Brightness green
 *     Brightness blue
 * Waveforms:
 *  0 Sinus (oo Ratio has no effect)
 *  1 Rect
 *  2 Triangle
 *  3 Saw
 */

#define N_LED_LEFT 12
#define N_LED_RIGHT 12
#define N_LEDS (N_LED_LEFT+N_LED_RIGHT)
#define DATA_PIN 4
#define PWM_PIN 11

#define DT_MS 5

#define N_WAVEFORMS 4


#define ADDRESS_USE_0 0
#define CONTENT_ID 0x8F

typedef int (*callback_int_t)(unsigned long);

callback_int_t callback_lst_int_t[N_WAVEFORMS];


typedef struct BlinkerSet
{
  float frequency;
  int waveform;
  float ooratio;
  float lrphase;
  int brght_red;
  int brght_green;
  int brght_blue;
  float stdF;
  float stdOOR;
  float stdBgt;
  int flashMs;
}blinkset;

blinkset _set = {1.0f,0,0.5f,0.0f,30,30,30,0.0f,0.0f,0.0f,0};
blinkset _rndSet = {1.0f,0,0.5f,0.0f,30,30,30,0.0f,0.0f,0.0f,0};
blinkset _default_set = {1.0f,0,0.5f,0.0f,50,50,50,0.0f,0.0f,0.0f,0};

unsigned long _last_time = millis();
unsigned long _last_start = millis();
unsigned long _rel_period_t = millis();
unsigned long _last_flash_t = millis();

int _forceOff = 0;

CRGB _leds[N_LEDS];

//char reciever_string[256] = {'\0'};


void updateSetRandomness(unsigned long t)
{
  static unsigned long last_t = 0;
  float f1;
  int r1,dt;

  dt = (int)((1.0f/_rndSet.frequency)*1000.0f);
  if( t-last_t >= dt )
  {
    r1 = random(100);
    f1 = ((float)r1)/100.0f*((_set.stdF)/100.0f);
    _rndSet.frequency = _set.frequency*(1+f1);
    if(_rndSet.frequency<=0.1f) _rndSet.frequency = 0.1f;

/*
    Serial.print("StdF: ");
    Serial.print(r1);
    Serial.print(" ");
    Serial.println((int)(_rndSet.frequency*10));
*/    
    r1 = random(100);
    f1 = ((float)r1)/100.0f*((_set.stdOOR)/100.0f);
    _rndSet.ooratio = _set.ooratio*(1+f1);
    if(_rndSet.stdOOR<=0.02f) _rndSet.stdOOR = 0.02f;
    else if(_rndSet.stdOOR>0.98f) _rndSet.stdOOR = 0.98f;
    
    r1 = random(100);
    f1 = ((float)r1)/100.0f*((_set.stdBgt)/100.0f);

    _rndSet.brght_red = (int)((float)(_set.brght_red)*(1+f1));
    if(_rndSet.brght_red<0) _rndSet.brght_red = 0;
    else if(_rndSet.brght_red>255) _rndSet.brght_red = 255;
    
    _rndSet.brght_green = (int)((float)(_set.brght_green)*(1+f1));
    if(_rndSet.brght_green<0) _rndSet.brght_green = 0;
    else if(_rndSet.brght_green>255) _rndSet.brght_green = 255;

    _rndSet.brght_blue = (int)((float)(_set.brght_blue)*(1+f1));
    if(_rndSet.brght_blue<0) _rndSet.brght_blue = 0;
    else if(_rndSet.brght_blue>255) _rndSet.brght_blue = 255;
/*
    Serial.print("StdB: ");
    Serial.print(r1);
    Serial.print(" ");
    Serial.print((int)(_set.stdBgt*100));
    Serial.print(" ");
    Serial.println(_rndSet.brght_blue);
*/
    _rel_period_t = last_t+dt;
    last_t = t;
    
    
  }
  
}

blinkset restoreSet( int *ok)
{
  blinkset res = _default_set;
  byte value = EEPROM.read(ADDRESS_USE_0);
  //Serial.println(value);
  if(value == CONTENT_ID)
  {
    if(ok) *ok = 1;
    EEPROM.get(ADDRESS_USE_0+1, res);
    return res;
  }
  
  if(ok) *ok = 0;
  return res;
}

void storeSet(blinkset set)
{
  EEPROM.update(ADDRESS_USE_0, CONTENT_ID);
  EEPROM.put(ADDRESS_USE_0+1, set);
  Serial.println("Stored set.");
}

void printset(blinkset set,int res)
{
    Serial.print("Set(");
    Serial.print(res);
    Serial.println("): ");
    Serial.print(set.frequency); Serial.print(",");
    Serial.print(set.waveform); Serial.print(",");
    Serial.print(set.ooratio); Serial.print(",");
    Serial.print(set.lrphase); Serial.print(",");
    Serial.print(set.brght_red); Serial.print(",");
    Serial.print(set.brght_green); Serial.print(",");
    Serial.print(set.brght_blue); Serial.print(",");
    Serial.print(set.stdF); Serial.print(",");
    Serial.print(set.stdOOR); Serial.print(",");
    Serial.print(set.stdBgt); Serial.print(",");
    Serial.println(set.flashMs);
}

// txt = str(frequency) + "," + str(waveform) + "," + str(onOffRatio) + "," + str(leftRightPhase) + "," + str(brightR) + "," + str(brightG) + "," + str(brightB) + "\n"
void readSetSerial()
{
  String str = Serial.readString();
  int res,w,r,g,b,i_f,io,ip,stdf,stdo,stdb,flashMs;

  Serial.println("Recieved string: ");
  Serial.println(str.c_str());
  
  res = sscanf(str.c_str(),"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",&i_f,&w,&io,&ip,&r,&g,&b,&stdf,&stdo,&stdb,&flashMs);

  _set.frequency=((float)i_f)/100.0f;
  _set.waveform=w;
  _set.ooratio=((float)io/100.0f);
  _set.brght_red=r;
  _set.brght_green=g;
  _set.brght_blue=b;
  _set.lrphase = ((float)(ip)/100.0f)/180.0f*M_PI;
  _set.stdF = stdf;
  _set.stdOOR = stdo;
  _set.stdBgt = stdb;
  _set.flashMs = flashMs;


  if(res == EOF || _set.waveform>N_WAVEFORMS || _set.waveform<0)
  {
    _set = _default_set;
    Serial.println("Failed - set to default.");
  }
  else
  {
    printset(_set,res);
  }

  storeSet(_set);
  _default_set = _set;
  
  _last_start = millis();
  _last_time = 0;
}

CRGB calculateRectColor(float t, float f, float ooratio, float phase, int max_red, int max_green, int max_blue)
{
  float v1,v3;
  int v2;
  float t_phase = phase/(2*M_PI)/f;
  
  
  v1 = ((t+t_phase)*f);
  v2 = (int)v1;

  v3 = v1-v2;

  if(v3<ooratio)
  {
    return CRGB(max_red,max_green,max_blue);
  }

  return CRGB(0,0,0);
}

CRGB calculateSinColor(float t, float f, float phase, int max_red, int max_green, int max_blue)
{
  int r,g,b;
  float fkt;
  
  fkt = (sin(2*M_PI*f*t+phase)*0.5f)+0.5f;
  
  r =  (int)(fkt*max_red+0.5f);
  g =  (int)(fkt*max_green+0.5f);
  b =  (int)(fkt*max_blue+0.5f);
  
  return CRGB(r,g,b);
}

CRGB calculateTriangleColor(float t, float f, float ooratio, float phase, int max_red, int max_green, int max_blue)
{
  int r,g,b;
  float v1,v3,fkt;
  int v2;
  float t_phase = phase/(2*M_PI)/f;
  
  fkt = 0;
  v1 = ((t+t_phase)*f);
  v2 = (int)v1;

  v3 = v1-v2; // relative position in period 0 - 1
  
  //  0 - 0.5 raising from 0 to 1
  if(v3<=0.5f)
    fkt = 2*v3;
  // 0.5 - 1 reducing from 1 to 0
  else
    fkt = 1.0f-2.0f*(v3-0.5f);

  r =  (int)(fkt*max_red+0.5f);
  g =  (int)(fkt*max_green+0.5f);
  b =  (int)(fkt*max_blue+0.5f);
  
  return CRGB(r,g,b);
}

CRGB calculateSawColor(float t, float f, float ooratio, float phase, int max_red, int max_green, int max_blue)
{
  int r,g,b;
  float v1,v3,fkt;
  int v2;
  float t_phase = phase/(2*M_PI)/f;
  
  fkt = 0;
  v1 = ((t+t_phase)*f);
  v2 = (int)v1;

  v3 = v1-v2; // relative position in period 0 - 1
  
  //  0->1, 1->0
  fkt = 1-v3;

  r =  (int)(fkt*max_red+0.5f);
  g =  (int)(fkt*max_green+0.5f);
  b =  (int)(fkt*max_blue+0.5f);
  
  return CRGB(r,g,b);
}

int getPercivedBrightness(CRGB col)
{
  float percCol = 0.3f*(float)(col.r)+0.59f*(float)(col.g)+0.11f*(float)(col.b);
  int percBr = (int)percCol;
  
  if(percBr>255) percBr=255;
  else if(percBr<0) percBr=0;

  return percBr;
}

int maxBrightness(CRGB rcol,CRGB lcol)
{
  int b1,b2;
  b1 = getPercivedBrightness(rcol);
  b2 = getPercivedBrightness(lcol);
  if(b2>b1) return b2;
  return b1;
}

void fillLEDArray(CRGB lcol, CRGB rcol)
{
  int i;
  
  for(i=0;i<N_LED_LEFT;i++)
    _leds[i] = lcol;
    
  for(i=N_LED_LEFT;i<N_LEDS;i++)
    _leds[i] = rcol;
}

int setSineWaveform(unsigned long t)
{
  // left, right, fill array.
  CRGB rcol,lcol;
  //unsigned long t = millis();
  float t_s;

  //t_s = (float)(t-_last_start)/1e3f;
  t_s = (float)t/1e3f;
/*
  Serial.print((int)(10*_rndSet.frequency));
  Serial.print(",");
  Serial.println(t_s);
*/  
  lcol = calculateSinColor(t_s, _rndSet.frequency, 0,_rndSet.brght_red,_rndSet.brght_green,_rndSet.brght_blue);
  rcol = calculateSinColor(t_s, _rndSet.frequency, _set.lrphase,_rndSet.brght_red,_rndSet.brght_green,_rndSet.brght_blue);

  fillLEDArray(lcol,rcol);

  return getPercivedBrightness(lcol);
}

int setRectWaveform(unsigned long t)
{
  CRGB rcol,lcol;
  //unsigned long t = millis();
  float t_s;

  //t_s = (float)(t-_last_start)/1e3f;
  t_s = (float)t/1e3f;

  lcol = calculateRectColor(t_s, _rndSet.frequency, _rndSet.ooratio, 0,_rndSet.brght_red,_rndSet.brght_green,_rndSet.brght_blue);
  rcol = calculateRectColor(t_s, _rndSet.frequency, _rndSet.ooratio, _set.lrphase,_rndSet.brght_red,_rndSet.brght_green,_rndSet.brght_blue);

  fillLEDArray(lcol,rcol);

  return getPercivedBrightness(lcol);
}

int setTriangleWaveform(unsigned long t)
{
  CRGB rcol,lcol;
  //unsigned long t = millis();
  float t_s;

  //t_s = (float)(t-_last_start)/1e3f;
  t_s = (float)t/1e3f;
  
  lcol = calculateTriangleColor(t_s, _rndSet.frequency, _rndSet.ooratio, 0,_rndSet.brght_red,_rndSet.brght_green,_rndSet.brght_blue);
  rcol = calculateTriangleColor(t_s, _rndSet.frequency, _rndSet.ooratio, _set.lrphase,_rndSet.brght_red,_rndSet.brght_green,_rndSet.brght_blue);

  fillLEDArray(lcol,rcol);

  return getPercivedBrightness(lcol);
}


int setSawWaveform(unsigned long t)
{
  CRGB rcol,lcol;
  //unsigned long t = millis();
  float t_s;

  //t_s = (float)(t-_last_start)/1e3f;
  t_s = (float)t/1e3f;
  
  lcol = calculateSawColor(t_s, _rndSet.frequency, _rndSet.ooratio, 0,_rndSet.brght_red,_rndSet.brght_green,_rndSet.brght_blue);
  rcol = calculateSawColor(t_s, _rndSet.frequency, _rndSet.ooratio, _set.lrphase,_rndSet.brght_red,_rndSet.brght_green,_rndSet.brght_blue);

  fillLEDArray(lcol,rcol);

  return getPercivedBrightness(lcol);
}

void fillArrayWithColor(CRGB*arr, CRGB col, int N)
{
  for(int i=0; i<N; i++) arr[i] = col;
}


void setPWM(int val)
{
  if(val>255) val=255;
  else if(val<0) val = 0;
  /*Serial.print(millis());
  Serial.print(" ");
  Serial.println(val);*/
  analogWrite(PWM_PIN,val);
}

void writeOutput(int val)
{
  Serial.print("\x02");
  Serial.print(millis()-_last_start);
  Serial.print(",");
  Serial.print(val);
  Serial.print("\n");
}

int updateLEDs(unsigned long t)
{
  int val = callback_lst_int_t[_set.waveform](t);
  setPWM(val);
  writeOutput(val);
  if(_set.flashMs>0)
  {
    if(t-_last_flash_t > _set.flashMs)
    {
      _forceOff = !_forceOff;
      _last_flash_t = t;
    }
  }
  else
     _forceOff = 0;

  if(_forceOff) fillLEDArray(CRGB(0,0,0),CRGB(0,0,0));
    
  FastLED.show();
  return val;
}

void setup()
{
  int ok;

  randomSeed(analogRead(0));
  
  blinkset btmp = _default_set;
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.setTimeout(25);

  Serial.println("F*100,Waveform,On/OffRatio*100,LRPhase(deg)*100,Red,Green,Blue,stdF%,stdOOR%,stdBgt%");
  Serial.println("Waveforms: 0 Sin, 1 Rect, 2 Triangle, 3 Saw");
  Serial.println("Eg1: 100,0,50,0,50,50,50,0,0,0,0");
  Serial.println("Eg2: 150,1,30,9000,55,55,55,20,20,20,100");
   
  
  CRGB nextCol = CRGB(4,4,4);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(_leds, N_LEDS);
  fillArrayWithColor(_leds, nextCol, N_LEDS);
  FastLED.show();


  pinMode(PWM_PIN, OUTPUT);  // sets the pin as output
  //TCCR2B = TCCR2B & B11111000 | B00000010; // for PWM on Pin 3 & 11 frequency of 3921.16 Hz
  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  setPWM(0);

  callback_lst_int_t[0] = setSineWaveform;
  callback_lst_int_t[1] = setRectWaveform;
  callback_lst_int_t[2] = setTriangleWaveform;
  callback_lst_int_t[3] = setSawWaveform;

  btmp = restoreSet( &ok);
  if(ok)
  {
    Serial.println("Restored set from EEPROM");
    printset(btmp,0);
    _default_set = _set = btmp;
  }

  _last_start = millis();
}

void loop()
{
  int val = 0;
  unsigned long time_ms = millis();

  if(time_ms-_last_time>=DT_MS) // update LED AND pwm
  {
    updateSetRandomness(time_ms);
    val = updateLEDs(time_ms-_rel_period_t);
    //setPWM(val);
    _last_time = time_ms;
  }

  if( Serial.available() > 0 ) readSetSerial();
  
}
