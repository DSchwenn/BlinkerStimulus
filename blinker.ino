
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <FastLED.h>
#include <math.h>

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

typedef int (*callback_int_t)();

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
}blinkset;

blinkset _set = {1.0f,0,0.5f,0.0f,30,30,30};
blinkset _default_set = {1.0f,0,0.5f,0.0f,50,50,50};
unsigned long _last_time = millis();
unsigned long _last_start = millis();

CRGB _leds[N_LEDS];

//char reciever_string[256] = {'\0'};


// txt = str(frequency) + "," + str(waveform) + "," + str(onOffRatio) + "," + str(leftRightPhase) + "," + str(brightR) + "," + str(brightG) + "," + str(brightB) + "\n"
void readSetSerial()
{
  String str = Serial.readString();
  int res,w,r,g,b,i_f,io,ip;

  Serial.println("Recieved string: ");
  Serial.println(str.c_str());
  
  res = sscanf(str.c_str(),"%d,%d,%d,%d,%d,%d,%d\n",&i_f,&w,&io,&ip,&r,&g,&b);

  _set.frequency=((float)i_f)/100.0f;
  _set.waveform=w;
  _set.ooratio=((float)io/100.0f);
  _set.brght_red=r;
  _set.brght_green=g;
  _set.brght_blue=b;
  _set.lrphase = ((float)(ip)/100.0f)/180.0f*M_PI;


  if(res == EOF || _set.waveform>N_WAVEFORMS || _set.waveform<0)
  {
    _set = _default_set;
    Serial.println("Failed - set to default.");
  }
  else
  {
    Serial.print("Set(");
    Serial.print(res);
    Serial.println("): ");
    Serial.print(_set.frequency); Serial.print(",");
    Serial.print(_set.waveform); Serial.print(",");
    Serial.print(_set.ooratio); Serial.print(",");
    Serial.print(_set.lrphase); Serial.print(",");
    Serial.print(_set.brght_red); Serial.print(",");
    Serial.print(_set.brght_green); Serial.print(",");
    Serial.println(_set.brght_blue);
  }
  
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

int setSineWaveform()
{
  // left, right, fill array.
  CRGB rcol,lcol;
  unsigned long t = millis();
  float t_s;

  t_s = (float)(t-_last_start)/1e3f;
  
  lcol = calculateSinColor(t_s, _set.frequency, 0,_set.brght_red,_set.brght_green,_set.brght_blue);
  rcol = calculateSinColor(t_s, _set.frequency, _set.lrphase,_set.brght_red,_set.brght_green,_set.brght_blue);

  fillLEDArray(lcol,rcol);

  return getPercivedBrightness(lcol);
}

int setRectWaveform()
{
  CRGB rcol,lcol;
  unsigned long t = millis();
  float t_s;

  t_s = (float)(t-_last_start)/1e3f;

  lcol = calculateRectColor(t_s, _set.frequency, _set.ooratio, 0,_set.brght_red,_set.brght_green,_set.brght_blue);
  rcol = calculateRectColor(t_s, _set.frequency, _set.ooratio, _set.lrphase,_set.brght_red,_set.brght_green,_set.brght_blue);

  fillLEDArray(lcol,rcol);

  return getPercivedBrightness(lcol);
}

int setTriangleWaveform()
{
  CRGB rcol,lcol;
  unsigned long t = millis();
  float t_s;

  t_s = (float)(t-_last_start)/1e3f;
  
  lcol = calculateTriangleColor(t_s, _set.frequency, _set.ooratio, 0,_set.brght_red,_set.brght_green,_set.brght_blue);
  rcol = calculateTriangleColor(t_s, _set.frequency, _set.ooratio, _set.lrphase,_set.brght_red,_set.brght_green,_set.brght_blue);

  fillLEDArray(lcol,rcol);

  return getPercivedBrightness(lcol);
}


int setSawWaveform()
{
  CRGB rcol,lcol;
  unsigned long t = millis();
  float t_s;

  t_s = (float)(t-_last_start)/1e3f;
  
  lcol = calculateSawColor(t_s, _set.frequency, _set.ooratio, 0,_set.brght_red,_set.brght_green,_set.brght_blue);
  rcol = calculateSawColor(t_s, _set.frequency, _set.ooratio, _set.lrphase,_set.brght_red,_set.brght_green,_set.brght_blue);

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

int updateLEDs()
{
  int val = callback_lst_int_t[_set.waveform]();
  setPWM(val);
  FastLED.show();
  return val;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.setTimeout(25);

  Serial.println("F*100,Waveform,On/OffRatio*100,LRPhase(deg)*100,Red,Green,Blue");
  Serial.println("Waveforms: 0 Sin, 1 Rect, 2 Triangle, 3 Saw");
  Serial.println("Eg1: 100,0,50,0,50,50,50");
  Serial.println("Eg2: 150,1,30,9000,55,55,55");
   
  
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

  _last_start = millis();
}

void loop()
{
  int val = 0;
  unsigned long time_ms = millis();

  if(time_ms-_last_time>=DT_MS) // update LED AND pwm
  {
    val = updateLEDs();
    //setPWM(val);
    _last_time = time_ms;
  }

  if( Serial.available() > 0 ) readSetSerial();
  
}
