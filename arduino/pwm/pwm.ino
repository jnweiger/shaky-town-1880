/*
 * (C) 2024, juergen@fabmail.org, All right reserved, Distribute under GPLv2 or ask. 
 * 
 * 2024-06-10, v0.2, jw - initial draft
 * 
 * This code is for an Arduino Nano board.
 *   Tools -> Board:    "Arduino Nano"
 *   Tools -> Processor "ATmega328p (old Bootloader)"  -> Overriding Baud Rate: 57600
 *   Tools -> Port:     "/dev/ttyUSB0"
 *
 * 
 * D9   - IRLZ43N Gate Pin 1 (left)
 * GND  - IRLZ43N Source Pin 3 (right)
 * VIN  - +5V Power supply  ------------- Motor+
 *        IRLZ43N Drain Pin 2 (center) -- Motor-
 * GND  - VL53LXX-V2  GND            
 * +5V  - VL53LXX-V2  VIN
 * A5   - VL53LXX-V2  SCL
 * A4   - VL53LXX-V2  SDA
 * D6   - WS2812B data_in
 *
 * Arduino-IDE -> Sketch -> Include Library -> Manage Libraries -> Adafruit VL53L0X
 * https://polluxlabs.net/arduino-tutorials/entfernungen-messen-mit-dem-vl53l0x-sensor-und-dem-arduino-uno/
 */

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>



// Create an object of the VL53L0X sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#define LONG_RANGE
// #define HIGH_SPEED
// #define HIGH_ACCURACY

// Constants
const int pwmPin = 9; // Pin 9 is connected to Timer1
const int ledPin = 13;  // Pin 13 has an onboard LED on most Arduino boards

#define WS2812_PIN 6

#define N_FLAMES 4              // number of flames
#define N_ADV_PER_STEP 3        // number of advancements per loop
#define NUM_PIXELS (3*25)   // how many LEDs we have
#define B(x) (x>>2)         // absolute brightness downscaler
#define DECAY_LIN_STEP 3
#define DECAY_LIN(x) (((x) > 10) ? ((x) - DECAY_LIN_STEP) : (x))
#define DECAY_PER_STEP 0.99     // decacy factor per advancement

#define MIN_PWM   55          // motor starts safely with this pwm.
#define MAX_PWM   255         // ICR1 keep in sync with setup()
#define IDLE_PWM  99          // motor speed with no sensor interaction

#define DIST_OUT_OF_RANGE 5000  // sensor cannot measure that far. This is the no measurement value.
#define MOTOR_SPEED 1.0       // range diff mm to pwm speed


Adafruit_NeoPixel pixels(NUM_PIXELS, WS2812_PIN, NEO_GRB + NEO_KHZ800);

typedef struct pingpong {
  // the value oszillates between min and max. The movement direction is vec = dir * spe, where dir is either +1 or -1.
  // we split the sign away, so that we can keep the direction even after speed was temporarily 0.
  int16_t val, min, max, spe, dir;
} pp;

struct flame {
  pp bright, bspeed, xpos, xspeed;
};

struct flame f[N_FLAMES];

struct rgb {
  uint8_t r, g, b;
};

char led_temperature = 0;   // cold: 0  .. warm: N_FLAMES


struct rgb fcolor[N_FLAMES*2] =
{
  // cool
  { B(133), B(255), B(155) },    // bright green
  { B(188), B(155), B(255) },    // bright blue
  { B(255), B(255), B(255) },    // white
  { B(155), B(225), B(255) },    // bright cyan
  // warm
  { B(255), B(255),  B(77) },    // bright yellow
  { B(255), B(211),  B(55) },    // bright peaches
  { B(255),  B(99),  B(55) },    // bright tomato
  { B(222), B(255), B(111) }     // bright lemon
};


#ifndef DECAY_LIN
unsigned char decay[256];
#endif

void setup_flames()
{
  //                 val, min, max, spe, dir
  f[0].bright = (pp){ 150, 127, 255, 7, +1 };
  f[1].bright = (pp){ 200, 127, 255, 9, -1 };
  f[2].bright = (pp){ 188, 127, 255, 3, +1 };
  f[3].bright = (pp){ 222, 127, 255, 1, +1 };

  f[0].bspeed = (pp){ 7*8, 5*8, 20*8,  7, -1 };
  f[1].bspeed = (pp){ 9*8, 7*8, 11*8, 10, +1 };
  f[2].bspeed = (pp){ 3*8, 0*8, 10*8,  3, +1 };
  f[3].bspeed = (pp){ 1*8, 0*8, 30*8, 22, +1 };

  f[0].xpos = (pp){ 222, 0, 16*3*25,  8, +1 };
  f[1].xpos = (pp){  50, 0, 16*3*25,  2, -1 };
  f[2].xpos = (pp){ 111, 0, 16*3*25, 15, +1 };
  f[3].xpos = (pp){ 278, 0, 16*3*25,  5, -1 };

  f[0].xspeed = (pp){ 8*8, 0*8, 13*8,  7, -1 };
  f[1].xspeed = (pp){ 2*8, 7*8, 23*8, 13, +1 };
  f[2].xspeed = (pp){15*8, 5*8, 17*8,  3, +1 };
  f[3].xspeed = (pp){ 5*8, 0*8,  7*8,  2, +1 };
#if N_FLAMES > 4
#error "main initializes only 4 flames"
#endif

#ifndef DECAY_LIN
  for (int i = 0; i < 256; i++)
     decay[i] = int(DECAC_PER_STEP * i);
//     decay[i] = (i > 10) ? i - DECAY_LIN_STEP : i;
#endif
}

void pp_advance(struct pingpong *p)
{
  p->val += p->dir * p->spe;
  if (p->val > p->max)
    {
      p->dir = -p->dir;
      // reflect back, go the same distance under max, that we were above max.
      p->val -= 2* (p->val - p->max);
    }
  if (p->val < p->min)
    {
      p->dir = -p->dir;
      // reflect back, go the same distance over min, that we were under min.
      p->val += 2* (p->min - p->val);
    }
}

// pp_set_speed only sets the magnitude of vec, but does not change the direction.
// Make sure vec fits within the limits of min and max,
// this is good to check here, as pp_advance() assumes, that one advancement can never hit both, min and max.
void pp_set_speed(struct pingpong *p, int vec)
{
  // vec = abs(vec);
  int vmax = p->max - p->min;
  if (vec > vmax) vec = vmax;
  if (vec < -vmax) vec = -vmax;
  p->spe = vec;
}

void flame_advance(struct flame *f)
{
  // nodulate a flame. its brightness changes
  pp_advance(&f->bright);
  pp_advance(&f->xpos);
  pp_advance(&f->xspeed);
 // pp_set_speed(&f->xpos,   f->xspeed.val >> 3); // divide by 8
 // pp_set_speed(&f->bright, f->bspeed.val >> 3); // divide by 8
}


void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  Serial.println(F("Hello"));

  // Initialize I2C communication
  if (!lox.begin()) {
    Serial.println(F("VL53L0X fail, try POWERCYLE"));
      while (1);
  }
  Serial.println(F("VL53L0X booted"));

#ifdef LONG_RANGE
  // ~/Arduino/libraries/Adafruit_VL53L0X/src/Adafruit_VL53L0X.cpp:232
  lox.configSensor(lox.VL53L0X_SENSE_LONG_RANGE);
#endif

  pinMode(pwmPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // Configure Timer1
  TCCR1A = 0; // Clear Timer/Counter Control Registers
  TCCR1B = 0;
  
  // Set to Fast PWM mode with ICR1 as top
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);
  
  // TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10); // prescaler 1024
  // TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);    // prescaler 256
  // TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);    // prescaler 64
  // TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);    // prescaler 8
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);    // prescaler 1

  // Set ICR1 7800 Hz (assuming 16 MHz clock amd prescaler 8)
  // so that ICR1 receives a nice 255.
  // F_CPU / (Prescaler * Target frequency) - 1
  ICR1 = 255; // (16,000,000 / ( 8 * 7810 ) ) - 1

  // Initialize Timer1 Compare Output Mode to clear OC1A on compare match
  TCCR1A |= (1 << COM1A1);
  
  // Start with 0% duty cycle
  OCR1A = 0;

  pixels.begin();
  pixels.clear();

  setup_flames();

  // make some light to start with...
  for (int n = 0; n < 25; n++)
  {
    pixels.setPixelColor(n,    fcolor[0].r>>2, fcolor[0].g>>2, fcolor[0].b>>2);
    pixels.setPixelColor(n+25, fcolor[1].r>>2, fcolor[1].g>>2, fcolor[1].b>>2);
    pixels.setPixelColor(n+50, fcolor[2].r>>2, fcolor[2].g>>2, fcolor[2].b>>2);
  }
  pixels.show();
}

void decay_flames()
{
  uint32_t color;
  for (int i = 0; i < NUM_PIXELS; i++)
    {
      color = pixels.getPixelColor(i);
      unsigned char r = (color >> 16) & 0xff;
      unsigned char g = (color >> 8) & 0xff;
      unsigned char b = (color >> 0) & 0xff;
#ifdef DECAY_LIN
      pixels.setPixelColor(i, DECAY_LIN(r), DECAY_LIN(g), DECAY_LIN(b));
#else
      pixels.setPixelColor(i, decay[r], decay[g], decay[b]);
#endif
    }
//  Serial.println(color);  
}

int16_t x[N_FLAMES] = { 10<<3, 30<<3, 50<<3, 70<<3 };
int8_t d[N_FLAMES] = { 1, -3, 5, -7 };

void advance_flames()
{
  for (uint8_t j = 0; j < N_ADV_PER_STEP; j++)
    {
      for (int i = 0; i < N_FLAMES; i++)
        {
//          int x = f[i].xpos.val >> 4;

//          uint8_t r = ((uint16_t)f[i].bright.val * fcolor[i+led_temperature].r)>>8;
//          uint8_t g = ((uint16_t)f[i].bright.val * fcolor[i+led_temperature].g)>>8;
//          uint8_t b = ((uint16_t)f[i].bright.val * fcolor[i+led_temperature].b)>>8;
  
          uint8_t r = fcolor[i+led_temperature].r;
          uint8_t g = fcolor[i+led_temperature].g;
          uint8_t b = fcolor[i+led_temperature].b;
          pixels.setPixelColor(x[i]>>3, pixels.Color(r, g, b));

          int16_t xi = x[i] + d[i];
          if (xi >= (NUM_PIXELS<<3)) x[i] = 0;
          else if (xi < 0) x[i] = (NUM_PIXELS<<3)-1;
          else x[i] = xi;
          
//          flame_advance(f+i);
        }
    }
  // Serial.println(f[0].bright.val);  
}

#if 0
int interpolate_pwm(double perc)
{
  //      U/sec   RPM at 5V
  // 255: 3 sec   20
  // 192: 4 sec   15
  // 159: 5 sec   12
  // 136: 6 sec   10
  // 110: 8 sec   7,5
  // 94: 10 sec   6
  // 85: 12 sec   5
  // 73: 16 sec   3,75
  // 65: 20 sec   3
  // 61: 24 sec   2,5
  // 55: 30 sec   2   (slowest possible)

  if (perc < 10) return 0;
  double r = perc * 0.2; // reduce range from 0..100 to 0..20
  if (r > 12)
    return int((r-12)*(255-157)/(20-12)+157.5);
  return int((r-2)*(157-55)/(12-2)+55.5);
}
#endif

// int perc = 0;
int8_t dirstep = 1;
uint8_t pwm = 55;

void dist_to_led_temp(uint16_t dist) {
  if (dist < 300)
    led_temperature = 4;
  else if (dist < 400)
    led_temperature = 3;
  else if (dist < 600)
    led_temperature = 2;
  else if (dist < 800)
    led_temperature = 1;
  else
    led_temperature = 0; 
}

uint16_t last_dist = 0;
void dist_to_motor_pwm(uint16_t dist) {
  Serial.println(dist);
  Serial.println(last_dist);
  if (dist == DIST_OUT_OF_RANGE) {
      pwm = IDLE_PWM;
      last_dist = 0;
      return;
    }
  if (last_dist != 0) {
    int16_t diff = (int16_t)(0.5 + abs((int16_t)last_dist - dist) * MOTOR_SPEED);
    Serial.println(diff);
    diff -= 30;
    if (diff < 0) {
      pwm = 0;
      return;     // don't update last_dist, so that things accumulate for next round.
    }
    if (diff <= MAX_PWM)
      pwm = diff;
    else
      pwm = MAX_PWM;
  }
  last_dist = dist;
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);   // Pass 'true' to get debug data

  if (measure.RangeStatus != 4) {     // VL53L0X_RANGESTATUS_NONE
    Serial.print(F("D (mm): "));
    Serial.println(measure.RangeMilliMeter);
    dist_to_led_temp(measure.RangeMilliMeter);
    dist_to_motor_pwm(measure.RangeMilliMeter);
    digitalWrite(ledPin, (measure.RangeMilliMeter > 150) ? LOW : HIGH);     // Turn on/off the LED on pin 13
  } else {
    Serial.println(F("Out of Range"));
    dist_to_led_temp(DIST_OUT_OF_RANGE);
    dist_to_motor_pwm(DIST_OUT_OF_RANGE);
    digitalWrite(ledPin, HIGH);     // Turn on the LED on pin 13

  }

#if 0
  int pwm = interpolate_pwm(perc);
  perc += dirstep;
  if (perc > 100)
  {
    perc = 100;
    dirstep = -dirstep;
  }
  if (perc < 0)
  {
    perc = 0;
    dirstep = -dirstep;
  }
#endif
#if 0
  pwm += dirstep;
  if (pwm > 254)
    {
      dirstep = -1;
    }
  else if (pwm < 56)
    {
      dirstep = 1;
    }
#endif
//  digitalWrite(ledPin, (pwm < 200) ? LOW : HIGH);     // Turn on/off the LED on pin 13
  OCR1A = pwm;                      // set duty cycle

  decay_flames();
  advance_flames();
  pixels.show();

  delay(200);
}
