/*
 *  shaky-town 1884
 *
 *  References:
 *  - https://github.com/stm32duino/VL53L0X/tree/main/examples
 *  - https://how2electronics.com/wp-content/uploads/2019/02/STM32-Pin-Details.jpg
 *  - https://github.com/vtx22/STM32-WS2812/blob/master/src/LED.cpp
 *
 *
 *  Requires:
 *   Tools -> Manage Libraries ... -> STM32duino VL53L0X
 *    -> install
 *   Tools -> Board:	"generic STM32F103C series"
 *   Tools -> Variant:	"STM32F103C8 (20k RAM, 64k Flash)"
 *   Tools -> Upload Method: "serial"
 *   Tools -> Speed:	"72 Mhz (Normal)"
 *   Tools -> Port:	"/dev/ttyUSB0"
 *
 * STM32F103C8 overview:
 * 3 × USARTs, 3 × 16-bit timers, 2 × SPIs, 2 × I2Cs, USB, CAN, 1 × PWM timer, 2 × ADCs
 *
 * All rights reserved. Distribute under MIT License or ask.
 *
 * V0.1 - 2024-06-24, jw - first light.
 * V0.2 - 2024-06-28, jw - idle loop nicer.
 *
 */
/* Includes ------------------------------------------------------------------*/
#include <Wire.h>
#include <vl53l0x_class.h>

#define MOTOR_PIN PB6 // PWM enabled pins are: PA0 to PA3; PA6 to PA10; PB0; PB1; PB6 to PB9


#define NUM_LEDS 75
#if 0

#include <Adafruit_NeoPixel.h>
#define WS2812_PIN PA8
// Create an instance of the Adafruit_NeoPixel class
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, WS2812_PIN, NEO_GRB + NEO_KHZ800);

#else

#include <WS2812B.h>  // stmduino library uses SPI1. Connect the WS2812B data input to SPI1 MOSI on your board. (PB5) -> PA7
#define WS2812_PIN PA7
WS2812B strip = WS2812B(NUM_LEDS);

#endif

#define SR04_TIMEOUT_MS 15  	// 15 ms timeout for 2m max distance
#define SR04_TRIG_PIN	PB8	// need to be 5V tolerant!
#define SR04_ECHO_PIN	PB7	// need to be 5V tolerant!
#define SOUND_SPEED	0.343   // mm/microsecond

void sr04_setup() {
  // Configure pins
  pinMode(SR04_TRIG_PIN, OUTPUT);
  pinMode(SR04_ECHO_PIN, INPUT);

  // Ensure trigger pin is low
  digitalWrite(SR04_TRIG_PIN, LOW);
}


uint32_t sr04_measure_distance_mm() {
  // Send trigger pulse
  digitalWrite(SR04_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SR04_TRIG_PIN, LOW);

  // Measure echo pulse duration
  unsigned long duration = pulseIn(SR04_ECHO_PIN, HIGH, SR04_TIMEOUT_MS * 1000);

  // Check for timeout
  if (duration == 0) {
    return 0;  // Indicate failure
  }

  // Calculate distance
  uint32_t distance = (int)((duration * SOUND_SPEED) / 2);

  return distance;
}


// device info is not exposed. Let's subclass ~/Arduino/libraries/STM32duino_VL53L0X/src/vl53l0x_class.h
class myVL53L0X : public VL53L0X {
  public:
    /** Constructor that matches the base class constructor
     * @param[in] i2c device I2C to be used for communication
     * @param[in] pin shutdown pin to be used as component GPIO0
     */
    myVL53L0X(TwoWire *i2c, int pin): VL53L0X(i2c, pin) {}

    // getter
    VL53L0X_DeviceInfo_t *deviceinfo() {
      return &DeviceInfo;
    }
};

// Create components.
// I2C2_SDA=PB11 & I2C2_SCL=PB10
// I2C1_SDA=PB7  & I2C1_SCL=PB6 (remap I2C1_SDA=PB9 & I2C1_SCL=PB8)	// PB8,PB9 taken by SR04 !
//TwoWire WIRE1(1, I2C_FAST_MODE);  // PB7, PB6);
TwoWire WIRE1(2, I2C_FAST_MODE);  // PB11, PB10);

myVL53L0X sensor_vl53l0x(&WIRE1, PB1); //PB12, XSHUT=PC6

uint8_t color_fan[4*3*3*3];	// for fillStrip_3x25()

/* Setup ---------------------------------------------------------------------*/
// the shipped blink example says the LED pin is PB1
#define LED_PIN PC13
void setup() {
  int status;
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // pinMode(WS2812_PIN, OUTPUT);

  // x is xpos in image
  // y is image
  // t is time
#define CFAN(t,y,x, r, g, b) color_fan[3*((t)*9+(y)*3+(x))+0] = r; color_fan[3*((t)*9+(y)*3+(x))+1] = g; color_fan[3*((t)*9+(y)*3+(x))+2] = b


#if 1
  // blue, puple
  CFAN(0,0,0,  25, 25,255); CFAN(0,0,1, 255,500,255); CFAN(0,0,2,  25, 25,255); // Kett0
  CFAN(1,0,0,  25, 25,255); CFAN(1,0,1, 255,500,255); CFAN(1,0,2,  25, 25,255); // Kett1
  CFAN(2,0,0, 255, 50,255); CFAN(2,0,1,   0,  0,255); CFAN(2,0,2, 255, 50,255); // Kett2
  CFAN(3,0,0, 255, 50,255); CFAN(3,0,1, 255, 50,255); CFAN(3,0,2,  25, 25,255); // Kett3
#endif

#if 1
  // yellow red...
  CFAN(0,1,0, 255,240, 20); CFAN(0,1,1, 255,  0,  0); CFAN(0,1,2, 255,  0,  0);	// Walch0
  CFAN(1,1,0, 255,  0,  0); CFAN(1,1,1, 255,240,  0); CFAN(1,1,2, 255,  0,  0);	// Walch1
  CFAN(2,1,0, 255,  0,  0); CFAN(3,1,1, 255,240,  0); CFAN(2,1,2, 255,  0,  0); // Walch2
  CFAN(3,1,0, 255,  0,  0); CFAN(3,1,1, 255,  0,  0); CFAN(3,1,2, 255,240,  0);	// Walch3
#endif

#if 1
  // geenish-blue with a hint of purple
  CFAN(0,2,0, 150,100,200); CFAN(0,2,1, 100,100,200); CFAN(0,2,2, 150,100,200); // Turm0
  CFAN(1,2,0, 100,100,255); CFAN(1,2,1, 100,255,100); CFAN(1,2,2, 100,100,255); // Turm1
  CFAN(2,2,0, 100,255,100); CFAN(2,2,1, 100,100,255); CFAN(2,2,2, 100,255,100); // Turm2
  CFAN(3,2,0, 100,255,100); CFAN(3,2,1, 100,100,255); CFAN(3,2,2, 100,255,100); // Turm3
#endif

  pinMode(MOTOR_PIN, OUTPUT);
  analogWrite(MOTOR_PIN, 100);    // range 0 .. 255 with analogWrite()

  // Initialize serial for output.
  Serial.begin(115200);

  // Initialize I2C bus.
  WIRE1.begin();

  // Configure VL53L0X component.
  sensor_vl53l0x.begin();
  // Switch off VL53L0X component.
  sensor_vl53l0x.VL53L0X_Off();

  // Initialize VL53L0X component.
  status = sensor_vl53l0x.InitSensor(0x10);
  if(status)
  {
    Serial.println("Init sensor_vl53l0x failed...");
  }
#if 1
  {
    // VL53L0X ES1 or later | Type: VL53L0X | Prod: VL53L0CAV0DH/1$5 | Type: 238 | Rev: 1, 1
    VL53L0X_DeviceInfo_t *di = sensor_vl53l0x.deviceinfo();
    char info[100];
    snprintf(info, sizeof(info), "%s | Type: %s | Prod: %s | Type: %d | Rev: %d, %d",
      di->Name, di->Type, di->ProductId,
      di->ProductType, di->ProductRevisionMajor, di->ProductRevisionMinor);
    Serial.println(info);
  }
#endif

  // init ultrasonic sensor
  sr04_setup();

  // Initialize the NeoPixel strip
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void fillStrip_3x25(uint32_t *colors) {
  // colors is an array of 9.
  // each three got to one image, 8,9,8 leds ieach.
  int led_idx = 0;
  int col_idx = 0;
  for (int img = 0; img < 3; img++) {
    for (int i = 0; i < 8; i++) strip.setPixelColor(led_idx++, colors[col_idx]);
    col_idx++;
    for (int i = 0; i < 9; i++) strip.setPixelColor(led_idx++, colors[col_idx]);
    col_idx++;
    for (int i = 0; i < 8; i++) strip.setPixelColor(led_idx++, colors[col_idx]);
    col_idx++;
  }
  strip.show();
}

// Function to fill the strip with a single color
void fillStripN(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
}

/* Loop ----------------------------------------------------------------------*/

int blink = 0;
uint16_t idle_tick = 0;	// state machine.
uint16_t busy_tick = 0;
uint16_t busy_counter = 0;  // incremented by busy(), reset by idle()
uint16_t force_idle = 0;  // set to FORCE_IDLE_COUNTDOWN when busy_counter reaches MAX_BUSY_COUNTER, decremented by idle();
#define MAX_BUSY_COUNTER 300    // 300: 30 sec
#define FORCE_IDLE_COUNTDOWN 800 // 600: 1min

void interpol9col(uint8_t idx1, uint8_t idx2, uint8_t perc, uint32_t *colorp) {
  uint8_t *f1 = &color_fan[9*3*idx1];
  uint8_t *f2 = &color_fan[9*3*idx2];


  uint8_t r, g, b;
  for (int i = 0; i < 9; i++) {
#if 1
    r = (int)( ((100-perc) * f1[3*i+0] + perc * f2[3*i+0]) * 0.01);
    g = (int)( ((100-perc) * f1[3*i+1] + perc * f2[3*i+1]) * 0.01);
    b = (int)( ((100-perc) * f1[3*i+2] + perc * f2[3*i+2]) * 0.01);
#else
    // not interpolating, just idx1
    r = f2[3*i+0];
    g = f2[3*i+1];
    b = f2[3*i+2];
#endif
    colorp[i] = strip.Color(r, g, b);
  }
}

#define LEDP 30
#define MOTP 3

uint8_t idle() {
  uint8_t m = 0;

  uint32_t colors[9];

  busy_tick = 0;
  busy_counter = 0;
  if (force_idle > 0) force_idle--;

  if (idle_tick >= 4*LEDP) idle_tick = 0;

  uint32_t led_step = idle_tick;
  if      (led_step < 1*LEDP) interpol9col(0, 1, (led_step - 0*LEDP)*100/LEDP, &colors[0]);
  else if (led_step < 2*LEDP) interpol9col(1, 2, (led_step - 1*LEDP)*100/LEDP, &colors[0]);
  else if (led_step < 3*LEDP) interpol9col(2, 3, (led_step - 2*LEDP)*100/LEDP, &colors[0]);
  else if (led_step < 4*LEDP) interpol9col(3, 0, (led_step - 3*LEDP)*100/LEDP, &colors[0]);

  fillStrip_3x25(colors);
  // idle_tick counts to 30 * 4 = 120;
  // motor pulse goes to 7 * 3 = 21 -> so ca 1/6 the time its moving.

  // slow motor pulse
  if      (idle_tick < 1*MOTP) m = 60;
  else if (idle_tick < 2*MOTP) m = 90;
  else if (idle_tick < 4*MOTP) m = 120; // twice as long
  else if (idle_tick < 5*MOTP) m = 90;
  else if (idle_tick < 6*MOTP) m = 60;
  else if (idle_tick < 7*MOTP) m = 40;

#if 0
  char report[64];
  snprintf(report, sizeof(report), "| idle: %ld | %ld | %d | p=%d", idle_tick, led_step, m, (led_step - 0*LEDP)*100/LEDP);
  Serial.println(report);
#endif

  return m;
}


void blinker(uint16_t off, uint8_t d, uint8_t idx, uint8_t width) {
  uint32_t c = strip.Color(255,0,0);
  if (busy_tick >= off && busy_tick < off+d) {
    for (uint8_t i = 0; i < width; i++) {
      if (idx+i < 75) strip.setPixelColor(idx+i, c);
    }
  }
}

void blinker4(uint16_t off, uint8_t d, uint8_t idx, uint8_t w) {
  blinker(off+0*d, d, idx+w, w);
  blinker(off+1*d, d, idx,   w);
  blinker(off+2*d, d, idx+w, w);
  blinker(off+3*d, d, idx,   w);
}

uint8_t busy(uint32_t d) {
  uint8_t m = 80;

  idle_tick = 0;
  busy_counter++;

  if      (d < 200) { fillStripN(strip.Color(255,   0,   0)); m = 255; } // Red
  else if (d < 300) { fillStripN(strip.Color(255, 145,   0)); m = 200; } // orange
  else if (d < 400) { fillStripN(strip.Color(255, 245,   0)); m = 150; } // yellow
  else              { fillStripN(strip.Color(255, 255, 255)); m = 100; } // white

  blinker4(10, 5,  2, 5);
  blinker4(30, 4,  2, 5);
  blinker4(46, 5,  2, 5);

  blinker4( 8, 3, 22, 3);
  blinker4(20, 3, 22, 3);
  blinker4(32, 3, 22, 3);

  blinker4(20, 2, 42, 3);
  blinker4(28, 2, 42, 3);
  blinker4(46, 2, 42, 3);
  blinker4(54, 2, 42, 3);

  blinker4( 0, 4, 62, 4);
  blinker4(16, 4, 62, 4);
  blinker4(20, 3, 50, 3);
  blinker4(32, 3, 50, 3);
  blinker4(44, 3, 50, 3);

  if (busy_tick > 66) busy_tick = 0;

  strip.show();

  return m;
}

uint32_t slow_dist = 9999;

void loop() {
  // Led blinking.
  // digitalWrite(LED_PIN, LOW);  delay(10);
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(100);
  // digitalWrite(LED_BUILTIN, LOW);

  // Read Range.
  uint32_t distance;
  int status;
  status = sensor_vl53l0x.GetDistance(&distance);

  uint32_t dist_sonic = sr04_measure_distance_mm();

  digitalWrite(LED_PIN, (distance < 400) ? HIGH : LOW);

  if (status == VL53L0X_ERROR_NONE)
  {
    // Output data.
    char report[64];
    snprintf(report, sizeof(report), "| Distance [mm]: %ld | sr04: %ld | f=%d | b=%d",
      distance, dist_sonic, force_idle, busy_counter);
    Serial.println(report);
  }

  uint8_t motor_speed = 60;
  if (busy_counter > MAX_BUSY_COUNTER) force_idle = FORCE_IDLE_COUNTDOWN;
  if (force_idle) distance = 0;

  if (slow_dist == 0)       slow_dist = distance; // jump out of zero, without going throgh short distances.
  if (slow_dist < distance) slow_dist += 10;
  if (slow_dist > distance) slow_dist = distance; // no else to avoid overshoot

  if (slow_dist == 0)       { motor_speed = idle(); }  		// 0 is infinite;
  else if (slow_dist <  50) { idle(); motor_speed = 0; }  	// no motor, when cover is very close.
  else if (slow_dist <  80) { motor_speed = idle(); }  		// cover applied!
  else if (slow_dist < 600) { motor_speed = busy(slow_dist); } // strong colors and fast motor
  else                      { motor_speed = idle(); }
  analogWrite(MOTOR_PIN, motor_speed);
  digitalWrite(WS2812_PIN, (blink) ? HIGH : LOW);
  blink = 1 - blink;
  idle_tick++;
  busy_tick++;
}
