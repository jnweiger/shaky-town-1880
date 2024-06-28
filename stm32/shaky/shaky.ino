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
// I2C1_SDA=PB7  & I2C1_SCL=PB6 (remap I2C1_SDA=PB9 & I2C1_SCL=PB8)
//TwoWire WIRE1(1, I2C_FAST_MODE);  // PB7, PB6);
TwoWire WIRE1(2, I2C_FAST_MODE);  // PB11, PB10);

myVL53L0X sensor_vl53l0x(&WIRE1, PB1); //PB12, XSHUT=PC6

uint8_t color_fan[3*3*3*3];	// for fillStrip_3x25()

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

  // geenish-blue with a hint of purple
  CFAN(0,0,0, 100,255,100); CFAN(0,0,1, 100,100,255); CFAN(0,0,2, 100,255,100);	// Turm0
  CFAN(1,0,0, 100,100,255); CFAN(1,0,1, 100,255,100); CFAN(1,0,2, 100,100,255);	// Turm1
  CFAN(2,0,0, 200,  0,200); CFAN(2,0,1, 200,100,  0); CFAN(2,0,2, 200,  0,200);	// Turm2

  // yellow red...
  CFAN(0,1,0, 200,200,  0); CFAN(0,1,1, 255,100,100); CFAN(0,1,2, 200,200,  0);	// Walch0
  CFAN(1,1,0, 255,100,100); CFAN(1,1,1, 200,200,  0); CFAN(1,1,2, 255,100,100);	// Walch1
  CFAN(2,1,0, 200,200,  0); CFAN(2,1,1, 235,150, 50); CFAN(2,1,2, 255,100,100);	// Walch2

  // blue, puple
  CFAN(0,2,0, 100,100,255); CFAN(0,2,1, 255,100,255); CFAN(0,2,2, 100,100,255);	// Walch0
  CFAN(1,2,0, 255,100,255); CFAN(1,2,1,   0,  0,255); CFAN(1,2,2, 255,100,255);	// Walch1
  CFAN(2,2,0, 255,100,255); CFAN(2,2,1, 127, 50,255); CFAN(2,2,2, 100,100,255);	// Walch2

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
  }
}

// Function to fill the strip with a single color
void fillStrip(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

/* Loop ----------------------------------------------------------------------*/

int blink = 0;
uint16_t idle_tick = 0;	// state machine.

interpol9col(uint8_t idx1, uint8_t idx2, uint8_t perc, uint32_t *colorp) {
  uint8_t *f1 = &color_fan[9*3*idx1];
  uint8_t *f2 = &color_fan[9*3*idx2];

  uint8_t r, g, b;
  for (int i = 0; i < 9; i++) {
    r = (int)(0.01 * (100-perc) * f1[3*i+0] + perc * f2[3*i+0]);
    g = (int)(0.01 * (100-perc) * f1[3*i+1] + perc * f2[3*i+1]);
    b = (int)(0.01 * (100-perc) * f1[3*i+2] + perc * f2[3*i+2]);
    colorp[i] = strip.Color(r, g, b);
  }
}

#define MOTOR_IDLE_SCALE 30
#define LED_IDLE_SCALE_SHIFT 4
#define LED_IDLE_SCALE (1<<(LED_IDLE_SCALE_SHIFT))

uint8_t idle() {
  uint8_t m = 0;

  uint32_t colors[9];

  uint32_t led_step = idle_tick >> LED_IDLE_SCALE_SHIFT;
  if      (led_step < 100) interpol9col(0, 1, led_step - 0*100, &color);
  else if (led_step < 200) interpol9col(1, 2, led_step - 1*100, &color);
  else if (led_step < 300) interpol9col(2, 0, led_step - 2*100, &color);
  else
     idle_tick = 0;

  // idle_tick counts to 300 * 16 = 4800;
  // motor pulse goes to 50 * 30 = 1500 -> so ca 1/3 the time its moving.

  // slow motor pulse
  if      (idle_tick < 10 * MOTOR_IDLE_SCALE) m = 60;
  else if (idle_tick < 20 * MOTOR_IDLE_SCALE) m = 90;
  else if (idle_tick < 30 * MOTOR_IDLE_SCALE) m = 120;
  else if (idle_tick < 40 * MOTOR_IDLE_SCALE) m = 90;
  else if (idle_tick < 50 * MOTOR_IDLE_SCALE) m = 60;
  return m;
}



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

  digitalWrite(LED_PIN, (distance < 400) ? HIGH : LOW);

  if (status == VL53L0X_ERROR_NONE)
  {
    // Output data.
    char report[64];
    snprintf(report, sizeof(report), "| Distance [mm]: %ld |", distance);
    Serial.println(report);
  }

  uint8_t motor_speed = 60:

  if (distance <  10 && distance > 0) { idle(); motor_speed = 0; }  // cover applied!
  else if (distance < 200) { fillStrip(strip.Color(255, 100, 100)); idle_tick = 0; motor_speed = 255); } // Red
  else if (distance < 300) { fillStrip(strip.Color(100, 255, 100)); idle_tick = 0; motor_speed = 100; }
  else if (distance < 450) { fillStrip(strip.Color(255, 255, 100)); idle_tick = 0; motor_speed = 84); }
  else if (distance < 600) { fillStrip(strip.Color(255, 255,   0)); idle_tick = 0; motor_speed = 64; } // Yellow
  else                     { motor_speed = idle(); }

  analogWrite(MOTOR_PIN, motor_speed);
  digitalWrite(WS2812_PIN, (blink) ? HIGH : LOW);
  blink = 1 - blink;
  idle_tick++;
}
