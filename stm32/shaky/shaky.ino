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

/* Setup ---------------------------------------------------------------------*/
// the shipped blink example says the LED pin is PB1
#define LED_PIN PC13
void setup() {
  int status;
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // pinMode(WS2812_PIN, OUTPUT);

  pinMode(MOTOR_PIN, OUTPUT);
  analogWrite(PB6, 100);    // range 0 .. 255 with analogWrite()

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

// Function to fill the strip with a single color
void fillStrip(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

/* Loop ----------------------------------------------------------------------*/

int blink = 0;

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

  if      (distance <  10) { fillStrip(strip.Color(0,     0,   0)); analogWrite(PB6,   0); }
  else if (distance < 100) { fillStrip(strip.Color(0,     0, 255)); analogWrite(PB6, 255); } // Blue
  else if (distance < 200) { fillStrip(strip.Color(0,   255, 255)); analogWrite(PB6, 128); }
  else if (distance < 300) { fillStrip(strip.Color(255, 255, 255)); analogWrite(PB6,  64); }
  else if (distance < 400) { fillStrip(strip.Color(255, 255,   0)); analogWrite(PB6,  32); } // Red
  else                     { fillStrip(strip.Color(  0,   0,   0)); analogWrite(PB6,   0); }

  digitalWrite(WS2812_PIN, (blink) ? HIGH : LOW);
  blink = 1 - blink;
  
}
