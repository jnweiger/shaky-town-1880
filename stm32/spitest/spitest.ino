#include <WS2812B.h>

/* 
 *  spi-test
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
#define LED_PIN PC13
#define NUM_LEDS 25

#if 0

#include <Adafruit_NeoPixel.h>
#define WS2812_PIN PA7
// Create an instance of the Adafruit_NeoPixel class
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, WS2812_PIN, NEO_GRB + NEO_KHZ800);

#endif

#if 1

#include <WS2812B.h>  // stmduino library uses SPI1. Connect the WS2812B data input to SPI1 MOSI on your board. (PB5) -> PA7
#define WS2812_PIN PA7
WS2812B strip = WS2812B(NUM_LEDS);

#endif

const int chipSelectPin = PB6;

void setup() {
//  pinMode(WS2812_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

//   Serial.begin(115200);
//  Serial.println("hello ");


  // Configure SPI settings
  pinMode(chipSelectPin, OUTPUT);  // Set the Chip Select pin as an output
  digitalWrite(chipSelectPin, HIGH); // Set CS high (inactive)

  strip.begin();
  strip.show();
}

/* Loop ----------------------------------------------------------------------*/

int blink = 0;

void loop() {
  uint16_t i;
  
  digitalWrite(chipSelectPin, (blink) ? HIGH : LOW);
  if (blink)
  {
     for(i = 0; i < strip.numPixels(); i++)
        strip.setPixelColor(i, strip.Color(100, 0, 0));
     strip.show();
  }
  else
  {
     for(i = 0; i < strip.numPixels(); i++)
        strip.setPixelColor(i, strip.Color(0, 100, 0));
     strip.show();
  }
  
  digitalWrite(LED_PIN, (blink) ? HIGH : LOW);
  blink = 1 - blink;
  delay(200);
}
