/* NOTE: This sketch required at least version 1.1.0 of Adafruit_Neopixel !!! */

#include <string.h>

#include <Arduino.h>

#include <SPI.h>

#include "Adafruit_BLE.h"

#include "Adafruit_BluefruitLE_SPI.h"

#include "Adafruit_BluefruitLE_UART.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#include "BluefruitConfig.h"

#define NEOPIXEL_VERSION_STRING "Neopixel v2.0"
#define PIN 6 /* Pin used to drive the NeoPixels */

#define MAXCOMPONENTS 4
uint8_t * pixelBuffer = NULL;
uint8_t width = 0;
uint8_t height = 0;
uint8_t stride;
uint8_t componentsValue;
bool is400Hz;
uint8_t components = 3; // only 3 and 4 are valid values
#define NUM_LEDS 12
#define DATA_PIN 6
uint8_t brightness = 255; /* Control the brightness of your leds */
#define SATURATION 255 /* Control the saturation of your leds */

#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
unsigned long patternInterval = 20; // time between steps in the pattern
unsigned long lastUpdate = 0; // for millis() when last update occoured
unsigned long intervals[] = {
  20,
  20,
  50,
  100
}; // speed for each pattern

// Create the bluefruit object, either software serial...uncomment these lines

// A small helper
void error(const __FlashStringHelper * err) {
  Serial.println(err);
  while (1);
}

void serial_printf(const char * format, ...) {
  char buffer[48];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.print(buffer);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE * ble, uint16_t timeout);
float parsefloat(uint8_t * buffer);
void printHex(const uint8_t * data,
  const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void) {
  Serial.begin(115200);

  // Config Neopixels
  //neopixel.begin();  FastLED.addLeds < NEOPIXEL, DATA_PIN > (leds, NUM_LEDS);
  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  Serial.println(F("OK!"));

  ble.println("AT+GAPDEVNAME=DeviceName");

  /* Disable command echo from Bluefruit */
  ble.echo(false);
  ble.info();
  ble.verbose(true); // debug info is a little annoying after this point!

  /* Wait for connection */
  while (!ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("***********************"));

  // Set Bluefruit to DATA mode
  Serial.println(F("Switching to DATA mode!"));
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("***********************"));

  strip.begin(); // This initializes the NeoPixel library.
  wipe(); // wipes the LED buffers
}

uint8_t red = 0;
uint8_t green = 0;
uint8_t blue = 0;
/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void) {
  static int pattern = 1;
  if (millis() - lastUpdate > patternInterval) updatePattern(pattern);
  /* Wait for new data to arrive */
  uint8_t len = readPacket( & ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  //printHex(packetbuffer, len);

  // Color
  if (packetbuffer[1] == 'C') {
    red = packetbuffer[2];
    green = packetbuffer[3];
    blue = packetbuffer[4];

    Serial.print("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);
  }

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print("Button ");
    Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
      if (buttnum == 1) {
        pattern = 1; // change pattern number
      } else if (buttnum == 2) {
        pattern = 2;
      } else if (buttnum == 3) {
        pattern = 3;
      } else if (buttnum == 4) {
        pattern = 1;
      } else if (buttnum == 5) {
        pattern = 0;
      }
      patternInterval = intervals[pattern]; // set speed for this pattern
      wipe(); // clear out the buffer 

    } else {
      Serial.println(" released");
    }
  }

  // Brightness
  if (packetbuffer[1] == 'S') {
    brightness = packetbuffer[2];
    Serial.print("Brightness ");
    Serial.print(brightness);
    Serial.println("");
  }

  // GPS Location
  if (packetbuffer[1] == 'L') {
    float lat, lon, alt;
    lat = parsefloat(packetbuffer + 2);
    lon = parsefloat(packetbuffer + 6);
    alt = parsefloat(packetbuffer + 10);
    Serial.print("GPS Location\t");
    Serial.print("Lat: ");
    Serial.print(lat, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print("Lon: ");
    Serial.print(lon, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print(alt, 4);
    Serial.println(" meters");
  }

  // Accelerometer
  if (packetbuffer[1] == 'A') {
    float x, y, z;
    x = parsefloat(packetbuffer + 2);
    y = parsefloat(packetbuffer + 6);
    z = parsefloat(packetbuffer + 10);
    Serial.print("Accel\t");
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(z);
    Serial.println();
  }

  // Magnetometer
  if (packetbuffer[1] == 'M') {
    float x, y, z;
    x = parsefloat(packetbuffer + 2);
    y = parsefloat(packetbuffer + 6);
    z = parsefloat(packetbuffer + 10);
    Serial.print("Mag\t");
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(z);
    Serial.println();
  }

  // Gyroscope
  if (packetbuffer[1] == 'G') {
    float x, y, z;
    x = parsefloat(packetbuffer + 2);
    y = parsefloat(packetbuffer + 6);
    z = parsefloat(packetbuffer + 10);
    Serial.print("Gyro\t");
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(z);
    Serial.println();
  }

  // Quaternions
  if (packetbuffer[1] == 'Q') {
    float x, y, z, w;
    x = parsefloat(packetbuffer + 2);
    y = parsefloat(packetbuffer + 6);
    z = parsefloat(packetbuffer + 10);
    w = parsefloat(packetbuffer + 14);
    Serial.print("Quat\t");
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(z);
    Serial.print('\t');
    Serial.print(w);
    Serial.println();
  }
}

void updatePattern(int pat) { // call the pattern currently being created
  switch (pat) {
  case 0:
    rainbow();
    break;
  case 1:
    rainbowCycle();
    break;
  case 2:
    theaterChaseRainbow();
    break;
  case 3:
    colorWipe(strip.Color(red, green, blue)); // red
    break;
  }
}

void rainbow() { // modified from Adafruit example to make it a state machine
  static uint16_t j = 0;
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setBrightness(brightness);
    strip.setPixelColor(i, Wheel((i + j) & 255));
  }
  strip.show();
  j++;
  if (j >= 256) j = 0;
  lastUpdate = millis(); // time for next change to the display

}
void rainbowCycle() { // modified from Adafruit example to make it a state machine
  static uint16_t j = 0;
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setBrightness(brightness);
    strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
  }
  strip.show();
  j++;
  if (j >= 256 * 5) j = 0;
  lastUpdate = millis(); // time for next change to the display
}

void theaterChaseRainbow() { // modified from Adafruit example to make it a state machine
  static int j = 0, q = 0;
  static boolean on = true;
  if (on) {
    for (int i = 0; i < strip.numPixels(); i = i + 3) {
      strip.setBrightness(brightness);
      strip.setPixelColor(i + q, Wheel((i + j) % 255)); //turn every third pixel on
    }
  } else {
    for (int i = 0; i < strip.numPixels(); i = i + 3) {
      strip.setPixelColor(i + q, 0); //turn every third pixel off
    }
  }
  on = !on; // toggel pixelse on or off for next time
  strip.show(); // display
  q++; // update the q variable
  if (q >= 3) { // if it overflows reset it and update the J variable
    q = 0;
    j++;
    if (j >= 256) j = 0;
  }
  lastUpdate = millis(); // time for next change to the display    
}

void colorWipe(uint32_t c) { // modified from Adafruit example to make it a state machine
  static int i = 0;
  strip.setPixelColor(i, c);
  strip.show();
  i++;
  if (i >= strip.numPixels()) {
    i = 0;
    wipe(); // blank out strip
  }
  lastUpdate = millis(); // time for next change to the display
}

void wipe() { // clear all LEDs
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
