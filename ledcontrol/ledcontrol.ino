/* NOTE: This sketch required at least version 1.1.0 of Adafruit_Neopixel !!! */

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

#define NUM_LEDS 12
#define DATA_PIN 6
uint8_t brightness = 255; /* Control the brightness of your leds */

#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
unsigned long patternInterval = 20; // time between steps in the pattern
unsigned long lastUpdate = 0; // for millis() when last update occoured
unsigned long intervals[] = {20,20,50,100}; // speed for each pattern

// Create the bluefruit object, either software serial...uncomment these lines


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void) {
  while (!Serial);  // required for Flora & Micro
  delay(500);
  Serial.begin(115200);

  // Config Neopixels
  //neopixel.begin();  FastLED.addLeds < NEOPIXEL, DATA_PIN > (leds, NUM_LEDS);

  strip.begin(); // This initializes the NeoPixel library.
  wipe(); // wipes the LED buffers
  
  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  Serial.println(F("OK!"));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
  ble.println("AT+GAPDEVNAME=DeviceName");
  ble.verbose(false);  // debug info is a little annoying after this point!

  
  /* Wait for connection */
  //while (!ble.isConnected()) {
  //  delay(500);
  //}

  Serial.println(F("***********************"));

  // Set Bluefruit to DATA mode
  Serial.println(F("Switching to DATA mode!"));
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("***********************"));


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
  static int pattern = 18;
  
  if (Serial.available())
  {
    pattern = Serial.parseInt(); //reads serial input
    Serial.print(pattern);
    return;
  }
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
      pattern = buttnum;
      //patternInterval = intervals[pattern]; // set speed for this pattern
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

    // dBA
  if (packetbuffer[1] == 'D') {
    uint8_t dba = packetbuffer[2];
    Serial.print("dBA ");
    Serial.print(dba);
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
    rainbowFlicker();
    break;
  case 4:
    colorWipe(strip.Color(red, green, blue)); // red
    break;
  case 5:
    colorFlicker(strip.Color(red, green, blue)); // red
    break;
  case 6:
    solid(strip.Color(red, green, blue)); // red
    break;
  case 18:
    wipe();
    break;
  }
}

void wipe() { // clear all LEDs
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
  strip.show();
}

void solid(uint32_t c) { // clear all LEDs
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setBrightness(brightness);
    strip.setPixelColor(i, c);
  }
  strip.show();
}

void rainbow() { // modified from Adafruit example to make it a state machine
  static uint16_t j = 0;
  patternInterval = 20;
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
  patternInterval = 20;
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setBrightness(brightness);
    strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
  }
  strip.show();
  j++;
  if (j >= 256 * 5) j = 0;
  lastUpdate = millis(); // time for next change to the display
}

void rainbowFlicker() { // modified from Adafruit example to make it a state machine
  static uint16_t j = 0;
  patternInterval = 100;
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setBrightness(brightness);
    strip.setPixelColor(random(NUM_LEDS), Wheel(((i * 256 / strip.numPixels()) + j) & 255));
  }
  strip.show();
  j++;
  if (j >= 256 * 5) j = 0;
  lastUpdate = millis(); // time for next change to the display
}

void theaterChaseRainbow() { // modified from Adafruit example to make it a state machine
  static int j = 0, q = 0;
  patternInterval = 100;
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
  patternInterval = 100;
  static uint16_t j = 0;
  static uint16_t r = strip.numPixels();
  static bool reverse = false;
  //for(uint16_t i=0; i<strip.numPixels(); i++) {
  strip.setBrightness(brightness);
  if (!reverse) {
    strip.setPixelColor(j, c);
    strip.setPixelColor(j - 1, 0);
    j++;
  } else {
    strip.setPixelColor(r, 0);
    strip.setPixelColor(r - 1, c);
    r--;
  }

  strip.show();
  //for(uint16_t i=strip.numPixels(); i>0; i--) {
  //  strip.setPixelColor(j, c);
  //  strip.setPixelColor(j + 1, 0);
  //}
  //strip.show();

  if (j >= strip.numPixels()) { j = 0; r = strip.numPixels(); reverse = true;}
  if (r == 1) { r = strip.numPixels(); reverse = false;}
  
  lastUpdate = millis(); // time for next change to the display 
}

void colorFlicker(uint32_t c) { // modified from Adafruit example to make it a state machine
  static uint16_t j = 0;
  patternInterval = 100;
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setBrightness(brightness);
    strip.setPixelColor(random(NUM_LEDS), c);
    strip.setPixelColor(random(NUM_LEDS), 0);
  }
  strip.show();
  j++;
  if (j >= 256 * 5) j = 0;
  lastUpdate = millis(); // time for next change to the display
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
