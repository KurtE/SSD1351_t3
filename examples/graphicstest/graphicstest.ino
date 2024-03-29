/***************************************************
  This is a library for several Adafruit displays based on ST77* drivers.

  Works with the Adafruit 1.8" TFT Breakout w/SD card
    ----> http://www.adafruit.com/products/358
  The 1.8" TFT shield
    ----> https://www.adafruit.com/product/802
  The 1.44" TFT breakout
    ----> https://www.adafruit.com/product/2088
  The 1.54" TFT breakout
    ----> https://www.adafruit.com/product/3787
  The 2.0" TFT breakout
    ----> https://www.adafruit.com/product/4311
  as well as Adafruit raw 1.8" TFT display
    ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

// This Teensy3 and 4 native optimized and extended version
// requires specific pins. 
// If you use the short version of the constructor and the DC
// pin is hardware CS pin, then it will be slower.

#define TFT_SCLK 13  // SCLK can also use pin 14
#define TFT_MOSI 11  // MOSI can also use pin 7
#define TFT_CS   10  // CS & DC can use pins 2, 6, 9, 10, 15, 20, 21, 22, 23
#define TFT_DC    9  //  but certain pairs must NOT be used: 2+10, 6+9, 20+23, 21+22
#define TFT_RST   8  // RST can use any pin
#define SD_CS     4  // CS for SD card, can use any pin

// Note the above pins are for the SPI object.  For those Teensy boards which have
// more than one SPI object, such as T3.5, T3.6, T4 which have at SPI1 and SPI2
// LC with SPI1, look at the cards that come with the teensy or the web page
// https://www.pjrc.com/teensy/pinout.html to select the appropriate IO pins.

#include <Adafruit_GFX.h>    // Core graphics library
#include <SSD1351_t3.h> // Hardware-specific library
#include <SPI.h>

// Option 1: use any pins but a little slower
// Note: code will detect if specified pins are the hardware SPI pins
//       and will use hardware SPI if appropriate
// For 1.44" and 1.8" TFT with ST7735 use
SSD1351_t3 tft = SSD1351_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Option 2: must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
//SSD1351_t3 tft = SSD1351_t3(cs, dc, rst);

float p = 3.1415926;


void setup(void) {
  pinMode(SD_CS, INPUT_PULLUP);  // don't touch the SD card
  Serial.begin(9600);
  Serial.print("hello!");

  // Use this initializer if you're using a 1.8" TFT 128x160 displays
  tft.begin();

  Serial.println("init");

  uint16_t time = millis();
  tft.fillScreen(SSD1351_BLACK);
  time = millis() - time;

  Serial.println(time, DEC);
  delay(500);

  // large block of text
  tft.fillScreen(SSD1351_BLACK);
  testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", SSD1351_WHITE);
  delay(1000);

  // tft print function!
  tftPrintTest();
  delay(4000);

  // a single pixel
  tft.drawPixel(tft.width()/2, tft.height()/2, SSD1351_GREEN);
  delay(500);

  // line draw test
  testlines(SSD1351_YELLOW);
  delay(500);

  // optimized lines
  testfastlines(SSD1351_RED, SSD1351_BLUE);
  delay(500);

  testdrawrects(SSD1351_GREEN);
  delay(500);

  testfillrects(SSD1351_YELLOW, SSD1351_MAGENTA);
  delay(500);

  tft.fillScreen(SSD1351_BLACK);
  testfillcircles(10, SSD1351_BLUE);
  testdrawcircles(10, SSD1351_WHITE);
  delay(500);

  testroundrects();
  delay(500);

  testtriangles();
  delay(500);

  mediabuttons();
  delay(500);

  Serial.println("done");
  delay(1000);
}

void loop() {
  tft.invertDisplay(true);
  delay(500);
  tft.invertDisplay(false);
  delay(500);
}

void testlines(uint16_t color) {
  tft.fillScreen(SSD1351_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(0, 0, x, tft.height()-1, color);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(0, 0, tft.width()-1, y, color);
  }

  tft.fillScreen(SSD1351_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(tft.width()-1, 0, x, tft.height()-1, color);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(tft.width()-1, 0, 0, y, color);
  }

  tft.fillScreen(SSD1351_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(0, tft.height()-1, x, 0, color);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(0, tft.height()-1, tft.width()-1, y, color);
  }

  tft.fillScreen(SSD1351_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(tft.width()-1, tft.height()-1, x, 0, color);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(tft.width()-1, tft.height()-1, 0, y, color);
  }
}

void testdrawtext(const char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

void testfastlines(uint16_t color1, uint16_t color2) {
  tft.fillScreen(SSD1351_BLACK);
  for (int16_t y=0; y < tft.height(); y+=5) {
    tft.drawFastHLine(0, y, tft.width(), color1);
  }
  for (int16_t x=0; x < tft.width(); x+=5) {
    tft.drawFastVLine(x, 0, tft.height(), color2);
  }
}

void testdrawrects(uint16_t color) {
  tft.fillScreen(SSD1351_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color);
  }
}

void testfillrects(uint16_t color1, uint16_t color2) {
  tft.fillScreen(SSD1351_BLACK);
  for (int16_t x=tft.width()-1; x > 6; x-=6) {
    tft.fillRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color1);
    tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color2);
  }
}

void testfillcircles(uint8_t radius, uint16_t color) {
  for (int16_t x=radius; x < tft.width(); x+=radius*2) {
    for (int16_t y=radius; y < tft.height(); y+=radius*2) {
      tft.fillCircle(x, y, radius, color);
    }
  }
}

void testdrawcircles(uint8_t radius, uint16_t color) {
  for (int16_t x=0; x < tft.width()+radius; x+=radius*2) {
    for (int16_t y=0; y < tft.height()+radius; y+=radius*2) {
      tft.drawCircle(x, y, radius, color);
    }
  }
}

void testtriangles() {
  tft.fillScreen(SSD1351_BLACK);
  int color = 0xF800;
  int t;
  int w = tft.width()/2;
  int x = tft.height()-1;
  int y = 0;
  int z = tft.width();
  for(t = 0 ; t <= 15; t+=1) {
    tft.drawTriangle(w, y, y, x, z, x, color);
    x-=4;
    y+=4;
    z-=4;
    color+=100;
  }
}

void testroundrects() {
  tft.fillScreen(SSD1351_BLACK);
  int color = 100;
  int i;
  int t;
  for(t = 0 ; t <= 4; t+=1) {
    int x = 0;
    int y = 0;
    int w = tft.width()-2;
    int h = tft.height()-2;
    for(i = 0 ; i <= 16; i+=1) {
      tft.drawRoundRect(x, y, w, h, 5, color);
      x+=2;
      y+=3;
      w-=4;
      h-=6;
      color+=1100;
    }
    color+=100;
  }
}

void tftPrintTest() {
  tft.setTextWrap(false);
  tft.fillScreen(SSD1351_BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(SSD1351_RED);
  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(SSD1351_YELLOW);
  tft.setTextSize(2);
  tft.println("Hello World!");
  tft.setTextColor(SSD1351_GREEN);
  tft.setTextSize(3);
  tft.println("Hello World!");
  tft.setTextColor(SSD1351_BLUE);
  tft.setTextSize(4);
  tft.print(1234.567);
  delay(1500);
  tft.setCursor(0, 0);
  tft.fillScreen(SSD1351_BLACK);
  tft.setTextColor(SSD1351_WHITE);
  tft.setTextSize(0);
  tft.println("Hello World!");
  tft.setTextSize(1);
  tft.setTextColor(SSD1351_GREEN);
  tft.print(p, 6);
  tft.println(" Want pi?");
  tft.println(" ");
  tft.print(8675309, HEX); // print 8,675,309 out in HEX!
  tft.println(" Print HEX!");
  tft.println(" ");
  tft.setTextColor(SSD1351_WHITE);
  tft.println("Sketch has been");
  tft.println("running for: ");
  tft.setTextColor(SSD1351_MAGENTA);
  tft.print(millis() / 1000);
  tft.setTextColor(SSD1351_WHITE);
  tft.print(" seconds.");
}

void mediabuttons() {
  // play
  tft.fillScreen(SSD1351_BLACK);
  tft.fillRoundRect(25, 10, 78, 60, 8, SSD1351_WHITE);
  tft.fillTriangle(42, 20, 42, 60, 90, 40, SSD1351_RED);
  delay(500);
  // pause
  tft.fillRoundRect(25, 90, 78, 60, 8, SSD1351_WHITE);
  tft.fillRoundRect(39, 98, 20, 45, 5, SSD1351_GREEN);
  tft.fillRoundRect(69, 98, 20, 45, 5, SSD1351_GREEN);
  delay(500);
  // play color
  tft.fillTriangle(42, 20, 42, 60, 90, 40, SSD1351_BLUE);
  delay(50);
  // pause color
  tft.fillRoundRect(39, 98, 20, 45, 5, SSD1351_RED);
  tft.fillRoundRect(69, 98, 20, 45, 5, SSD1351_RED);
  // play color
  tft.fillTriangle(42, 20, 42, 60, 90, 40, SSD1351_GREEN);
}
