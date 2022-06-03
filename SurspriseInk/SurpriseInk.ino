//#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMono12pt7b.h>


// Include images:
// We can show images using bitmaps, which usually we store in seperate files
// inside the sketch folder. Those "*.h" files contain the image data.
// We need to include them here to be able to use them.
#include "skulltest.h"




// Connections for Adafruit ESP32 Feather
static const uint8_t EPD_BUSY = 32;  // Ada 32 to EPD BUSY
static const uint8_t EPD_CS   = 15;  // Ada 15 to EPD CS
static const uint8_t EPD_RST  = 27; // Ada 27 to EPD RST
static const uint8_t EPD_DC   = 33; // Ada 33 to EPD DC
static const uint8_t EPD_SCK  = 5; // Ada 5 to EPD CLK
static const uint8_t EPD_MISO = 19; // Master-In Slave-Out not used, as no data from display
static const uint8_t EPD_MOSI = 18; // Ada 18 to EPD DIN

//Include the appropriate line from the 'GxEPD2_Example' code for your particular display.
//This works for the 2.9 inch black and white display.  Add pin numbers.
GxEPD2_3C<GxEPD2_290c, GxEPD2_290c::HEIGHT> display(GxEPD2_290c(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

void setup() {
  Serial.begin(115200);
  SPI.begin(EPD_SCK, EPD_MISO, EPD_MOSI, EPD_CS);
  // Initialize display and set font
  display.init(115200);
  display.setRotation(3);     // landscape orientaion
  display.setFullWindow(); 
  display.setFont(&FreeMono12pt7b);
  display.setTextColor(GxEPD_BLACK, GxEPD_WHITE);

  // Write something to display buffer
  display.fillScreen(GxEPD_WHITE);
  //display.setCursor(2, 20);
  //display.print("Hello World!");
  // Color options are GxEPD_BLACK, GxEPD_WHITE, GxEPD_RED
  //display.drawBitmap(0,0, gImage_yt1, 200,45, GxEPD_BLACK);  // Print YouTube logo - Black part (POSITION_X, POSITION_Y, IMAGE_NAME, IMAGE_WIDTH, IMAGE_HEIGHT, COLOR)
  //display.drawBitmap(0,0, yt2, 200,45, GxEPD_BLACK);  // Print YouTube logo - Red part (POSITION_X, POSITION_Y, IMAGE_NAME, IMAGE_WIDTH, IMAGE_HEIGHT, COLOR)
  display.drawBitmap(0,0, gImage_skulltest, 296,128, display.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);  // Print YouTube logo - Red part (POSITION_X, POSITION_Y, IMAGE_NAME, IMAGE_WIDTH, IMAGE_HEIGHT, COLOR)
                                          // In the color part we put that if the display supports red color then use it. If not, use black.

  // Update display
  display.display();
  Serial.print("Test");
}

void loop() {
  // These aren't the droids you're looking for.
}
