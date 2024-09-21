#include <GxEPD2_3C.h>
#include <Adafruit_GFX.h>  // GxEPD2 is based on Adafruit GFX
#include <Fonts/FreeMonoBold9pt7b.h>  // Ensure the correct font is included
// Select display class matching the panel
#define GxEPD2_DISPLAY_CLASS GxEPD2_3C
#define GxEPD2_DRIVER_CLASS GxEPD2_290c     // GDEW029Z10 128x296

// Connections for Adafruit ESP32 Feather
static const uint8_t EPD_BUSY = 32;  // to EPD BUSY
static const uint8_t EPD_CSSS = 15;  // to EPD CS
static const uint8_t EPD_RST  = 27;  // to EPD RST
static const uint8_t EPD_DC   = 33;  // to EPD DC
static const uint8_t EPD_SCK  = 5;   // to EPD CLK
static const uint8_t EPD_MISO = 19;  // not used, no data from display
static const uint8_t EPD_MOSI = 18;  // to EPD DIN

#define MAX_DISPLAY_BUFFER_SIZE 65536ul // e.g.
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8) ? EPD::HEIGHT : (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8))

GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, MAX_HEIGHT(GxEPD2_DRIVER_CLASS)> display(GxEPD2_290c(EPD_CSSS, EPD_DC, EPD_RST, EPD_BUSY));

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");

  // Initialize the display
  display.init(115200);
  display.setRotation(1);  // Set landscape orientation

  // Map SPI pins
  SPI.begin(EPD_SCK, EPD_MISO, EPD_MOSI, EPD_CSSS);
  // Clear the display
  helloWorld();
}

void loop() {
  // Nothing to do here
}
const char HelloWorld[] = "Hello World!";

void helloWorld()
{
  //Serial.println("helloWorld");
  display.setRotation(1);
  display.setFont(&FreeMonoBold9pt7b);
  if (display.epd2.WIDTH < 104) display.setFont(0);
  display.setTextColor(GxEPD_BLACK);
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds(HelloWorld, 0, 0, &tbx, &tby, &tbw, &tbh);
  // center bounding box by transposition of origin:
  uint16_t x = ((display.width() - tbw) / 2) - tbx;
  uint16_t y = ((display.height() - tbh) / 2) - tby;
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y);
    display.print(HelloWorld);
  }
  while (display.nextPage());
  //Serial.println("helloWorld done");
}
