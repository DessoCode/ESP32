#include <GxEPD2_3C.h>
#include <Adafruit_GFX.h>  // GxEPD2 is based on Adafruit GFX
#include <Fonts/FreeMonoBold9pt7b.h>  // Ensure the correct font is included
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <PNGdec.h>  // Library for handling PNG

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

// Wi-Fi credentials
const char* ssid = "telenet-ap-5660427";
const char* password = "az4NstAyaasc";

// API URL to get image URL
const char* apiUrl = "https://us-central1-inkypal-98899.cloudfunctions.net/getRandomLikedImage?uid=eGEbpUu1P2Y94RKUoukIJiDuibD3";

WiFiClientSecure client;

// PNG decoder object
PNG png;

// Custom file handling functions for PNGdec
void* pngOpen(const char* filename, int32_t* size) {
  // This function is not used since we are reading from a stream
  return nullptr;
}

void pngClose(void* handle) {
  // Close the connection to the client
}

int32_t pngRead(PNGFILE* handle, uint8_t* buffer, int32_t length) {
  return client.read(buffer, length);  // Read the requested number of bytes from the stream
}

int32_t pngSeek(PNGFILE* handle, int32_t position) {
  // Seeking is not supported in streaming from HTTP
  return 0;
}

// Function to draw PNG image
void pngDraw(PNGDRAW* pDraw) {
  uint8_t lineBuffer[128];  // Buffer for a line of pixels (adjust to width of the image)

  for (int i = 0; i < pDraw->iWidth; i++) {
    uint8_t r = pDraw->pPixels[i * 3 + 0];
    uint8_t g = pDraw->pPixels[i * 3 + 1];
    uint8_t b = pDraw->pPixels[i * 3 + 2];

    uint8_t color;
    if (r > 200 && g < 50 && b < 50) {
      color = GxEPD_RED;  // Red
    } else if (r < 50 && g < 50 && b < 50) {
      color = GxEPD_BLACK;  // Black
    } else {
      color = GxEPD_WHITE;  // White
    }
    lineBuffer[i] = color;
  }

  display.writeImage(lineBuffer, nullptr, 0, pDraw->y, pDraw->iWidth, 1);  // Draw the line on the display
}

// Function to fetch and display PNG from URL
void displayPNGFromUrl(const char* imageUrl) {
  HTTPClient https;
  if (https.begin(client, imageUrl)) {
    int httpCode = https.GET();
    if (httpCode == HTTP_CODE_OK) {
      WiFiClient* stream = https.getStreamPtr();
      int32_t rc = png.open((const char*)stream, pngOpen, pngClose, pngRead, pngSeek, pngDraw);

      if (rc == PNG_SUCCESS) {
        png.decode(nullptr, 0);
        png.close();
        Serial.println("PNG Image displayed successfully.");
      } else {
        Serial.printf("Error decoding PNG image, error code: %d\n", rc);
      }
    } else {
      Serial.printf("Failed to download image, HTTP code: %d\n", httpCode);
    }
    https.end();
  } else {
    Serial.println("Unable to connect to image URL");
  }
}

// Wi-Fi connect function
void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");

  // Initialize the display
  display.init(115200);
  display.setRotation(1);  // Set landscape orientation
  SPI.begin(EPD_SCK, EPD_MISO, EPD_MOSI, EPD_CSSS);

  // Connect to Wi-Fi
  connectToWiFi();

  // Fetch and display PNG image from the API
  display.fillScreen(GxEPD_WHITE);  // Clear the display to white
  display.display();  // Update display to apply the clear screen
  Serial.println("Display cleared to white.");

  // Call API to fetch and display the image
  displayPNGFromUrl(apiUrl);
}

void loop() {
  // Nothing to do here
}
