#include <GxEPD2_3C.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <PNGdec.h>

// Define display class and driver
#define GxEPD2_DISPLAY_CLASS GxEPD2_3C
#define GxEPD2_DRIVER_CLASS GxEPD2_290c  // GDEW029Z10 128x296

// Display pin definitions
static const uint8_t EPD_BUSY = 32;  // to EPD BUSY
static const uint8_t EPD_CSSS = 15;  // to EPD CS
static const uint8_t EPD_RST  = 27;  // to EPD RST
static const uint8_t EPD_DC   = 33;  // to EPD DC
static const uint8_t EPD_SCK  = 5;   // to EPD CLK
static const uint8_t EPD_MISO = 19;  // not used, no data from display
static const uint8_t EPD_MOSI = 18;  // to EPD DIN

// Instantiate the display
GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, GxEPD2_290c::HEIGHT> display(GxEPD2_290c(EPD_CSSS, EPD_DC, EPD_RST, EPD_BUSY));

// Wi-Fi credentials
const char* ssid = "telenet-ap-5660427";
const char* password = "az4NstAyaasc";

// API URL to get image URL
const char* apiUrl = "https://us-central1-inkypal-98899.cloudfunctions.net/getRandomLikedImage?uid=eGEbpUu1P2Y94RKUoukIJiDuibD3";

// PNG decoder object
PNG png;
WiFiClientSecure client;

// Function to handle PNG drawing to e-ink
// Function to handle PNG drawing to e-ink
// Function to handle PNG drawing to e-ink
void pngDraw(PNGDRAW *pDraw) {
  uint16_t y = pDraw->y;  // Use pDraw->y for the y-coordinate, as pDraw provides this

  uint8_t lineBuffer[128];  // Buffer size based on the width of the image

  for (int i = 0; i < pDraw->iWidth; i++) {
    // Extract RGB values
    uint8_t r = pDraw->pPixels[i * 3 + 0];
    uint8_t g = pDraw->pPixels[i * 3 + 1];
    uint8_t b = pDraw->pPixels[i * 3 + 2];

    // Map RGB to e-paper colors (black, white, red)
    uint8_t color;
    if (r > 200 && g < 50 && b < 50) {
      color = GxEPD_RED;  // Red
    } else if (r < 50 && g < 50 and b < 50) {
      color = GxEPD_BLACK;  // Black
    } else {
      color = GxEPD_WHITE;  // White
    }
    lineBuffer[i] = color;
  }

  // Draw the line on the e-paper display at the correct y-position
  display.drawImage(lineBuffer, 0, y, pDraw->iWidth, 1);  // x starts at 0
}



// Function to download and display PNG from URL
void displayPNGFromUrl(const char* imageUrl) {
  client.setInsecure();  // Bypass SSL certificate verification

  HTTPClient https;
  if (https.begin(client, imageUrl)) {
    int httpCode = https.GET();
    if (httpCode > 0 && httpCode == HTTP_CODE_OK) {
      WiFiClient* stream = https.getStreamPtr();
      if (png.open((const char*)stream, pngOpen, pngClose, pngRead, pngSeek, pngDraw) == PNG_SUCCESS) {
        display.clearScreen();
        png.decode(NULL, 0);  // Start decoding
        png.close();
        display.display();  // Refresh display after drawing the image
        Serial.println("PNG Image displayed successfully");
      } else {
        Serial.println("Failed to decode PNG");
      }
    } else {
      Serial.printf("HTTP request failed with code: %d\n", httpCode);
    }
    https.end();
  } else {
    Serial.println("Unable to connect to image URL");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");

  // Initialize display
  display.init(115200);
  display.setRotation(1);  // Landscape orientation
  display.setFullWindow();  // Full window mode

  // Map SPI pins
  SPI.begin(EPD_SCK, EPD_MISO, EPD_MOSI, EPD_CSSS);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");

  // Fetch image URL from API
  HTTPClient http;
  http.begin(apiUrl);
  int httpCode = http.GET();
  if (httpCode > 0 && httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    Serial.println("API Response: " + payload);

    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload);
    if (!error) {
      const char* imageUrl = doc["url"];
      Serial.println("Image URL: " + String(imageUrl));
      displayPNGFromUrl(imageUrl);  // Download and display the image
    } else {
      Serial.println("Failed to parse JSON");
    }
  } else {
    Serial.printf("Failed to get image URL from API, code: %d\n", httpCode);
  }
  http.end();
}

void loop() {
  // Empty loop
}

// Custom file handling functions for PNGdec
void *pngOpen(const char *filename, int32_t *size) {
  return nullptr;
}

void pngClose(void *handle) {
  // Nothing to close here
}

int32_t pngRead(PNGFILE *handle, uint8_t *buffer, int32_t length) {
  return client.read(buffer, length);  // Read from stream
}

int32_t pngSeek(PNGFILE *handle, int32_t position) {
  return 0;  // Seeking not supported for streaming
}
