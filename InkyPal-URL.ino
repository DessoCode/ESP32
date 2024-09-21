// Include necessary libraries
#include <GxEPD2_3C.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TJpg_Decoder.h>

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

// Display buffer size
#define MAX_DISPLAY_BUFFER_SIZE 65536ul
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8) ? EPD::HEIGHT : (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8))

// Instantiate the display
GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, MAX_HEIGHT(GxEPD2_DRIVER_CLASS)> display(GxEPD2_290c(EPD_CSSS, EPD_DC, EPD_RST, EPD_BUSY));

// Wi-Fi credentials
const char* ssid = "telenet-ap-5660427";
const char* password = "az4NstAyaasc";

// API URL
const char* apiUrl = "https://us-central1-inkypal-98899.cloudfunctions.net/getRandomLikedImage?uid=eGEbpUu1P2Y94RKUoukIJiDuibD3";

// Function to draw pixels on the display
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap);

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Setup started!");

  // Initialize the display
  display.init(115200);
  display.setRotation(1); // Set display orientation
  display.setFullWindow(); // Use full window mode

  // Map SPI pins
  SPI.begin(EPD_SCK, EPD_MISO, EPD_MOSI, EPD_CSSS);

  // Connect to Wi-Fi
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");

  // Fetch the image URL from the API
  HTTPClient http;
  http.begin(apiUrl);
  int httpCode = http.GET();
  if (httpCode > 0) {
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      Serial.println("API Response: " + payload);

      // Parse JSON to extract image URL
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, payload);
      if (error) {
        Serial.print("JSON deserialization failed: ");
        Serial.println(error.f_str());
        return;
      }
      const char* imageUrl = doc["image"];
      Serial.println("Image URL: " + String(imageUrl));

      // Download and display the image
      displayImageFromUrl(imageUrl);
    } else {
      Serial.printf("HTTP GET failed with code: %d\n", httpCode);
    }
  } else {
    Serial.printf("HTTP GET failed: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}

void loop() {
  // Empty loop
}

// Function to download and display image from URL
void displayImageFromUrl(const char* imageUrl) {
  HTTPClient httpImage;
  httpImage.begin(imageUrl);
  int httpCodeImage = httpImage.GET();

  if (httpCodeImage > 0) {
    if (httpCodeImage == HTTP_CODE_OK) {
      WiFiClient *stream = httpImage.getStreamPtr();
      int len = httpImage.getSize();
      if (len > 0) {
        // Allocate buffer for the image
        uint8_t *jpegData = (uint8_t *)malloc(len);
        if (jpegData) {
          int index = 0;
          while (httpImage.connected() && index < len) {
            size_t size = stream->available();
            if (size) {
              int c = stream->readBytes(jpegData + index, size);
              index += c;
            }
            delay(1);
          }

          // Initialize the JPEG decoder
          TJpgDec.setJpgScale(1);
          TJpgDec.setCallback(tft_output);

          // Clear the display
          display.firstPage();
          do {
            // Decode and render the image
            TJpgDec.drawJpg(0, 0, jpegData, len);
          } while (display.nextPage());

          // Free the buffer memory
          free(jpegData);
        } else {
          Serial.println("Failed to allocate memory for image");
        }
      } else {
        Serial.println("Image size is zero or not specified");
      }
    } else {
      Serial.printf("Failed to download image, HTTP code: %d\n", httpCodeImage);
    }
  } else {
    Serial.printf("Image download failed: %s\n", httpImage.errorToString(httpCodeImage).c_str());
  }
  httpImage.end();
}

// Callback function for the JPEG decoder
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  uint16_t *pImg = bitmap;
  uint16_t color;
  for (int16_t i = 0; i < h; i++) {
    for (int16_t j = 0; j < w; j++) {
      color = pImg[j];
      uint8_t r = ((color >> 11) & 0x1F) << 3;
      uint8_t g = ((color >> 5) & 0x3F) << 2;
      uint8_t b = (color & 0x1F) << 3;

      uint16_t displayColor;
      if (r > 200 && g < 50 && b < 50) {
        displayColor = GxEPD_RED; // Red pixel
      } else if (r < 50 && g < 50 && b < 50) {
        displayColor = GxEPD_BLACK; // Black pixel
      } else {
        displayColor = GxEPD_WHITE; // White pixel
      }

      display.drawPixel(x + j, y + i, displayColor);
    }
    pImg += w;
  }
  return true;
}
