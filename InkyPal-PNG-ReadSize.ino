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

#define MAX_DISPLAY_BUFFER_SIZE 65536ul
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8) ? EPD::HEIGHT : (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8))

GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, MAX_HEIGHT(GxEPD2_DRIVER_CLASS)> display(GxEPD2_290c(EPD_CSSS, EPD_DC, EPD_RST, EPD_BUSY));

// Wi-Fi credentials
const char* ssid = "telenet-ap-5660427";
const char* password = "az4NstAyaasc";

// API URL to get image URL
const char* apiUrl = "https://us-central1-inkypal-98899.cloudfunctions.net/getRandomLikedImage?uid=eGEbpUu1P2Y94RKUoukIJiDuibD3";

PNG png;  // PNG decoder object
WiFiClientSecure client;

// Buffer size for downloading the image
#define BUFFER_SIZE 4096  // Adjust the buffer size as needed

// Function to handle PNG drawing line-by-line
void pngDraw(PNGDRAW *pDraw) {
  uint8_t lineBuffer[296];  // Buffer for a single line (max width 296)

  for (int i = 0; i < pDraw->iWidth; i++) {
    uint8_t r = pDraw->pPixels[i * 3 + 0];
    uint8_t g = pDraw->pPixels[i * 3 + 1];
    uint8_t b = pDraw->pPixels[i * 3 + 2];
    uint8_t gray = (r + g + b) / 3;

    lineBuffer[i] = (gray > 128) ? GxEPD_WHITE : GxEPD_BLACK;
  }

  display.writeImage(lineBuffer, 0, pDraw->y, pDraw->iWidth, 1);
}

// Function to download the image data into a buffer
bool downloadImageToBuffer(const char* imageUrl, uint8_t** imageBuffer, int32_t* imageSize) {
  client.setInsecure();

  HTTPClient https;
  if (https.begin(client, imageUrl)) {
    int httpCode = https.GET();
    Serial.printf("HTTP request returned: %d\n", httpCode);

    if (httpCode == HTTP_CODE_OK) {
      int32_t contentLength = https.getSize();
      if (contentLength <= 0) {
        Serial.println("Content-Length is invalid.");
        return false;
      }

      // Allocate buffer to hold the image
      *imageBuffer = (uint8_t*)malloc(contentLength);
      if (!*imageBuffer) {
        Serial.println("Failed to allocate memory for image buffer.");
        return false;
      }

      // Read the image data into the buffer
      int totalBytesRead = 0;
      WiFiClient* stream = https.getStreamPtr();
      while (stream->connected() && totalBytesRead < contentLength) {
        int bytesToRead = stream->available();
        if (bytesToRead > 0) {
          int bytesRead = stream->readBytes(*imageBuffer + totalBytesRead, bytesToRead);
          totalBytesRead += bytesRead;
          Serial.printf("Read %d bytes, total %d/%d\n", bytesRead, totalBytesRead, contentLength);
        }
        delay(10);  // Small delay to avoid watchdog timer issues
      }

      *imageSize = totalBytesRead;
      return true;
    } else {
      Serial.printf("HTTP request failed with code: %d\n", httpCode);
      return false;
    }
  } else {
    Serial.println("Unable to connect to image URL");
    return false;
  }
}

// Function to display the PNG image from buffer
void displayPNGFromBuffer(uint8_t* imageBuffer, int32_t imageSize) {
  int result = png.openFLASH(imageBuffer, imageSize, pngDraw);
  if (result == PNG_SUCCESS) {
    Serial.printf("Image width: %d, height: %d, bpp: %d\n", png.getWidth(), png.getHeight(), png.getBpp());
    Serial.printf("Pixel type: %d\n", png.getPixelType());

    display.setFullWindow();
    display.firstPage();
    do {
      png.decode(NULL, 0);
    } while (display.nextPage());

    display.display();
    Serial.println("PNG Image displayed successfully");
    png.close();
  } else {
    Serial.println("Failed to decode PNG. Error code: " + String(result));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");

  display.init(115200);
  display.setRotation(1);  // Landscape orientation
  display.setFullWindow();  // Full window mode

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");

  HTTPClient http;
  http.begin(apiUrl);
  int httpCode = http.GET();
  Serial.printf("API HTTP request returned: %d\n", httpCode);

  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    Serial.println("API Response: " + payload);

    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload);
    if (!error) {
      const char* imageUrl = doc["url"];
      Serial.println("Image URL: " + String(imageUrl));

      uint8_t* imageBuffer = nullptr;
      int32_t imageSize = 0;

      if (downloadImageToBuffer(imageUrl, &imageBuffer, &imageSize)) {
        displayPNGFromBuffer(imageBuffer, imageSize);
        free(imageBuffer);  // Free the buffer after use
      } else {
        Serial.println("Failed to download image.");
      }
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
