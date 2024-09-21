#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <GxEPD2_3C.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <HTTPClient.h>
#include <PNGdec.h>

// Define display class matching the panel
#define GxEPD2_DISPLAY_CLASS GxEPD2_3C
#define GxEPD2_DRIVER_CLASS GxEPD2_290c

// Connections for Adafruit ESP32 Feather
static const uint8_t EPD_BUSY = 32;
static const uint8_t EPD_CSSS = 15;
static const uint8_t EPD_RST  = 27;
static const uint8_t EPD_DC   = 33;
static const uint8_t EPD_SCK  = 5;
static const uint8_t EPD_MOSI = 18;

GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, 128> display(GxEPD2_290c(EPD_CSSS, EPD_DC, EPD_RST, EPD_BUSY));

// Wi-Fi credentials
const char* ssid = "telenet-ap-5660427";
const char* password = "az4NstAyaasc";

// API URL to get image URL
const char* apiUrl = "https://us-central1-inkypal-98899.cloudfunctions.net/getRandomLikedImage?uid=eGEbpUu1P2Y94RKUoukIJiDuibD3";

WiFiClientSecure client;
PNG png;

// Adjust this value to shift the image left or right
int xOffset = 0;  // Set this to a positive number to shift right, negative to shift left

// Function to download the image data into a buffer
bool downloadImageToBuffer(const char* imageUrl, uint8_t** imageBuffer, int32_t* imageSize) {
  client.setInsecure();  // Disable certificate verification

  HTTPClient https;
  if (https.begin(client, imageUrl)) {
    int httpCode = https.GET();
    Serial.printf("HTTP request returned: %d\n", httpCode);

    if (httpCode == HTTP_CODE_OK) {
      int32_t contentLength = https.getSize();
      Serial.printf("Content Length: %d bytes\n", contentLength);  // Log image size
      if (contentLength <= 0) {
        Serial.println("Invalid Content-Length.");
        return false;
      }

      *imageBuffer = (uint8_t*)malloc(contentLength);
      if (!*imageBuffer) {
        Serial.println("Failed to allocate memory for image buffer.");
        return false;
      }

      int totalBytesRead = 0;
      WiFiClient* stream = https.getStreamPtr();
      while (stream->connected() && totalBytesRead < contentLength) {
        int bytesToRead = stream->available();
        if (bytesToRead > 0) {
          int bytesRead = stream->readBytes(*imageBuffer + totalBytesRead, bytesToRead);
          totalBytesRead += bytesRead;
          Serial.printf("Read %d bytes, total %d/%d\n", bytesRead, totalBytesRead, contentLength);
        }
        delay(10);
      }

      *imageSize = totalBytesRead;
      Serial.printf("Total Image Size: %d bytes\n", *imageSize);  // Log the final size after download
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

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");

  display.init(115200);
  display.setRotation(1);  // Reintroducing rotation to match the image orientation
  display.setFullWindow();

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
        free(imageBuffer);
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

void displayPNGFromBuffer(uint8_t* imageBuffer, int32_t imageSize) {
  int result = png.openFLASH(imageBuffer, imageSize, pngDraw);
  if (result == PNG_SUCCESS) {
    Serial.printf("Image width: %d, height: %d, bpp: %d\n", png.getWidth(), png.getHeight(), png.getBpp());

    // Set up the display to not clear or flash after each update
    display.setPartialWindow(0, 0, 296, 128);  // Set to full display area

    display.firstPage();  // Start the page but don't clear the screen
    do {
      png.decode(NULL, 0);  // Decode image (no refresh until finished)
    } while (display.nextPage());

    // We don't call display.display() to avoid clearing the screen after the image
    Serial.println("PNG Image displayed successfully");
    png.close();
  } else {
    Serial.println("Failed to decode PNG. Error code: " + String(result));
  }
}

// Function to handle PNG drawing line-by-line
void pngDraw(PNGDRAW *pDraw) {
  int width = 128;   // Target height of the display (since it's rotated)
  int height = 296;  // Target width of the display (since it's rotated)

  uint8_t lineBuffer[128];  // Adjust buffer for the correct width (128)

  for (int i = 0; i < width; i++) {
    uint8_t r = pDraw->pPixels[i * 3 + 0];
    uint8_t g = pDraw->pPixels[i * 3 + 1];
    uint8_t b = pDraw->pPixels[i * 3 + 2];
    uint8_t gray = (r + g + b) / 3;

    lineBuffer[i] = (gray > 128) ? GxEPD_WHITE : GxEPD_BLACK;
  }

  // Manually rotate image by drawing each pixel at a rotated position
  for (int i = 0; i < width; i++) {
    // Adjust X and Y coordinates to shift the image based on xOffset
    display.drawPixel(pDraw->y, width - i - 1 + xOffset, lineBuffer[i]);  // Rotate by 90 degrees with X offset
  }
}
