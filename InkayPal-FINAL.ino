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
  display.init(115200, true, 2, false); // Explicitly initialize for 3-color mode
  //display.init(115200);
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
    // Ensure the screen has time to fully update
    display.display();  // Complete the display update

    delay(5000);  // Wait for 5 seconds to ensure the display fully refreshes
    
    // We don't call display.display() to avoid clearing the screen after the image
    Serial.println("PNG Image displayed successfully");
    png.close();
  } else {
    Serial.println("Failed to decode PNG. Error code: " + String(result));
  }
}

// Function to handle PNG drawing line-by-line (rotate 90 degrees counterclockwise)
void pngDraw(PNGDRAW *pDraw) {
  uint16_t lineBuffer[296]; // Use 16-bit color
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_LITTLE_ENDIAN, 0xffffffff);

  for (int i = 0; i < 296; i++) {
    uint16_t color = lineBuffer[i];
    uint8_t r = (color & 0xF800) >> 8;
    uint8_t g = (color & 0x07E0) >> 3;
    uint8_t b = (color & 0x001F) << 3;

    // Calculate brightness
    uint16_t brightness = (r + g + b) / 3;

    // Determine color based on brightness
    uint16_t epd_color;
    if (brightness > 200) {
      epd_color = GxEPD_WHITE;
    } else if (brightness < 50) {
      epd_color = GxEPD_BLACK;
    } else {
      epd_color = GxEPD_RED; // This will appear as yellow on the display
    }

    // Draw pixel with horizontal flip (i becomes the x-coordinate)
    display.drawPixel(i, pDraw->y, epd_color);
  }
}
