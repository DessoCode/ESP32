#include <GxEPD2_3C.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

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
      const char* imageUrl = doc["url"];
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
  WiFiClientSecure client;
  client.setInsecure();  // Bypass SSL certificate verification (Not Secure)

  HTTPClient https;
  if (https.begin(client, imageUrl)) {  // HTTPS connection
    int httpCode = https.GET();
    if (httpCode > 0) {
      if (httpCode == HTTP_CODE_OK) {
        int len = https.getSize();
        Serial.printf("Image size: %d bytes\n", len);

        // Check if content type is BMP
        String contentType = https.header("Content-Type");
        Serial.println("Content-Type: " + contentType);

        if (!contentType.startsWith("image/bmp")) {
          Serial.println("Error: Content is not BMP format");
          https.end();
          return;
        }

        // Stream and display the BMP image
        WiFiClient* stream = https.getStreamPtr();

        // Display the BMP image from the stream
        displayBMPFromStream(stream, len);

      } else {
        Serial.printf("Failed to download image, HTTP code: %d\n", httpCode);
      }
    } else {
      Serial.printf("Image download failed: %s\n", https.errorToString(httpCode).c_str());
    }
    https.end();
  } else {
    Serial.println("Unable to connect to image URL");
  }
}

// Function to read 16-bit value from the stream
uint16_t read16(WiFiClient* client) {
  uint8_t b0 = client->read();
  uint8_t b1 = client->read();
  return (uint16_t)(b1 << 8 | b0);
}

// Function to read 32-bit value from the stream
uint32_t read32(WiFiClient* client) {
  uint16_t b0 = read16(client);
  uint16_t b1 = read16(client);
  return (uint32_t)(b1 << 16 | b0);
}

// Function to skip bytes in the stream
void skipBytesInStream(WiFiClient* client, int bytesToSkip) {
  uint8_t skipBuf[64];
  while (bytesToSkip > 0) {
    size_t chunkSize = bytesToSkip > sizeof(skipBuf) ? sizeof(skipBuf) : bytesToSkip;
    size_t bytesRead = client->readBytes(skipBuf, chunkSize);
    if (bytesRead == 0) {
      // Error or EOF
      break;
    }
    bytesToSkip -= bytesRead;
  }
}

void displayBMPFromStream(WiFiClient* client, int len) {
  if (!client) {
    Serial.println("Client is null");
    return;
  }

  // Parse BMP header
  if (read16(client) != 0x4D42) { // BMP signature
    Serial.println("Not a valid BMP file");
    return;
  }

  uint32_t fileSize = read32(client);
  read32(client); // Skip creator bytes
  uint32_t imageOffset = read32(client); // Start of image data
  uint32_t headerSize = read32(client);
  int32_t width = (int32_t)read32(client);
  int32_t height = (int32_t)read32(client);
  uint16_t planes = read16(client);
  uint16_t depth = read16(client);
  uint32_t compression = read32(client);

  Serial.printf("BMP Image: %dx%d, %d bits per pixel, imageOffset=%d\n", width, height, depth, imageOffset);

  if (compression != 0) {
    Serial.println("Compression not supported");
    return;
  }

  if (depth != 24) {
    Serial.println("Only 24-bit BMP images are supported");
    return;
  }

  if (planes != 1) {
    Serial.println("Invalid BMP file");
    return;
  }

  // BMP rows are padded to 4-byte boundaries
  uint32_t rowSize = (width * 3 + 3) & ~3;

  // If height is negative, the image is stored top-down
  bool flip = true;
  if (height < 0) {
    height = -height;
    flip = false;
  }

  int w = width;
  int h = height;
  if (w > display.width()) w = display.width();
  if (h > display.height()) h = display.height();

  // Skip to the start of image data
  int skipBytes = imageOffset - 54; // We've read 54 bytes so far
  if (skipBytes > 0) {
    skipBytesInStream(client, skipBytes);
  }

  display.clearScreen();

  uint8_t sdbuffer[3 * w]; // Pixel buffer (3 bytes per pixel)
  uint8_t rgb[3];

  display.firstPage();
  do {
    for (int y = 0; y < h; y++) {
      uint32_t pos;
      if (flip) {
        pos = imageOffset + (height - 1 - y) * rowSize;
      } else {
        pos = imageOffset + y * rowSize;
      }

      // Move to the start of the line
      int bytesToSkip = pos - imageOffset;
      if (bytesToSkip > 0) {
        skipBytesInStream(client, bytesToSkip);
        imageOffset += bytesToSkip;
      }

      // Read a row of pixels
      int bytesRead = client->readBytes((char *)sdbuffer, 3 * w);
      if (bytesRead < 3 * w) {
        Serial.println("Error reading BMP image data");
        return;
      }
      imageOffset += bytesRead;

      // Process pixels
      int buffidx = 0;
      for (int x = 0; x < w; x++) {
        // BMP color order is BGR
        rgb[2] = sdbuffer[buffidx++]; // Blue
        rgb[1] = sdbuffer[buffidx++]; // Green
        rgb[0] = sdbuffer[buffidx++]; // Red

        uint16_t color;
        uint8_t r = rgb[0];
        uint8_t g = rgb[1];
        uint8_t b = rgb[2];

        // Map RGB to display colors
        if (r > 200 && g < 50 && b < 50) {
          color = GxEPD_RED; // Red pixel
        } else if (r < 50 && g < 50 && b < 50) {
          color = GxEPD_BLACK; // Black pixel
        } else {
          color = GxEPD_WHITE; // White pixel
        }

        display.drawPixel(x, y, color);
      }

      // Skip any padding bytes at the end of the row
      int padding = rowSize - (3 * width);
      if (padding > 0) {
        skipBytesInStream(client, padding);
        imageOffset += padding;
      }
    }
  } while (display.nextPage());

  Serial.println("Image displayed successfully");
}
