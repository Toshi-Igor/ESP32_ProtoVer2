#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <RtcDS1302.h>  // Include DS1302 library by Makuna

// DS1302 Pins on ESP32
#define DS1302_CLK 5
#define DS1302_DAT 18
#define DS1302_RST 19

// Create a ThreeWire instance (required for RtcDS1302)
ThreeWire myWire(DS1302_DAT, DS1302_CLK, DS1302_RST);

// Initialize DS1302 RTC using the ThreeWire instance
RtcDS1302<ThreeWire> rtc(myWire);

// WiFi and WebSocket configuration
const char* ssid = "C0nVeRgE_NeT";
const char* password = "h1ND1K0@L@m";
const char* websocket_host = "192.168.1.12";
uint16_t websocket_port = 8080;
const char* websocket_url = "/";

// DHT22 Sensor Setup
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// WebSocket client object
WebSocketsClient webSocket;

// Change deviceId to an integer
int deviceId = 2;
unsigned long previousMillis = 0;
const long data_interval = 60000;  // 1 minute in milliseconds (60 * 1000)

// Time window for operation: 8 AM to 5 PM
int startHour = 8;
int endHour = 17;

// Function to generate timestamp string
String getTimestamp() {
  RtcDateTime now = rtc.GetDateTime();
  char timestamp[20];
  sprintf(timestamp, "%02u:%02u:%02u", now.Hour(), now.Minute(), now.Second());
  return String(timestamp);
}

// Function to handle errors with timestamps
void handleError(const String& errorMsg, const String& suggestion) {
  String timestamp = getTimestamp();  // Get the current time from RTC
  Serial.println("[" + timestamp + "] Error: " + errorMsg);
  Serial.println("[" + timestamp + "] Suggested fix: " + suggestion);
}

// Function to check and reconnect to Wi-Fi
void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Attempting to reconnect...");

    // Disconnect and start reconnecting
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    
    int retryCount = 0;
    const int maxRetries = 10;  // Maximum number of retry attempts
    while (WiFi.status() != WL_CONNECTED && retryCount < maxRetries) {
      delay(1000);  // Wait 1 second between retries
      Serial.print(".");
      retryCount++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nReconnected to WiFi.");
    } else {
      Serial.println("\nFailed to reconnect to WiFi.");
    }
  }
}

// WebSocket event handler
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  String timestamp = getTimestamp();  // Get the current timestamp
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("[" + timestamp + "] Disconnected from WebSocket server.");
      handleError("WebSocket Disconnected", "Check server connection or WebSocket server URL.");
      break;
    case WStype_CONNECTED:
      Serial.println("[" + timestamp + "] Connected to WebSocket server.");
      break;
    case WStype_TEXT:
      Serial.printf("[%s] Message from server: %s\n", timestamp.c_str(), payload);
      break;
    case WStype_ERROR:
      Serial.println("[" + timestamp + "] WebSocket error occurred.");
      handleError("WebSocket Error", "Check network connectivity or server health.");
      break;
  }
}

// Function to check the RTC status
void checkRtcStatus() {
  if (!rtc.IsDateTimeValid()) {
    Serial.println("RTC is not running or needs reset!");

    // Optionally reset the time if needed
    // rtc.SetDateTime(RtcDateTime(__DATE__, __TIME__));  // Only use this if the RTC has lost its time
  } else {
    RtcDateTime now = rtc.GetDateTime();
    Serial.print("RTC is running. Current time: ");
    Serial.print(now.Hour());
    Serial.print(":");
    Serial.print(now.Minute());
    Serial.print(":");
    Serial.println(now.Second());
  }
}

void setup() {
  Serial.begin(115200);

  // Start the DHT sensor
  dht.begin();

  // Initialize RTC
  rtc.Begin();

  // For first-time use, set the RTC to the current compile time (only do this once)
  //RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  //rtc.SetDateTime(compiled);

  // Check RTC status after setting the time
  checkRtcStatus();  // Print the current time to verify

  // Connect to WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  int retries = 0;
  const int maxRetries = 10;

  while (WiFi.status() != WL_CONNECTED && retries < maxRetries) {
    delay(1000);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
  } else {
    handleError("Failed to connect to WiFi", "Check WiFi credentials or router availability.");
    return;  // Exit if WiFi connection fails
  }

  // Setup WebSocket connection
  Serial.println("Connecting to WebSocket server...");
  webSocket.begin(websocket_host, websocket_port, websocket_url);
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  checkWiFiConnection();  // Check Wi-Fi connection periodically

  RtcDateTime currentTime = rtc.GetDateTime();
  int currentHour = currentTime.Hour();
  int currentMinute = currentTime.Minute();
  int currentSecond = currentTime.Second();

  // Check if the current time is within operational hours (8 AM to 5 PM)
  if (currentHour >= startHour && currentHour < endHour) {
    webSocket.loop();  // Handle WebSocket events

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= data_interval) {
      previousMillis = currentMillis;

      float t = dht.readTemperature();
      float h = dht.readHumidity();
      float hi = dht.computeHeatIndex(t, h, false);

      if (isnan(t) || isnan(h) || isnan(hi)) {
        handleError("Failed to read from DHT sensor", "Ensure the sensor is connected properly and try resetting the device.");
        return;
      }

      // Get timestamp from RTC (for Serial Monitor only)
      String timestamp = getTimestamp();

      // Check if WebSocket is connected before sending data
      if (webSocket.isConnected()) {
        String jsonData = "{\"deviceId\":" + String(deviceId) + ",\"temperature\":" + String(t) + ",\"humidity\":" + String(h) + ",\"heat_index\":" + String(hi) + "}";

        // Print data to Serial with timestamp
        Serial.println("[" + timestamp + "] Sent data: " + jsonData);

        // Send data via WebSocket (without timestamp)
        webSocket.sendTXT(jsonData);
      } else {
        handleError("WebSocket not connected", "Check if WebSocket server is online and reachable.");
      }
    }
  }
}
