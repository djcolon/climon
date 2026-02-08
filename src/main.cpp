// Libraries ///////////////////////////////////////////////////////////////////
#include <Arduino.h>
// Watchdog
#include <esp_task_wdt.h>   
// WiFi
#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <ESPmDNS.h>
// BME280
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
// Server
#include "ESPAsyncWebServer.h"
// Preferences
#include <Preferences.h>

// Definitions /////////////////////////////////////////////////////////////////
// Pins
#define INTERNAL_LED 8
#define BME_SDA 8
#define BME_SCL 9
#define BUTTON_PIN 1

// Config
#define HOSTNAME "cliMon"
String hostname = HOSTNAME;

// Globals
Adafruit_BME280 bme;
AsyncWebServer server(80);
WiFiManager wm; // global wm instance
unsigned long lastConnectionTry;
uint64_t uptime = 0;
// Track how often we have tried to reconnect.
int retry_count = 0;
#define MAX_RETRY_COUNT 3
#define CONNECTION_RETRY_INTERVAL 30000
#define WDT_TIMEOUT 10

float temperature = 0.0;
float humidity = 0.0;
float pressure = 0.0;

// Functions ///////////////////////////////////////////////////////////////////
/**
 * Sets up mDNS so we have a name on the network.
*/
void setupMdns() {
    //initialize mDNS service
    esp_err_t err = mdns_init();
    if (err) {
        printf("MDNS Init failed: %d\n", err);
        return;
    }

    //set hostname
    esp_err_t result = mdns_hostname_set(hostname.c_str());
    if(result == ESP_OK) {
      Serial.println("mdns_hostname_set success.");
    }
    else if(result == ESP_ERR_INVALID_ARG) {
      Serial.println("mdns_hostname_set invalid arg.");
    }
    else if(result == ESP_ERR_NO_MEM) {
      Serial.println("mdns_hostname_set no mem.");
    }
    //set default instance
    result = mdns_instance_name_set(hostname.c_str());
    if(result == ESP_OK) {
      Serial.println("mdns_instance_name_set success.");
    }
    else if(result == ESP_ERR_INVALID_ARG) {
      Serial.println("mdns_instance_name_set invalid arg.");
    }
    else if(result == ESP_ERR_NO_MEM) {
      Serial.println("mdns_instance_name_set no mem.");
    }
}

/**
 * Tries to get hostname from preferences.
 * Returns default if not found.
 */
void loadPreferences() {
  Preferences preferences;
  preferences.begin("cliMon", true);
  hostname = preferences.getString("hostname", HOSTNAME);
  Serial.println("Loaded hostname from preferences: '" + hostname + "'");
  preferences.end();
}

/**
 * Tries to store hostname to preferences.
 * Returns default if not found.
 */
void savePreferences() {
  Preferences preferences;
  preferences.begin("cliMon", false);
  preferences.putString("hostname", hostname);
  Serial.println("Saved hostname to preferences: '" + hostname + "'");
  preferences.end();
}

/**
 * Removes preferences.
 */
void clearPreferences() {
  Preferences preferences;
  preferences.begin("cliMon", false);
  preferences.clear();
  Serial.println("Cleared preferences.");
  preferences.end();
}

/**
 * Called on boot.
*/
void setup() {
  // Serial
  Serial.begin(115200);

  // Watchdog
  Serial.println("Configuring watchdog.");
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  // Pins
  pinMode(BUTTON_PIN, INPUT);
  pinMode(INTERNAL_LED, OUTPUT);
  digitalWrite(INTERNAL_LED, LOW);
  // Get preferences.
  loadPreferences();
  // Set hostname.
  WiFi.hostname(hostname);

  // WiFi
  // Local initialization. Once its business is done, there is no need to keep
  // it around.
  WiFiManager wm;
  if(digitalRead(BUTTON_PIN) == HIGH) {
    Serial.println("Resetting wifi settings.");
    wm.resetSettings();
    clearPreferences();
    digitalWrite(INTERNAL_LED, LOW);
    for(int i = 0; i < 4; i++) {
      delay(250);
      digitalWrite(INTERNAL_LED, HIGH);
      delay(250);
      digitalWrite(INTERNAL_LED, LOW);
    }
    ESP.restart();
  }
  // fetches ssid and pass from eeprom and tries to connect
  // if it does not connect it starts an access point with the specified name
  // here  "AutoConnectAP"
  // and goes into a blocking loop awaiting configuration
  WiFiManagerParameter custom_hostname("hostname", "Hostname", "", 40);
  wm.addParameter(&custom_hostname);
  if(!wm.autoConnect(HOSTNAME)) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(5000);
  }
  // Get the custom hostname from wifiManager.
  char hostname_param[40];
  strcpy(hostname_param, custom_hostname.getValue());
  // We set the default to be empty, so if it is not, set it and store it.
  if((hostname_param != NULL) && (hostname_param[0] != '\0')) {
    hostname = String(hostname_param);
    savePreferences();
    delay(3000);
    ESP.restart();
  }

  lastConnectionTry = millis();
  
  // if you get here you have connected to the WiFi
  Serial.println("Connected to WiFi. Starting.");
  // Print out the IP.
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // Then set up MDNS
  setupMdns();

  // BME280
  Wire.begin(BME_SDA, BME_SCL);
  unsigned status;
    
  // default settings
  status = bme.begin(0x76, &Wire);  
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10);
  }
  
  Serial.println("-- Default Test --");

  Serial.println();

  // Set up our endpoint.
  // Respond with a json object containing measurements.
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) { 
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->print("{");
    response->print("\"i\":\"" + hostname + "\"");
    response->printf("\"t\":%.2f,", temperature);
    response->printf("\"p\":%.2f,", pressure);
    response->printf("\"rh\":%.2f", humidity);
    response->printf("\"u\":%d", uptime);
    response->print("}");
    request->send(response);
  }); 
  server.begin();

  // And we're done.
}

/**
 * Prints values read from sensor.
 */
void printValues() {
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Uptime = ");
  Serial.print(uptime);
  Serial.println(" s");

  Serial.println();
}

/**
 * Main application loop.
*/
void loop() {
  unsigned long currentMillis = millis();
  if(WiFi.status() == WL_CONNECTED) {
    uptime = (currentMillis - lastConnectionTry) / 1000;
    // Temp in °C
    temperature = bme.readTemperature();
    // Pressure in hPa
    pressure = bme.readPressure() / 100.0F;
    // Humidity in %
    humidity = bme.readHumidity();
    printValues();
    Serial.println("Hostname: '" + hostname + "'");
    // Do our values make sense?
    if(
      temperature < -30.0 ||
      temperature > 60.0 ||
      humidity < 1.0 ||
      humidity > 100.0
    ) {
      Serial.println("Read unlikely values from sensor. Restarting...");
      ESP.restart();
    }
    else {
      Serial.println("Resetting WDT with valid sensor values...");
      esp_task_wdt_reset();
    }
  }
  // if WiFi is down, try reconnecting
  else if(currentMillis - lastConnectionTry >= CONNECTION_RETRY_INTERVAL) {
    retry_count++;
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    lastConnectionTry = currentMillis;
    if(retry_count >= MAX_RETRY_COUNT) {
      // Don't actually need to do this I suppose.
      retry_count = 0;
      ESP.restart();
    }
    else {
      Serial.println("Resetting WDT whislt reconnecting...");
      esp_task_wdt_reset();
    }
  }
  
  delay(5000);
}
