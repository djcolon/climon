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
// Version
#include "version.h"

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
unsigned long lastConnectionTry;
uint64_t uptime = 0;
// WiFi
WiFiManager wm;
WiFiManagerParameter custom_hostname("hostname", "Hostname", "", 40);
// Track how often we have tried to reconnect.
int retry_count = 0;
#define MAX_RETRY_COUNT 3
#define CONNECTION_RETRY_INTERVAL 30000
#define WDT_TIMEOUT 10
// Sensor values
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
 * Sets up the temp sensor.
 */
void setupSensor() {
  // BME280
  Wire.begin(BME_SDA, BME_SCL);
  unsigned status;
  
  // Init temp sensor.
  status = bme.begin(0x76, &Wire);
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10);
  }
  // Sample infrequently to prevent self-heating causing overly high readings.
  bme.setSampling(
    Adafruit_BME280::MODE_FORCED,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::FILTER_OFF
  );
}

/**
 * Set up our webserver.
 */
void setupServer() {
  // Respond with a json object containing measurements.
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) { 
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->print("{");
    response->print("\"h\":\"" + hostname + "\",");
    response->printf("\"t\":%.2f,", temperature);
    response->printf("\"p\":%.2f,", pressure);
    response->printf("\"rh\":%.2f,", humidity);
    response->printf("\"u\":%d,", uptime);
    response->printf("\"v\":\"%s\"", VERSION);
    response->print("}");
    request->send(response);
  }); 
  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404, "text/plain", "The princess is in another castle. Go away.");
  });
  server.begin();
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
 * Called when the config portal saves config.
 *
 * Prints configured params.
 * Validates them and stores them.
 */
void saveParamsCallback () {
  Serial.println("Get Params:");
  Serial.print(custom_hostname.getID());
  Serial.print(" : ");
  Serial.println(custom_hostname.getValue());

  // Get the custom hostname from wifiManager.
  char hostname_param[40];
  strcpy(hostname_param, custom_hostname.getValue());
  // We set the default to be empty, so if it is not, set it and store it.
  if((hostname_param != NULL) && (hostname_param[0] != '\0')) {
    hostname = String(hostname_param);
    savePreferences();
    // Then restart to properly init.
    //delay(2000);
    //ESP.restart();
  }
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
  wm.addParameter(&custom_hostname);
  wm.setConfigPortalBlocking(false);
  wm.setSaveParamsCallback(saveParamsCallback);
  if(!wm.autoConnect(HOSTNAME)) {
    Serial.println("Failed to connect to wifi. Starting config portal.");
  }

  lastConnectionTry = millis();
  
  // if you get here you have connected to the WiFi
  Serial.println("Connected to WiFi. Starting.");
  // Print out the IP.
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // Then set up our sub-systems.
  setupMdns();
  setupSensor();
  setupServer();

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

  Serial.print("IP = ");
  Serial.println(WiFi.localIP().toString());

  Serial.print("Hostname = ");
  Serial.println(hostname);

  Serial.print("Version = ");
  Serial.println(VERSION);

  Serial.println();
}

/**
 * Reads the sensor in forced mode, and stores values to globals.
 */
void readSensor() {
  bme.takeForcedMeasurement();
  // Temp in °C
  temperature = bme.readTemperature();
  // Pressure in hPa
  pressure = bme.readPressure() / 100.0F;
  // Humidity in %
  humidity = bme.readHumidity();
}

/**
 * Main application loop.
*/
void loop() {
  unsigned long currentMillis = millis();

  // If we're running the WifiManager config portal, process it here.
  wm.process();

  if(WiFi.status() == WL_CONNECTED) {
    // Reset last connection time if current millis overflows (every ~49 days)
    if(currentMillis < lastConnectionTry) {
      lastConnectionTry = currentMillis - 1;
    }
    uptime = (currentMillis - lastConnectionTry) / 1000;

    readSensor();
    printValues();

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
      esp_task_wdt_reset();
    }
  }
  
  if(wm.getConfigPortalActive()) {
    esp_task_wdt_reset();
  }
  else {
    // Only delay our sensor reads if we're not running captive portal.
    delay(5000);
  }
}
