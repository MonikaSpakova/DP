extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <Arduino.h>
#include <WiFi.h>
#include <OneWire.h>
#include <Wire.h>
#include <SPI.h>
#include <AsyncMqttClient.h>
#include <HTTPClient.h>
#include <UrlEncode.h>

#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "DHT.h"

// Defining the pin on ESP32 
#define SENSOR_MOIST_LEFT 34
#define SENSOR_MOIST_RIGHT 35
#define ONE_WIRE_BUS 33
#define LIGHT_SENSOR_PIN 32
#define RELAY_PIN_PUMP  26
#define RELAY_PIN_VALVE_SOIL  19
#define RELAY_PIN_VENT  16
#define RELAY_PIN_LIGHT  17
#define RELAY_PIN_HEATER  18
#define LEVEL_PIN 27
#define DHTTYPE2 DHT22
#define DHTTYPE1 DHT21
#define DOOR_SENSOR_PIN 13

uint8_t DHTPin22 = 4;
uint8_t DHTPin21 = 25;
DHT dht22(DHTPin22, DHTTYPE2);
DHT dht21(DHTPin21, DHTTYPE1);
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

// Initialize a Adafruit_BME680
Adafruit_BME680 bme;

//Moisture sensor
const int airValueLeft = 3380;   //You need to replace this value with value of the sensor in the air
const int waterValueLeft = 1670; //You need to replace this value with value of the sensor in the water
const int airValueRight = 3520;  //You need to replace this value with value of the sensor in the air
const int waterValueRight = 1410; //You need to replace this value with value of the sensor in the water

// Network credentials
#define WIFI_SSID "ssis"
#define WIFI_PASSWORD "password"

// +international_country_code + phone number
String phoneNumber = "number";
String apiKey = "api";

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(x, x, x, x)
#define MQTT_PORT x
  
// MQTT Publish Topics
#define MQTT_PUB_TEMP_BME680 "esp/bme680/temperature"
#define MQTT_PUB_HUM_BME680  "esp/bme680/humidity"
#define MQTT_PUB_PRES_BME680 "esp/bme680/pressure"
#define MQTT_PUB_GAS_BME680  "esp/bme680/gas"

#define MQTT_PUB_TEMP_DHT21 "esp/dht21/temperature"
#define MQTT_PUB_HUM_DHT21 "esp/dht21/humidity"

#define MQTT_PUB_TEMP_DHT22 "esp/dht22/temperature"
#define MQTT_PUB_HUM_DHT22 "esp/dht22/humidity"

#define MQTT_PUB_LIGHT "esp/ldr/light"

#define MQTT_PUB_SOIL_RIGHT "esp/right/moisture"
#define MQTT_PUB_SOIL_LEFT "esp/left/moisture"

#define MQTT_PUB_TEMP_WATER "esp/water/temperature"
#define MQTT_PUB_TEMP_SOIL "esp/soil/temperature"

#define MQTT_PUB_LEVEL "esp/tank/level"

#define MQTT_PUB_DOOR "esp/mc-38/doors"

// MQTT Subscribe Topics
#define MQTT_SUB_LIGHT "esp/growlight/light"
#define MQTT_SUB_HEAT "esp/heater/heat"
#define MQTT_SUB_VENT "esp/ventilation/vent"
#define MQTT_SUB_PUMP "esp/reservoir/pump"
#define MQTT_SUB_VALVE "esp/soil/valve"

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings
int QoS = 2;

float BME680_temperature;
float BME680_humidity;
float BME680_pressure;
float BME680_gasResistance;

float DHT21_temperature;
float DHT21_humidity;

float DHT22_temperature;
float DHT22_humidity;

float light;
float soilmoisture_right;
float soilmoisture_left;
float water_temperature;
float soil_temperature;
bool tanklevel;
float savedValue_temperature = 0.0;
float savedValue_humidity = 0.0;

int doorState;

int status_light = 0;
int status_heat = 0;
int status_vent = 0;
int status_pump = 0;
int status_valve = 0;

// Wifi client
WiFiClient wifiClient;

// MQTT client
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void sendMessage(String message){

  // Data to send with HTTP POST
  String url = "https://api.callmebot.com/whatsapp.php?phone=" + phoneNumber + "&apikey=" + apiKey + "&text=" + urlEncode(message);    
  HTTPClient http;
  http.begin(url);

  // Specify content-type header
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  
  // Send HTTP POST request
  int httpResponseCode = http.POST(url);
  if (httpResponseCode == 200){
    Serial.print("Message sent successfully");
  }
  else{
    Serial.println("Error sending the message");
    Serial.print("HTTP response code: ");
    Serial.println(httpResponseCode);
  }

  // Free resources
  http.end();
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  // Subscribe when it connects to the broker
  uint16_t packetIdSub1 = mqttClient.subscribe(MQTT_SUB_LIGHT, 2);
  uint16_t packetIdSub2 = mqttClient.subscribe(MQTT_SUB_HEAT, 2);
  uint16_t packetIdSub3 = mqttClient.subscribe(MQTT_SUB_VENT, 2);
  uint16_t packetIdSub4 = mqttClient.subscribe(MQTT_SUB_PUMP, 2);
  uint16_t packetIdSub5 = mqttClient.subscribe(MQTT_SUB_VALVE, 2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  // Save the message in a variable
  String receivedMessage = String(payload);

  // Save topic in a String variable
  String receivedTopic = String(topic);  
  
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(receivedTopic);
  Serial.print("  payload: ");
  Serial.println(receivedMessage);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
  
  // Handle each topic differently
  if (receivedTopic == MQTT_SUB_LIGHT) {
    if (receivedMessage == "1") {
      digitalWrite(RELAY_PIN_LIGHT, LOW);
      status_light = 1;
    }
    else {
      digitalWrite(RELAY_PIN_LIGHT, HIGH);
      status_light = 0;
    }
  }
  else if (receivedTopic == MQTT_SUB_HEAT) {
    if (receivedMessage == "1") {
      digitalWrite(RELAY_PIN_HEATER, LOW);
      status_heat = 1;
    }
    else {
      digitalWrite(RELAY_PIN_HEATER, HIGH);
      status_heat = 0;
    }
  }
  else if (receivedTopic == MQTT_SUB_VENT) {
    if (receivedMessage == "1") {
      digitalWrite(RELAY_PIN_VENT, LOW);
      status_vent = 1;
    }
    else {
      digitalWrite(RELAY_PIN_VENT, HIGH);
      status_vent = 0;
    }
  }
  else if (receivedTopic == MQTT_SUB_PUMP) {
    if (receivedMessage == "1") {
      digitalWrite(RELAY_PIN_PUMP, LOW);
      status_pump = 1;
    }
    else {
      digitalWrite(RELAY_PIN_PUMP, HIGH);
      status_pump = 0;
    }
  }
  else if (receivedTopic == MQTT_SUB_VALVE) {
    if (receivedMessage == "1") {
      digitalWrite(RELAY_PIN_VALVE_SOIL, LOW);
      status_valve = 1;
    }
    else {
      digitalWrite(RELAY_PIN_VALVE_SOIL, HIGH);
      status_valve = 0;
    }
  }
}


void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

bool tankLevelSwitch(){
  // Switch to show position of the tank
  pinMode(LEVEL_PIN, INPUT_PULLUP);
  int lastState = digitalRead(LEVEL_PIN); //Tank level High/Low
  if (lastState == HIGH) 
  {
    return true;
  }
  else
  {
    return false;
  }
}

void getBME680Readings(){
  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  BME680_temperature = bme.temperature;
  BME680_pressure = bme.pressure / 100.0;
  BME680_humidity = bme.humidity;
  BME680_gasResistance = bme.gas_resistance / 1000.0;
}

void checkDHT(float& temperature, float& humidity) {
  if (isnan(temperature) || temperature < 0 || temperature > 50) {
    temperature = savedValue_temperature;
  } else {
    savedValue_temperature = temperature;
  }

  if (isnan(humidity) || humidity < 0 || humidity > 100) {
    humidity = savedValue_humidity;
  } else {
    savedValue_humidity = humidity;
  }
}

void checkMoisture(float& moisture) {
  if (moisture < 0) {
    moisture = 0;
  } 
  else if (moisture > 100) {
    moisture = 100;
  }
  else {
    moisture = moisture;
  }
}

void checkDoors(int& statedoors) {
  if (statedoors == HIGH) {
    statedoors = 0;
  } else {
    statedoors = 1;
  }
}

void errorSafety(){
   digitalWrite(RELAY_PIN_PUMP, HIGH);
   digitalWrite(RELAY_PIN_VENT, HIGH);
   digitalWrite(RELAY_PIN_HEATER, HIGH);
   digitalWrite(RELAY_PIN_LIGHT, HIGH);
   digitalWrite(RELAY_PIN_VALVE_SOIL, HIGH);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  //Wire.begin(8, 9); // initialize I2C communication with SDA on pin 8 and SCL on pin 9

  // Initialize the BME680 sensor
  if (!bme.begin(0x77)) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  }
  
  pinMode(SENSOR_MOIST_LEFT, OUTPUT);
  pinMode(SENSOR_MOIST_RIGHT, OUTPUT);
  pinMode(RELAY_PIN_PUMP, OUTPUT);
  pinMode(RELAY_PIN_VALVE_SOIL, OUTPUT);
  pinMode(RELAY_PIN_VENT, OUTPUT);
  pinMode(RELAY_PIN_LIGHT, OUTPUT);
  pinMode(RELAY_PIN_HEATER, OUTPUT);
  pinMode(LEVEL_PIN, INPUT_PULLUP);
  pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(DHTPin21, INPUT);
  pinMode(DHTPin22, INPUT);
  dht21.begin();
  dht22.begin();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  
  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToWifi();
  errorSafety();

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

}

void loop() {
  unsigned long currentMillis = millis();
  // Every x number of seconds (interval = 5 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    
    //TEMPERATURE SENZOR
       sensors.requestTemperatures();
       
      // doors
      doorState = digitalRead(DOOR_SENSOR_PIN);
      checkDoors(doorState);

      //DHT21
      DHT21_humidity = dht21.readHumidity();
      DHT21_temperature = dht21.readTemperature();

      //DHT22
      DHT22_humidity = dht22.readHumidity();
      DHT22_temperature = dht22.readTemperature();

      //Moisture sensor
      int soilMoistureValueLeft = analogRead(SENSOR_MOIST_LEFT);
      soilmoisture_left = map(soilMoistureValueLeft, airValueLeft, waterValueLeft, 0, 100);
      checkMoisture(soilmoisture_left);
      int soilMoistureValueRight = analogRead(SENSOR_MOIST_RIGHT);
      soilmoisture_right = map(soilMoistureValueRight, airValueRight, waterValueRight, 0, 100);
      checkMoisture(soilmoisture_right);
    
      // Tank Level Switch
      tanklevel = tankLevelSwitch();  

      //Light sensor
      int lightSensorValue = analogRead(LIGHT_SENSOR_PIN);
      light = map(lightSensorValue, 4095, 0, 0, 100);

      water_temperature = sensors.getTempCByIndex(0);
      soil_temperature = sensors.getTempCByIndex(1);

    getBME680Readings();
    Serial.println();
    Serial.printf("Temperature DHT21 = %.2f ºC \n", DHT21_temperature);
    Serial.printf("Humidity DHT21 = %.2f % \n", DHT21_humidity);
    Serial.printf("Temperature DHT22 = %.2f ºC \n", DHT22_temperature);
    Serial.printf("Humidity DHT22 = %.2f % \n", DHT22_humidity);
    Serial.printf("Light = %.2f % \n", light);
    Serial.printf("Soil Humidity - Left = %.2f % \n", soilmoisture_left);
    Serial.printf("Soil Humidity - Right = %.2f % \n", soilmoisture_right);
    Serial.printf("Water Temperature = %.2f ºC \n", water_temperature);
    Serial.printf("Soil Temperature = %.2f ºC \n", soil_temperature);
    Serial.printf("Temperature BME680 = %.2f ºC \n", BME680_temperature);
    Serial.printf("Humidity BME680 = %.2f % \n", BME680_humidity);
    Serial.printf("Pressure BME680 = %.2f hPa \n", BME680_pressure);
    Serial.printf("Gas Resistance BME680 = %.2f KOhm \n", BME680_gasResistance);
    Serial.printf("Light = %d \n", status_light);
    Serial.printf("Heat = %d \n", status_heat);
    Serial.printf("Pump = %d \n", status_pump);
    Serial.printf("Vent = %d \n", status_vent);
    Serial.printf("Valve = %d \n", status_valve);
    
    // Publish an MQTT message on topic esp/bme680/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP_BME680, QoS, true, String(BME680_temperature).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_TEMP_BME680, packetIdPub1);
    Serial.printf("Message: %.2f \n", BME680_temperature);

    // Publish an MQTT message on topic esp/bme680/humidity
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM_BME680, QoS, true, String(BME680_humidity).c_str());

    // Publish an MQTT message on topic esp/bme680/pressure
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_PRES_BME680, QoS, true, String(BME680_pressure).c_str());

    // Publish an MQTT message on topic esp/bme680/gas
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_GAS_BME680, QoS, true, String(BME680_gasResistance).c_str());

    // Publish an MQTT message on topic esp/dht21/temperature
    uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_TEMP_DHT21, QoS, true, String(DHT21_temperature).c_str());

    // Publish an MQTT message on topic esp/dht21/humidity
    uint16_t packetIdPub6 = mqttClient.publish(MQTT_PUB_HUM_DHT21, QoS, true, String(DHT21_humidity).c_str());

    // Publish an MQTT message on topic esp/dht21/temperature
    uint16_t packetIdPub7 = mqttClient.publish(MQTT_PUB_TEMP_DHT22, QoS, true, String(DHT22_temperature).c_str());

    // Publish an MQTT message on topic esp/dht21/humidity
    uint16_t packetIdPub8 = mqttClient.publish(MQTT_PUB_HUM_DHT22, QoS, true, String(DHT22_humidity).c_str());

    // Publish an MQTT message on topic esp/bme680/gas
    uint16_t packetIdPub9 = mqttClient.publish(MQTT_PUB_LIGHT, QoS, true, String(light).c_str());
    
    // Publish an MQTT message on topic esp/bme680/gas
    uint16_t packetIdPub10 = mqttClient.publish(MQTT_PUB_SOIL_RIGHT, QoS, true, String(soilmoisture_right).c_str());

    // Publish an MQTT message on topic esp/bme680/gas
    uint16_t packetIdPub11 = mqttClient.publish(MQTT_PUB_SOIL_LEFT, QoS, true, String(soilmoisture_left).c_str());

    // Publish an MQTT message on topic esp/bme680/gas
    uint16_t packetIdPub12 = mqttClient.publish(MQTT_PUB_LEVEL, QoS, true, String(tanklevel).c_str());
    
    // Publish an MQTT message on topic esp/bme680/gas
    uint16_t packetIdPub13 = mqttClient.publish(MQTT_PUB_TEMP_WATER, QoS, true, String(water_temperature).c_str());

    // Publish an MQTT message on topic esp/bme680/gas
    uint16_t packetIdPub14 = mqttClient.publish(MQTT_PUB_TEMP_SOIL, QoS, true, String(soil_temperature).c_str());

    // Publish an MQTT message on topic esp/bme680/gas
    uint16_t packetIdPub15 = mqttClient.publish(MQTT_PUB_DOOR, QoS, true, String(doorState).c_str());

    // Send Message to WhatsAPP
    if (soilmoisture_left < 10.0 || soilmoisture_right < 10.0 || BME680_temperature > 40.0 || tanklevel == 0 || doorState == 1) {
        String stringOne = "Hi, Monika!";
        String stringTwo = "Please check GreenHouse! Here are the parameters of the GreenHouse!:";
        String stringFinal = (stringOne + "\n"  + stringTwo + "\n" + "\n" + 
                              "Soil Moisture Left Side: " + soilmoisture_left + " %" + "\n" + 
                              "Soil Moisture Right Side: " + soilmoisture_right + " %" + "\n" +  
                              "Temperature BME680: " + BME680_temperature + " °C" + "\n" + 
                              "Pressure BME680: " + BME680_pressure + " hPa" + "\n" + 
                              "Humidity BME680: " + BME680_humidity + "%" + "\n" + 
                              "Gas BME680: " + BME680_gasResistance + " kOhm" + "\n" + 
                              "Temperature DHT21: " + DHT21_temperature + " °C" + "\n" + 
                              "Humidity DHT21: " + DHT21_humidity + " %" + "\n" + 
                              "Temperature Soil: " + soil_temperature + " °C" + "\n" + 
                              "Temperature Water: " + water_temperature + " °C" + "\n" + 
                              "Light Value: " + light + "%"  + "\n" +
                              "Level Tank: " + tanklevel + "\n" +
                              "Doors: " + doorState + "\n" +
                              "Have a nice day!");
        String textMsg = stringFinal;
        sendMessage(textMsg);
    }  
  }
}
