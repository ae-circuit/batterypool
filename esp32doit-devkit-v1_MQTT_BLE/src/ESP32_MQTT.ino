#include <WiFi.h>
#include <BluetoothSerial.h>
#include <PubSubClient.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT broker settings
const char* mqttServer = "MQTT_SERVER_ADDRESS";
const int mqttPort = 1883;
const char* mqttUsername = "YOUR_MQTT_USERNAME";
const char* mqttPassword = "YOUR_MQTT_PASSWORD";
const char* mqttTopic = "YOUR_MQTT_TOPIC";

// Bluetooth Serial object
BluetoothSerial SerialBT;

// MQTT client object
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Initialize Bluetooth Serial
  SerialBT.begin("ESP32-BLE");

  // Connect to Wi-Fi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");

  // Set MQTT server and callback function
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);
  
  // Connect to MQTT server
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT server...");
    if (mqttClient.connect("ESP32", mqttUsername, mqttPassword)) {
      Serial.println("Connected to MQTT server!");
      mqttClient.subscribe(mqttTopic);
    } else {
      Serial.print("Failed, rc=");
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }
}

void loop() {
  // Check for new BLE data
  if (SerialBT.available()) {
    String bleData = SerialBT.readString();
    Serial.println("Received from BLE: " + bleData);
    mqttClient.publish(mqttTopic, bleData.c_str());
  }

  // Maintain MQTT connection
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received from MQTT: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Send the MQTT message back to BLE
  SerialBT.println((char*)payload);
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.println("Reconnecting to MQTT server...");
    if (mqttClient.connect("ESP32", mqttUsername, mqttPassword)) {
      Serial.println("Connected to MQTT server!");
      mqttClient.subscribe(mqttTopic);
    } else {
      Serial.print("Failed, rc=");
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }
}
