#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>
#include <Firebase_ESP_Client.h>
#include "ThingSpeak.h"
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

HardwareSerial SerialPort(2);

#define DATABASE_URL "<Firebase Realtime Database Link>"
#define API_KEY "<Firebase API Key>"
#define SSID "<WIFI SSID>"
#define PASSWORD "<WIFI PASSWORD>"
#define USER_EMAIL "<Firebase Database User Email>"
#define USER_PASSWORD "<Firebase Database User Password>"
#define CHANNEL_ID <Thing Speak Channel ID>
#define THINGSPEAK_API_KEY "<Thing Speak API Key>"

const char* mqttServer = "broker.mqtt.cool";
const char *ntpServerName = "pool.ntp.org";
const int mqttPort = 1883;
String receivedValue = "-1";
String lastReceivedValue = "-2";
int receivedCount = 1;
char logMessage[50];
char receivedChars[10];
char formattedTime[20];
char pumpState[10];
String uid;
String mode = "Sensor";

WiFiClient espClient;
WiFiClient thingSpeakClient;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServerName);
PubSubClient client(espClient);
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

void setup() {
  Serial.begin(115200);
  SerialPort.begin(9600, SERIAL_8N1, 17, 16);
  connectToWiFi();
  connectToMQTT();
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  Firebase.reconnectWiFi(true);
  config.token_status_callback = tokenStatusCallback;
  config.max_token_generation_retry = 5;
  Firebase.begin(&config, &auth);
  Serial.println("Getting User UID");
  while ((auth.token.uid) == "") {
    Serial.print('.');
    delay(1000);
  }
  // Print user UID
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);
  ThingSpeak.begin(thingSpeakClient);
  timeClient.begin();
}
void connectToWiFi() {
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void connectToMQTT() {
  client.setServer(mqttServer, mqttPort);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("2022-CS-56/esp/esp-controller")) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("Failed with state ");
      Serial.println(client.state());
      delay(2000);
    }
  }
  client.subscribe("2022-CS-56/esp/pumpControl");
  client.subscribe("2022-CS-56/esp/modeIndicator");
  client.setCallback(callback);
}

void publishMessage(char* mqttTopic, char* message) {
  if (client.connected()) {
    client.publish(mqttTopic, message);
    Serial.println("Message published to MQTT");
  } else {
    Serial.println("Failed to publish message. Reconnecting to MQTT...");
    connectToMQTT(); // Reconnect to MQTT
    if (client.connected()) {
      client.publish(mqttTopic, message);
      Serial.println("Message published to MQTT");
    } else {
      Serial.print("Failed to reconnect with state ");
      Serial.println(client.state());
    }
  }
}

void updateFormattedDate() {
  // Update the NTP client and get the current time
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();

  // Convert epoch time to a printable format
  struct tm *timeInfo;
  timeInfo = gmtime((time_t *)&epochTime); // Use gmtime for UTC time
  strftime(formattedTime, sizeof(formattedTime), "%Y-%m-%d %H:%M:%S", timeInfo);
}

void sendLogMessage(String logString) {
  logString.toCharArray(logMessage, sizeof(logMessage));
  publishMessage("2022-CS-56/esp/logs", logMessage);
}

void sendMoistureLevel(char* level) {
  Serial.print("Received Number: ");
  Serial.println(level);

  publishMessage("2022-CS-56/esp/moistureLevel", level);
  sendLogMessage(String(formattedTime) + " Sensor Reading: " + String(level));
  
  if (Firebase.ready()) {
    Firebase.RTDB.setInt(&fbdo, "/Readings/Moisture", receivedValue.toInt());
  }
}

void sendPumpState(char* state) {
  Serial.print("Pump State: ");
  Serial.println(state);
  strcpy(pumpState, state);
  Serial.println(pumpState);

  publishMessage("2022-CS-56/esp/pumpState", state);
  publishMessage("2022-CS-56/esp/pumpControl", state);
  sendLogMessage(String(formattedTime) + " Pump Turned " + String(state));
  
  if (Firebase.ready()) {
    Firebase.RTDB.setString(&fbdo, "/States/Pump", state);
  }
}

void writeToThingSpeak(int level) {
  ThingSpeak.writeField(CHANNEL_ID, 1, level, THINGSPEAK_API_KEY);
}

void callback(char* topic, byte* payload, unsigned int length) {
  String topicStr = topic;
  String payloadStr(reinterpret_cast<char*>(payload), length);
  Serial.println(topicStr);
  Serial.println(topicStr == "2022-CS-56/esp/modeIndicator");
  Serial.println(payloadStr);
  Serial.println(payloadStr == "Manual");
  if (topicStr == "2022-CS-56/esp/pumpControl") {
    if (mode == "Sensor" && payloadStr != pumpState) {
      publishMessage("2022-CS-56/esp/pumpControl", pumpState);
      sendLogMessage("System in Sensor Mode: Cannot Toogle Pump");
    } else if (payloadStr == "ON") {
      SerialPort.print('3');
    } else if (payloadStr == "OFF") {
      SerialPort.print('4');
    }
  } else if (topicStr == "2022-CS-56/esp/modeIndicator") {
    Serial.println(mode);
    if (mode != payloadStr) {
      if (payloadStr == "Manual") {
        SerialPort.print('1');
        Serial.println('1');
      } else {
        SerialPort.print('2');
        Serial.println('2');
      }
    }
  }
}

void loop() {
  client.loop();
  if (SerialPort.available() > 0) {
    receivedValue = SerialPort.readStringUntil('\n');
    receivedValue.trim();
    receivedValue.toCharArray(receivedChars, 10);

    if ( !isDigit(receivedValue.charAt(0)) ) {
        if (receivedValue == "Manual" || receivedValue == "Sensor") {
          mode = receivedValue;
          if (Firebase.ready()) {
            Firebase.RTDB.setString(&fbdo, "/States/Mode", receivedChars);
          }
        } else {
          updateFormattedDate();
          sendPumpState(receivedChars);
        }
    } else {
      if (receivedValue != lastReceivedValue) {
        updateFormattedDate();
        sendMoistureLevel(receivedChars);
        writeToThingSpeak(receivedValue.toInt());
        lastReceivedValue = receivedValue;
      }
    }
  }
}
