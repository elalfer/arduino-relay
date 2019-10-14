#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <vector>


// How much time for press emulation (ms)
#define PRESS_TIME 500

/// Network config
#define RECONNECT_DELAY 1000

#define PRODUCTION
#ifndef PRODUCTION

  // Device specific config
  char ssid[] = "MyWifi";               // your network SSID (name)
  char pass[] = "WifiPassword";        // your network password

  // MQTT server config
  const char* mqttServer = "192.168.0.1";     // MQTT server domain name or IP address
  const int   mqttPort = 1883;                    // MQTT server port
  const char* mqttUsername = "";   // Optional, leave blank if not required
  const char* mqttPassword = "";       // Optional, leave blank if not required
  const char* mqttClientName = "relay";
  #define NAME mydevice
#else
  #include "network_config.h"
#endif


#define GET_TOPIC(PREFIX,ACTION) "relay_" #PREFIX "/" #ACTION
const char* mqttFeedbackTopic  = GET_TOPIC(NAME, Get);
const char* mqttSubscribeTopic = GET_TOPIC(NAME, Set);

unsigned long mqttPreviousTime;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

static const std::vector<uint8_t> pins = {D0, D1, D5, D6};

bool mqttConnect() {
  while(!mqtt.connected()) {
    if (mqtt.connect(mqttClientName, mqttUsername, mqttPassword)) {
      Serial.print(F("MQTT connected: "));
      Serial.println(mqttServer);
      mqtt.subscribe(mqttSubscribeTopic);
    }
    else {
      Serial.print(F("MQTT connection failed: "));
      Serial.print(mqtt.state());
      Serial.println();
      Serial.println(mqttServer);
      delay(RECONNECT_DELAY);
    }
  }
  return mqtt.connected();
}

// Handles messages received in the mqttSubscribeTopic
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Handles unused parameters
  (void)topic;
  (void)length;
  payload[length] = 0;

  Serial.printf("topic: %s, payload: %s\n", topic, payload);

  if (strncmp((char *)payload, "ON", 2) == 0)
    digitalWrite(pins[atoi((const char *)(payload)+2)], LOW);
  if (strncmp((char *)payload, "OFF", 3) == 0)
    digitalWrite(pins[atoi((const char *)(payload)+3)], HIGH);
  if (strncmp((char *)payload,"BLINK", 5) == 0)
  {
    int pin_id = atoi((const char *)(payload)+5);
    digitalWrite(pins[pin_id], LOW);
    delay(PRESS_TIME);
    digitalWrite(pins[pin_id], HIGH);

    char tmp_str[32];
    memset(tmp_str, 0 , 32);
    sprintf(tmp_str, "OFF%d", pin_id);

    mqtt.publish(mqttFeedbackTopic, tmp_str, true);
  }
}

void mqttHandle() {
  if (!mqtt.connected()) {
    unsigned long mqttCurrentTime = millis();
    if (mqttCurrentTime - mqttPreviousTime > 5000) {
      mqttPreviousTime = mqttCurrentTime;
      if (mqttConnect()) {
        Serial.println(F("MQTT disconnected, successfully reconnected."));
        mqttPreviousTime = 0;
      }
      else Serial.println(F("MQTT disconnected, failed to reconnect."));
    }
  }
  else mqtt.loop();
}

void setup() {

  // Setup pins
  for(auto&& pin : pins)
  {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }

  Serial.begin(115200);
  Serial.print("init done!\n");

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(RECONNECT_DELAY);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  /// Connect to MQTT
  mqtt.setServer(mqttServer, mqttPort);
  mqtt.setCallback(mqttCallback);
  if (mqttConnect())
    mqttPreviousTime = millis();
  else 
    mqttPreviousTime = 0;
    
}

void loop() {
  // Handle MQTT messages
  mqttHandle();
}