#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// WiFi credentials
#define WLAN_SSID       "KPL_TECH"
#define WLAN_PASS       "999999999"

// Adafruit MQTT configuration
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "robotanh"
#define AIO_KEY         "aio_DcoT58uRbG9c6MIVAlfjTLu1nDcT"

// MQTT client
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// MQTT topics for publishing
Adafruit_MQTT_Publish voltage_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/voltage");
Adafruit_MQTT_Publish current_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/current");
Adafruit_MQTT_Publish light_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/light");
Adafruit_MQTT_Publish pot_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/potentiometer");
Adafruit_MQTT_Publish temp_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");

// Subscription setup
Adafruit_MQTT_Subscribe light_sub = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/led", MQTT_QOS_1);

int led_counter = 0;
int led_status = HIGH;

// Callback function for light subscription
void lightcallback(char* value, uint16_t len) {
  if (value[0] == '0') Serial.print('a');
  if (value[0] == '1') Serial.print('A');
}

// UART processing function
void handleUART() {
  // Check if UART data is available
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // Read the complete line
    input.trim(); // Remove any trailing whitespace

    // Parse and publish data based on attribute tags
    if (input.startsWith("!VOLTAGE:")) {
      String voltage = input.substring(9, input.length() - 1); // Extract value
      voltage_pub.publish(voltage.c_str()); // Publish to voltage topic
    } else if (input.startsWith("!CURRENT:")) {
      String current = input.substring(9, input.length() - 1); // Extract value
      current_pub.publish(current.c_str()); // Publish to current topic
    } else if (input.startsWith("!LIGHT:")) {
      String light = input.substring(7, input.length() - 1); // Extract value
      light_pub.publish(light.c_str()); // Publish to light topic
    } else if (input.startsWith("!POTENTIOMETER:")) {
      String pot = input.substring(15, input.length() - 1); // Extract value
      pot_pub.publish(pot.c_str()); // Publish to potentiometer topic
    } else if (input.startsWith("!TEMPERATURE:")) {
      String temp = input.substring(13, input.length() - 1); // Extract value
      temp_pub.publish(temp.c_str()); // Publish to temperature topic
    } else {
      // Unknown input
      Serial.println("Unknown UART message received");
    }
  }
}

void setup() {
  // Set pin modes
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH); // Set busy pin HIGH

  // Initialize Serial and disable debug output
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  // Connect to WiFi
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // Subscribe to light feed
  light_sub.setCallback(lightcallback);
  mqtt.subscribe(&light_sub);

  // Connect to MQTT
  while (mqtt.connect() != 0) { 
    mqtt.disconnect();
    delay(500);
  }

  digitalWrite(5, LOW); // Set busy pin LOW after setup
}

void loop() {
  // Process MQTT packets
  mqtt.processPackets(1);

  // Handle UART input
  handleUART();

  // LED toggling logic
  led_counter++;
  if (led_counter == 100) {
    led_counter = 0;
    led_status = !led_status; // Toggle LED status
    digitalWrite(2, led_status);
  }
  delay(10);
}
