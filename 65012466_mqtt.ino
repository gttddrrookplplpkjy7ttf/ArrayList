#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"

const char* ssid = "";
const char* password = "";

#define mqttServer "broker.emqx.io"
#define mqttPort 1883
#define mqttUser ""
#define mqttPassword ""

Adafruit_SHT31 sht31 = Adafruit_SHT31();

String msg;     //messge ที่รับค่าเข้ามาเฉยๆ
unsigned long myTime = 0;

#define ledRed 25
#define ledBlue 27
#define Relay01  32   // กำหนดขาที่ต่อกับ Relay 1
#define Relay02  14   // กำหนดขาที่ต่อกับ Relay 2


WiFiClient wificlient;
PubSubClient mqttClient(wificlient);

void setupRelays() {
  pinMode(Relay01, OUTPUT);
  pinMode(Relay02, OUTPUT);
  
  // ตั้งค่าเริ่มต้นให้ Relay ปิด (HIGH)
  digitalWrite(Relay01, HIGH);
  digitalWrite(Relay02, HIGH);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  
  pinMode(ledRed, OUTPUT);
  digitalWrite(ledRed, HIGH);     //OFF

  pinMode(ledBlue, OUTPUT);
  digitalWrite(ledBlue, HIGH);     //OFF

  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  setupRelays(); // เรียกใช้ฟังก์ชัน setupRelays() เพื่อกำหนดขา GPIO และตั้งค่า Relay

  setupMQTT();
}


void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  //boolean publish (topic,)
  if (millis() - myTime > 10000) {   //  10s = 10000. 22000-11000 > 10000  = =true
    readSht31();
    myTime = millis();
  }

  mqttClient.loop();
}

void readSht31() {
  float tempC = sht31.readTemperature();
  float humidity = sht31.readHumidity();
  
  String data = "Temperature: " + String(tempC) + "°C, Humidity: " + String(humidity) + "%";
  Serial.println(data);
  mqttClient.publish("CPE345IoT/65015916/msg/SHT31", data.c_str());
  

}

void setupMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  // set the callback function
  mqttClient.setCallback(callback);
}

void reconnect() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
    Serial.println("Reconnecting to MQTT Broker..");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword)) {
      Serial.println("Connected.");
      Serial.println(clientId);
      // subscribe to topic
      mqttClient.subscribe("CPE345IoT/65015916/msg/#");
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String sTopic = String(topic);
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    msg += (char)payload[i];
  }

  if(sTopic == "CPE345IoT/65015916/msg/Relay01") {
    // ON/OFF
    if(msg == "ON") {
      digitalWrite(Relay01, LOW);    //ON
      digitalWrite(ledRed, LOW);
    }
    else {
      digitalWrite(Relay01, HIGH);   //OFF
      digitalWrite(ledRed, HIGH);
    }
  }

  else if(sTopic == "CPE345IoT/65015916/msg/Relay02") {
    // ON/OFF
    if(msg == "ON") {
      digitalWrite(Relay02, LOW);    //ON
      digitalWrite(ledBlue, LOW);
      delay(5000);
    }
    else {
      digitalWrite(Relay02, HIGH);   //OFF
      digitalWrite(ledBlue, HIGH);
      delay(5000);
    }
  }
  msg = "";
  Serial.println("\n======================================");
}
