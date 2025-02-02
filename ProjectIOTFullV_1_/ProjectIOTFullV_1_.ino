#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#define DHTPIN 23          // ขาที่ใช้สำหรับ DHT11
#define DHTTYPE DHT11      // ใช้เซ็นเซอร์ DHT11
DHT dht(DHTPIN, DHTTYPE);  // เริ่มต้นใช้งาน DHT
#include <U8g2lib.h>
#include <Wire.h>  // IIC
#include "normal.h"
#include "cry.h"
#include "pumpon.h"
#include "thank.h"
#include "pumpoff.h"
#include "hugy.h"
#include "hello.h"
#include "full.h"


const int relayPin = 26;   // ขาที่ใช้สำหรับ relay
const int trigPin = 5;     // ขาสำหรับ Trigger ของ ultrasonic sensor
const int echoPin = 18;    // ขาสำหรับ Echo ของ ultrasonic sensor
const int buzzerPin = 33;
int sensorPin = 35;  // ขาสำหรับเซ็นเซอร์ความชื้นดิน (ต้องใช้ ADC พิน)

const int wet = 1530;  // ค่าความชื้นเมื่อดินเปียก
const int dry = 4095;  // ค่าความชื้นเมื่อดินแห้ง
long duration;         // ตัวแปรเวลาที่ใช้ในการสะท้อน
float distance;        // ตัวแปรระยะทางที่คำนวณจาก ultrasonic sensor
float humidity;
float temperature;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);  // เริ่มต้นการใช้งานสำหรับหน้าจอ OLED

const char* ssid = "gg";
const char* password = "rpki2336";
const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;
const char* mqtt_Client = "e6ad96e5-d7f4-4e23-a4f1-ddf052651f98";
const char* mqtt_username = "myXnbQ2fnU2ADLq5s6eddshk4jkV9igg";
const char* mqtt_password = "aLqs2ciB2rt7vdyhARBjV3sBSNaNUvg2";

int counter = 0;       // ตัวแปรสำหรับจัดการการแสดงภาพแบบ Animation
int soilMoisture = 0;  // ตัวแปรสำหรับเก็บค่าความชื้นของดิน

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
char msg[100];
String DataString;
bool pumpActive = false;

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqtt_Client, mqtt_username, mqtt_password)) {  // เชื่อมต่อกับ MQTT BROKER
      Serial.println("connected");
      client.subscribe("@msg/operator");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String message;
  for (int i = 0; i < length; i++) {
    message = message + char(payload[i]);
  }
  Serial.println(message);
  if (String(topic) == "@msg/operator") {
    if (message == "ON") {
      digitalWrite(relayPin, HIGH);  // Turn on relay
      Serial.println("Relay ON");
      u8g2.clearBuffer();                                           // clear ความจำไม่ให้ค้าง
      u8g2.drawXBMP(0, 0, 128, 64, epd_bitmap_allArray3[counter]);  // วาดเฟรม
      u8g2.sendBuffer();                                            // ถ่ายโอนความจำจอ oled
      counter = (counter + 1) % 2;                                  // เพิ่ม counter สำหรับ epd_bitmap_allArray
      delay(500);
    } else if (message == "OFF") {
      digitalWrite(relayPin, LOW);  // Turn off relay
      Serial.println("Relay OFF");
      u8g2.clearBuffer();                                           // clear ความจำไม่ให้ค้าง
      u8g2.drawXBMP(0, 0, 128, 64, epd_bitmap_allArray5[counter]);  // วาดเฟรม
      u8g2.sendBuffer();                                            // ถ่ายโอนความจำจอ oled
      counter = (counter + 1) % 2;                                  // เพิ่ม counter สำหรับ epd_bitmap_allArray
      delay(500);
    }
  }
}

void setup() {
  Serial.begin(115200);
  u8g2.begin();
  pinMode(relayPin, OUTPUT);  // ตั้งค่า relay ให้เป็น output
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(relayPin, HIGH);  // Relay OFF initially
  digitalWrite(buzzerPin, LOW);

  // ตั้งค่าขา Trig และ Echo ของ ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  dht.begin();


  // เชื่อมต่อกับ WIFI
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.subscribe("@msg/operator");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if (soilMoisture > 70) {
    // กลับไปยังหน้าเริ่มต้นหากค่าความชื้นมากกว่า 70%
    u8g2.clearBuffer();
    u8g2.drawXBMP(0, 0, 128, 64, epd_bitmap_allArray8[counter]);
    u8g2.sendBuffer();
    counter = (counter + 1) % 2;
    delay(500);

  } else if (soilMoisture < 40) {
    u8g2.clearBuffer();
    u8g2.drawXBMP(0, 0, 128, 64, epd_bitmap_allArray2[counter]);
    u8g2.sendBuffer();
    counter = (counter + 1) % 2;
    delay(500);

  } else if (soilMoisture >= 41 && soilMoisture <= 69) {
    u8g2.clearBuffer();
    u8g2.drawXBMP(0, 0, 128, 64, epd_bitmap_allArray1[counter]);
    u8g2.sendBuffer();
    counter = (counter + 1) % 4;
    delay(500);
  }

  // เริ่มต้นการทำงานของ ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // อ่านค่าเวลาที่ใช้ในการสะท้อน
  duration = pulseIn(echoPin, HIGH);
  distance = (duration * 0.0343) / 2;  // คำนวณระยะทาง



  // ตรวจสอบระยะทางและทำงานกับ buzzer
  if (distance < 10) {
    Serial.println("Buzzer ON");
    digitalWrite(buzzerPin, HIGH);
    tone(buzzerPin, 3000, 100);
    digitalWrite(buzzerPin, LOW);
    delay(100);
  } else {
    digitalWrite(buzzerPin, LOW);
  }

  // ตรวจสอบระยะทาง
  if (distance < 10) {
    // แสดงภาพจาก epd_bitmap_allArray ถ้าระยะทางน้อยกว่า 10 cm
    u8g2.clearBuffer();
    u8g2.drawXBMP(0, 0, 128, 64, epd_bitmap_allArray4[counter]);  // แสดง array4
    u8g2.sendBuffer();                                            // ถ่ายความจำไป oled
    counter = (counter + 1) % 2;                                  // เพิ่ม counter สำหรับ epd_bitmap_allArray
    delay(500);
  }

  long now = millis();
  if (now - lastMsg > 5000) {  // จับเวลาส่งข้อมูลทุก ๆ 5 วินาที
    lastMsg = now;
    int sensorValue = 0;  // ค่าที่อ่านได้จากเซ็นเซอร์ความชื้นดิน

    // อ่านค่าความชื้นในดินและแปลงเป็นเปอร์เซ็นต์
    sensorValue = analogRead(sensorPin);
    int moisturePercent = constrain(map(sensorValue, dry, wet, 0, 100), 0, 100);
    soilMoisture = moisturePercent;

   
    Serial.print("Raw soil moisture sensor value: ");  //ไว้ดูค่าดิบของเซนเซอร์วัดความชื้นในดินเผื่อสายแตกหักหลุดพัง
    Serial.println(sensorValue);

    Serial.print("sensor = ");
    Serial.println(sensorValue);
    Serial.print(" - percentage = ");
    Serial.println(moisturePercent);

    // อ่านค่าอุณหภูมิและความชื้น
    float h = dht.readHumidity();
    float t = dht.readTemperature();

 if (isnan(h) || isnan(t)) {
      Serial.println("การอ่านค่าจากเซ็นเซอร์ DHT ล้มเหลว!");
      return;  // ข้ามรอบการทำงานนี้
    }



    // เตรียมข้อมูลสำหรับการส่ง
    DataString = "{\"data\":{\"temperature\":" + (String)t + ",\"humidity\":" + (String)h + ",\"soil_moisture\":" + (String)moisturePercent + "}}";
    DataString.toCharArray(msg, 100);

    // พิมพ์ข้อมูลเพื่อแสดงใน Serial Monitor
    Serial.println("Hello PROJECT2024");
    Serial.print("Publish message: ");
    Serial.println(msg);

    // ส่งข้อมูลไปยัง MQTT
    client.publish("@shadow/data/update", msg);
  }
  delay(1);
}
