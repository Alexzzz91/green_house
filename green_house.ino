#include <Arduino.h>
#if defined(ESP8266)
  /* ESP8266 Dependencies */
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
  #include <ESPAsyncWebServer.h>
#elif defined(ESP32)
  /* ESP32 Dependencies */
  #include <WiFi.h>
  #include <AsyncTCP.h>
  #include <ESPAsyncWebServer.h>
#endif
#include <ESPDash.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

#define CLK 18
#define DT 19
#define SW 23

#include "GyverEncoder.h"
Encoder enc1(CLK, DT, SW);  // для работы c кнопкой

#define _LCD_TYPE 1  // для работы с I2C дисплеями
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4); // адрес 0x27 или 0x3f

#define DHTPIN 15     
#define PIN_MOSFET 25

#define DHTTYPE    DHT11

DHT dht(DHTPIN, DHTTYPE);

#include <microDS3231.h>
MicroDS3231 rtc;

/* Your SoftAP WiFi Credentials */
const char* ssid = "bel_green_house"; // SSID
const char* password = "Bel559865"; // Password

const int lightSensorPin = 39; // освещение
int lightSensorValue = 0;
int readGroundState = LOW;      // подача питания на пин мосфета
const long interval = 5000;  // интервал чтения данных с датчиков влажности земли (в миллисекундах)

unsigned long previousMillis = 0;  
unsigned long currentMillis = 0;   

const int groundSensor1Pin = 33;
int groundSensor1Value = 0;
const int groundSensor2Pin = 34;
int groundSensor2Value = 0;
const int groundSensor3Pin = 35;
int groundSensor3Value = 0;

//
const int motor1Pin1 = 13;
const int motor1Pin2 = 12;

const int motor2Pin1 = 14;
const int motor2Pin2 = 27;

const int selenoid1 = 5;
const int selenoid2 = 17;
const int selenoid3 = 16;
const int lampPin = 4;

// значения для реле
boolean motor1Value1 = 0;
boolean motor1Value2 = 0;

boolean motor2Value1 = 0;
boolean motor2Value2 = 0;

boolean selenoid1Value = 0;
boolean selenoid2Value = 0;
boolean selenoid3Value = 0;
boolean lampValue = 0;

// слайдеры для реле
int normalTemperature = 26;
int normalGroundHumiditySlider1 = 40;
int normalGroundHumiditySlider2 = 40;
int normalGroundHumiditySlider3 = 40;

// для меню
// переменные
int val1 = 0;
int val2 = 0;
int val3 = 0;
int val4 = 0;

int8_t arrowPos = 0;  // позиция стрелки

// баттарея
const int batteryPin = 36;
int batteryValue = 0;

/* Start Webserver */
AsyncWebServer server(80);

/* Attach ESP-DASH to AsyncWebServer */
ESPDash dashboard(&server);

/* 
  Dashboard Cards 
  Format - (Dashboard Instance, Card Type, Card Name, Card Symbol(optional) )
*/
Card techTemperature(&dashboard, TEMPERATURE_CARD, "Температура в корпусе", "°C");
Card temperature(&dashboard, TEMPERATURE_CARD, "Температура в теплице", "°C");
Card humidity(&dashboard, HUMIDITY_CARD, "Влажность", "%");
Card lightSensor(&dashboard, GENERIC_CARD, "Датчик света", "%");

Card groundSensor1(&dashboard, HUMIDITY_CARD, "Грядка 1", "%");
Card groundSensor2(&dashboard, HUMIDITY_CARD, "Грядка 2", "%");
Card groundSensor3(&dashboard, HUMIDITY_CARD, "Грядка 3", "%");

Card batteryCard(&dashboard, GENERIC_CARD, "Заряд баттареи", "%");

/* 
  Button Card
  Format - (Dashboard Instance, Card Type, Card Name)
*/
Card rele1Control(&dashboard, BUTTON_CARD, "Реле 1");
Card rele2Control(&dashboard, BUTTON_CARD, "Реле 2");
Card rele3Control(&dashboard, BUTTON_CARD, "Реле 3");
Card rele4Control(&dashboard, BUTTON_CARD, "Реле 4");
Card rele5Control(&dashboard, BUTTON_CARD, "Реле 5");
Card rele6Control(&dashboard, BUTTON_CARD, "Реле 6");
Card rele7Control(&dashboard, BUTTON_CARD, "Реле 7");
Card rele8Control(&dashboard, BUTTON_CARD, "Реле 8");

Card temperatureSlider(&dashboard, SLIDER_CARD, "Поддерживать температуру", "", 0, 60);
Card groundHumiditySlider1(&dashboard, SLIDER_CARD, "Влажность земли1 для полива", "", 0, 100);
Card groundHumiditySlider2(&dashboard, SLIDER_CARD, "Влажность земли2 для полива", "", 0, 100);
Card groundHumiditySlider3(&dashboard, SLIDER_CARD, "Влажность земли3 для полива", "", 0, 100);

void setup() {
  Serial.begin(115200);

  // раскоментируй, чтобы записать в часы время
  // rtc.setTime(COMPILE_TIME);
  
  /* Start Access Point */
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, password);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  rele1Control.attachCallback([&](bool value){
    motor1Value1 = value;
    rele1Control.update(motor1Value1);
    dashboard.sendUpdates();
  });

  rele2Control.attachCallback([&](bool value){
    motor1Value2 = value;
    rele2Control.update(motor1Value2);
    dashboard.sendUpdates();
  });

  rele3Control.attachCallback([&](bool value){
    motor2Value1 = value;
    rele3Control.update(motor2Value1);
    dashboard.sendUpdates();
  });

  rele4Control.attachCallback([&](bool value){
    motor2Value2 = value;
    rele4Control.update(motor2Value2);
    dashboard.sendUpdates();
  });

  rele5Control.attachCallback([&](bool value){
    selenoid1Value = value;
    rele5Control.update(selenoid1Value);
    dashboard.sendUpdates();
  });

  rele6Control.attachCallback([&](bool value){
    selenoid2Value = value;
    rele6Control.update(selenoid2Value);
    dashboard.sendUpdates();
  });

  rele7Control.attachCallback([&](bool value){
    selenoid3Value = value;
    rele7Control.update(selenoid3Value);
    dashboard.sendUpdates();
  });

  rele8Control.attachCallback([&](bool value){
    lampValue = value;
    rele8Control.update(lampValue);
    dashboard.sendUpdates();
  });

  temperatureSlider.attachCallback([&](int value){
    normalTemperature = value;
    temperatureSlider.update(normalTemperature);
    dashboard.sendUpdates();
  });

  groundHumiditySlider1.attachCallback([&](int value){
    normalGroundHumiditySlider1 = value;
    groundHumiditySlider1.update(normalGroundHumiditySlider1);
    dashboard.sendUpdates();
  });

  groundHumiditySlider2.attachCallback([&](int value){
    normalGroundHumiditySlider2 = value;
    groundHumiditySlider2.update(normalGroundHumiditySlider2);
    dashboard.sendUpdates();
  });

  groundHumiditySlider3.attachCallback([&](int value){
    normalGroundHumiditySlider3 = value;
    groundHumiditySlider3.update(normalGroundHumiditySlider3);
    dashboard.sendUpdates();
  });
  
  /* Start AsyncWebServer */
  server.begin();
  dht.begin();
  pinMode(PIN_MOSFET, OUTPUT);
  
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(selenoid1, OUTPUT);
  pinMode(selenoid2, OUTPUT);
  pinMode(selenoid3, OUTPUT);
  pinMode(lampPin, OUTPUT);
  
  enc1.setType(TYPE2);

  lcd.init();
  lcd.backlight();
  printGUI();   // выводим интерфейс

}

void readLightSensor() {
  lightSensorValue = map(analogRead(lightSensorPin), 0, 4095, 0, 100);
//  Serial.print("lightSensorValue: ");
//  Serial.println(lightSensorValue);
  
  lightSensor.update((int)lightSensorValue);
}

void readDhtSensor() {
  float temperatureValue = dht.readTemperature();
  if (!isnan(temperatureValue)) {
    temperature.update((int)temperatureValue);
//    Serial.print("temperatureValue: ");
//    Serial.println(temperatureValue);
  }

  float humidityValue = dht.readHumidity();
  if (!isnan(humidityValue)) {
    humidity.update((int)humidityValue);
//    Serial.print("humidityValue: ");
//    Serial.println(humidityValue);
  }
}

void readGroundSensors() {
  currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (readGroundState == LOW) {
      readGroundState = HIGH;
    } else {
      readGroundState = LOW;
    }

    digitalWrite(PIN_MOSFET, readGroundState);
    
    if (readGroundState == HIGH) {
        groundSensor1Value = map(analogRead(groundSensor1Pin), 0, 4095, 0, 100);
        
        groundSensor1.update((int)groundSensor1Value);
//        Serial.print("groundSensor1Value: ");
//        Serial.println(groundSensor1Value);

        
        groundSensor2Value = map(analogRead(groundSensor2Pin), 0, 4095, 0, 100);
        
        groundSensor2.update((int)groundSensor2Value);
//        Serial.print("groundSensor2Value: ");
//        Serial.println(groundSensor2Value);
        
        groundSensor3Value = map(analogRead(groundSensor3Pin), 0, 4095, 0, 100);
        
        groundSensor3.update((int)groundSensor3Value);
//        Serial.print("groundSensor3Value: ");
//        Serial.println(groundSensor3Value);
    }
  } 
}

void readBatteryValue() {
  batteryValue = map(analogRead(batteryPin), 0, 4095, 0, 100);
//  Serial.print("batteryValue: ");
//  Serial.println(batteryValue);
  
  batteryCard.update((int)batteryValue);
}

void controlRele() {
  digitalWrite(motor1Pin1,motor1Value1);  // включить
  digitalWrite(motor1Pin2,motor1Value2);  // включить
  digitalWrite(motor2Pin1,motor2Value1);  // включить
  digitalWrite(motor2Pin2,motor2Value2);  // включить
  
  digitalWrite(selenoid1,selenoid1Value);  // включить
  digitalWrite(selenoid2,selenoid2Value);  // включить
  digitalWrite(selenoid3,selenoid3Value);  // включить
  digitalWrite(lampPin,lampValue);  // включить
}

void loop() {
  enc1.tick();

  if (enc1.isTurn()) {  // при любом повороте
    lcd.clear();        // очищаем дисплей

    if (enc1.isRight()) {
      arrowPos++;
      if (arrowPos >= 4) arrowPos = 3;  // ограничиваем позицию курсора
    }
    if (enc1.isLeft()) {
      arrowPos--;
      if (arrowPos < 0) arrowPos = 0;  // ограничиваем позицию курсора
    }

    // при нажатом повороте меняем переменные ++
    if (enc1.isRightH()) {
      switch (arrowPos) {
        case 0: val1++;
          break;
        case 1: val2++;
          break;
        case 2: val3++;
          break;
        case 3: val4++;
          break;
      }
    }

    // при нажатом повороте меняем переменные --
    if (enc1.isLeftH()) {
      switch (arrowPos) {
        case 0: val1--;
          break;
        case 1: val2--;
          break;
        case 2: val3--;
          break;
        case 3: val4--;
          break;
      }
    }
    
    printGUI();   // выводим интерфейс
  }  

    currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    readLightSensor();
    readDhtSensor();
    readGroundSensors();
    readBatteryValue();
    controlRele();
    
    /* Send Updtes to our Dashboard (realtime) */
    dashboard.sendUpdates();
    
    // выводим дату и время готовыми строками
//    Serial.println(rtc.getTimeString());
//    Serial.println(rtc.getDateString());
  
    techTemperature.update((int)rtc.getTemperatureFloat());

  } 
//  delay(300);
}


void printGUI() {
  lcd.setCursor(0, 0); lcd.print("val1:"); lcd.print(val1);
  lcd.setCursor(8, 0); lcd.print("val2:"); lcd.print(val2);
  lcd.setCursor(0, 1); lcd.print("val3:"); lcd.print(val3);
  lcd.setCursor(8, 1); lcd.print("val4:"); lcd.print(val4);
  // выводим стрелку
  switch (arrowPos) {
    case 0: lcd.setCursor(4, 0);
      break;
    case 1: lcd.setCursor(12, 0);
      break;
    case 2: lcd.setCursor(4, 1);
      break;
    case 3: lcd.setCursor(12, 1);
      break;
  }
  lcd.write(126);   // вывести стрелку
}
