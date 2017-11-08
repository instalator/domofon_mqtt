#include <SPI.h>
#include <Ethernet2.h>
#include <PubSubClient.h>  // MQTT 
#include <PololuLedStrip.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#define ID_CONNECT   "domofon"
#define MOSFET    3  //Выход полевик
#define FOTO_PIN  A0 //Вход фоторезистора
#define RING_BTN  A1 //Вход кнопки звонка
#define OFF       5  // 7 Отключение трубки
#define EMUL      6  // 6 Эмуляция трубки
#define OPEN      7  // 5 Открыть
#define UP        9  // 4 Поднятие трубки
#define RING      8  // 3 Вход сигнала вызова
#define LED_COUNT 4
#define MEM_R 2
#define MEM_G 3
#define MEM_B 4

PololuLedStrip<3> ledStrip;
rgb_color colors[LED_COUNT];
byte mac[]    = { 0xD0, 0x11, 0x01, 0x41, 0x8A, 0x13 };
byte server[] = { 192, 168, 1, 190 }; //IP Брокера
byte ip[]     = { 192, 168, 1, 60 };  //IP Клиента

////////////////////////////////////////////////////////////////////////////
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String strTopic = String(topic);
  String strPayload = String((char*)payload);
  callback_iobroker(strTopic, strPayload);
}
////////////////////////////////////////////////////////////////////////////
EthernetClient ethClient;
PubSubClient client(server, 1883, callback, ethClient);

unsigned long prevMillis = 0; //для reconnect
unsigned long prevMillis2 = 0;
unsigned long prevMillis3 = millis();
unsigned long prevMillis4 = millis();
unsigned long prevMillis5 = millis();
int count = 0;
static char buf [100];
unsigned int fotoValue = analogRead(FOTO_PIN);
bool flag1 = false;
bool flag2 = false;
bool flag_ring = false;
bool ButtonOn = true;
rgb_color color;

const int start_DI_pin [] = {8, A1, A0}; // Порты ВВОДА
int n_DI_pin = sizeof(start_DI_pin) / sizeof(start_DI_pin[0]) - 1; //Вычисляем длинну массива
const int start_DO_pin [] = {9, 7, 6, 5, 3}; //Порты ВЫВОДА
int n_DO_pin = sizeof(start_DO_pin) / sizeof(start_DO_pin[0]) - 1; //Вычисляем длинну массива

///////////////////////////////////////////////////////////////////////////

void setup() {
  MCUSR = 0;
  wdt_disable();
  if(EEPROM.read(MEM_R) == 255 && EEPROM.read(MEM_G) == 255 && EEPROM.read(MEM_B) == 255){
    SetRGB(0, 0, 0);
  } else {
    LedDefault();
  }
  //Serial.begin(115200);
  for (int i = 0 ; i <= n_DI_pin; i++) {
    pinMode (start_DI_pin [i], INPUT);
  }
  for (int i = 0 ; i <= n_DO_pin; i++) {
    pinMode (start_DO_pin [i], OUTPUT);
  }

  Ethernet.begin(mac, ip);
  if (client.connect(ID_CONNECT)) {
    PubTopic();
    client.publish("myhome/Domofon/connection", "true");
    client.subscribe("myhome/Domofon/#");
  }
  wdt_enable(WDTO_8S);
}
/////////////////////////////////////////////////////////////////////////
void loop() {
  wdt_reset();
  client.loop();
  if (millis() - prevMillis3 > 10000 && flag_ring) {
    flag_ring = false;
    LedDefault();
  }
  if (flag_ring) {
    uint16_t time = millis() >> 2;
    for(uint16_t i = 0; i < LED_COUNT; i++){
      byte x = (time >> 2) - (i << 3);
      colors[i] = hsvToRgb((uint32_t)x * 359 / 256, 255, 255);
    }
    ledStrip.write(colors, LED_COUNT);
  }

  fotoValue = analogRead(FOTO_PIN);
  if (digitalRead(RING) == HIGH && !flag1 && (millis() - prevMillis5 > 5000)) { // Звонок домофона
    prevMillis5 = millis();
    client.publish("myhome/Domofon/Ring", "true");
    flag1 = true;
  }
  else if (digitalRead(RING) == LOW && flag1 && (millis() - prevMillis5 > 5000)) { // Звонок домофона
    client.publish("myhome/Domofon/Ring", "false");
    flag1 = false;

  }
  if (digitalRead(RING_BTN) == LOW && !flag2 && (millis() - prevMillis4 > 10000) && ButtonOn) { // Звонок
    prevMillis4 = millis();
    client.publish("myhome/Domofon/Doorbell", "true");
    flag2 = true;
    prevMillis3 = millis();
    flag_ring = true;
  }
  else if (digitalRead(RING_BTN) == HIGH && flag2 && millis() - prevMillis4 > 1000) { // Звонок
    client.publish("myhome/Domofon/Doorbell", "false");
    flag2 = false;
  }
  if (millis() - prevMillis2 > 5000) {
    prevMillis2 = millis();
    client.publish("myhome/Domofon/Lux", IntToChar(fotoValue));
  }

  if (!client.connected()) {
    if (millis() - prevMillis > 5000) {
      prevMillis = millis();
      reconnect();
    }
  }
}

void PubTopic() {
  client.publish("myhome/Domofon/Ring", "false");
  client.publish("myhome/Domofon/Open", "false");
  client.publish("myhome/Domofon/Lux", IntToChar(fotoValue));
  client.publish("myhome/Domofon/Emulation", "false");
  client.publish("myhome/Domofon/Off", "false");
  client.publish("myhome/Domofon/Doorbell", "false");
  client.publish("myhome/Domofon/Led", "");
  client.publish("myhome/Domofon/LedDef", "");
  client.publish("myhome/Domofon/ButtonOff", "false");
}

void reconnect() {
  count++;
  if (client.connect(ID_CONNECT)) {
    count = 0;
    wdt_reset();
    client.publish("myhome/Domofon/connection", "true");
    client.subscribe("myhome/Domofon/#");
  }
  if (count > 10) {
    wdt_enable(WDTO_15MS);
    for (;;) {}
  }
}


void callback_iobroker(String strTopic, String strPayload) {
  if (strTopic == "myhome/Domofon/Open") {
    digitalWrite(UP, HIGH);
    delay(1000);
    digitalWrite(OPEN, HIGH);
    delay(1500);
    digitalWrite(OPEN, LOW);
    delay(500);
    digitalWrite(UP, LOW);
    client.publish("myhome/Domofon/Open", "false");
  }
   else if (strTopic == "myhome/Domofon/Off") {
    if (strPayload == "true") {
      digitalWrite(OFF, HIGH);
      client.publish("myhome/Domofon/Off", "true");
    } else {
      digitalWrite(OFF, LOW);
      client.publish("myhome/Domofon/Off", "false");
    }
  }
  else if (strTopic == "myhome/Domofon/Emulation") {
    if (strPayload == "true") {
      digitalWrite(EMUL, HIGH);
      client.publish("myhome/Domofon/Emulation", "true");
    } else {
      digitalWrite(EMUL, LOW);
      client.publish("myhome/Domofon/Emulation", "false");
    }
  }
  else if (strTopic == "myhome/Domofon/ButtonOff") {
    if (strPayload == "true") {
      ButtonOn = false;
      client.publish("myhome/Domofon/ButtonOff", "true");
    } else {
      ButtonOn = true;
      client.publish("myhome/Domofon/ButtonOff", "false");
    }
  }
  else if (strTopic == "myhome/Domofon/Led") {
    int R = strPayload.substring(1, strPayload.indexOf('G')).toInt();
    int G = strPayload.substring(strPayload.indexOf('G') + 1, strPayload.lastIndexOf('B')).toInt();
    int B = strPayload.substring(strPayload.lastIndexOf('B') + 1).toInt();
    SetRGB(R, G, B);
    //client.publish("myhome/Domofon/Led", strPayload);
  }
  else if (strTopic == "myhome/Domofon/LedDef") {
    int R = strPayload.substring(1, strPayload.indexOf('G')).toInt();
    int G = strPayload.substring(strPayload.indexOf('G') + 1, strPayload.lastIndexOf('B')).toInt();
    int B = strPayload.substring(strPayload.lastIndexOf('B') + 1).toInt();
    EEPROM.write(MEM_R, byte(R));
    EEPROM.write(MEM_G, byte(G));
    EEPROM.write(MEM_B, byte(B));
    client.publish("myhome/Domofon/Led", "OK");
    LedDefault();
  }
}

void SetRGB(int R, int G, int B){
    rgb_color color;
    color.red = R;
    color.green = G;
    color.blue = B;
    for (uint16_t i = 0; i < LED_COUNT; i++) {
      colors[i] = color;
    }
    ledStrip.write(colors, LED_COUNT);
    return;
}

void LedDefault() {
  rgb_color color;
  color.red =   EEPROM.read(MEM_R);
  color.green = EEPROM.read(MEM_G);
  color.blue =  EEPROM.read(MEM_B);
  for (uint16_t i = 0; i < LED_COUNT; i++) {
    colors[i] = color;
  }
  ledStrip.write(colors, LED_COUNT);
}

const char* IntToChar (int v) {
  sprintf(buf, "%d", v);
  return buf;
}

const char* BoolToChar (bool r) {
  return r ? "true" : "false";
}

rgb_color hsvToRgb(uint16_t h, uint8_t s, uint8_t v){
    uint8_t f = (h % 60) * 255 / 60;
    uint8_t p = (255 - s) * (uint16_t)v / 255;
    uint8_t q = (255 - f * (uint16_t)s / 255) * (uint16_t)v / 255;
    uint8_t t = (255 - (255 - f) * (uint16_t)s / 255) * (uint16_t)v / 255;
    uint8_t r = 0, g = 0, b = 0;
    switch((h / 60) % 6){
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        case 5: r = v; g = p; b = q; break;
    }
    return rgb_color(r/2, g/2, b/2);
}
