#include <SPI.h>
#include <Ethernet2.h>
#include <PubSubClient.h>  // MQTT 
#include <PololuLedStrip.h>
#include <avr/wdt.h>
#include <TextFinder.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>

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
#define MEM_R 52
#define MEM_G 53
#define MEM_B 54

PololuLedStrip<3> ledStrip;
rgb_color colors[LED_COUNT];
byte mac[]    = { 0xD0, 0x11, 0x01, 0x41, 0x8A, 0x13 }; //MAC адрес контроллера
byte mqtt_serv[] = {192, 168, 1, 190}; //IP MQTT брокера

const char htmlx0[] PROGMEM = "<html><title>Controller IO setup Page</title><body marginwidth=\"0\" marginheight=\"0\" ";
const char htmlx1[] PROGMEM = "leftmargin=\"0\" \"><table bgcolor=\"#999999\" border";
const char htmlx2[] PROGMEM = "=\"0\" width=\"100%\" cellpadding=\"1\" ";
const char htmlx3[] PROGMEM = "\"><tr><td>&nbsp Controller IO setup Page</td></tr></table><br>";
const char* const string_table0[] PROGMEM = {htmlx0, htmlx1, htmlx2, htmlx3};

const char htmla0[] PROGMEM = "<script>function hex2num (s_hex) {eval(\"var n_num=0X\" + s_hex);return n_num;}";
const char htmla1[] PROGMEM = "</script><table><form><input type=\"hidden\" name=\"SBM\" value=\"1\"><tr><td>MAC:&nbsp&nbsp&nbsp";
const char htmla2[] PROGMEM = "<input id=\"T1\" type=\"text\" size=\"1\" maxlength=\"2\" name=\"DT1\" value=\"";
const char htmla3[] PROGMEM = "\">.<input id=\"T3\" type=\"text\" size=\"1\" maxlength=\"2\" name=\"DT2\" value=\"";
const char htmla4[] PROGMEM = "\">.<input id=\"T5\" type=\"text\" size=\"1\" maxlength=\"2\" name=\"DT3\" value=\"";
const char htmla5[] PROGMEM = "\">.<input id=\"T7\" type=\"text\" size=\"1\" maxlength=\"2\" name=\"DT4\" value=\"";
const char htmla6[] PROGMEM = "\">.<input id=\"T9\" type=\"text\" size=\"1\" maxlength=\"2\" name=\"DT5\" value=\"";
const char htmla7[] PROGMEM = "\">.<input id=\"T11\" type=\"text\" size=\"1\" maxlength=\"2\" name=\"DT6\" value=\"";
const char* const string_table1[] PROGMEM = {htmla0, htmla1, htmla2, htmla3, htmla4, htmla5, htmla6, htmla7};

const char htmlb0[] PROGMEM = "\"><input id=\"T2\" type=\"hidden\" name=\"DT1\"><input id=\"T4\" type=\"hidden\" name=\"DT2";
const char htmlb1[] PROGMEM = "\"><input id=\"T6\" type=\"hidden\" name=\"DT3\"><input id=\"T8\" type=\"hidden\" name=\"DT4";
const char htmlb2[] PROGMEM = "\"><input id=\"T10\" type=\"hidden\" name=\"DT5\"><input id=\"T12\" type=\"hidden\" name=\"D";
const char htmlb3[] PROGMEM = "T6\"></td></tr><tr><td>MQTT: <input type=\"text\" size=\"1\" maxlength=\"3\" name=\"DT7\" value=\"";
const char htmlb4[] PROGMEM = "\">.<input type=\"text\" size=\"1\" maxlength=\"3\" name=\"DT8\" value=\"";
const char htmlb5[] PROGMEM = "\">.<input type=\"text\" size=\"1\" maxlength=\"3\" name=\"DT9\" value=\"";
const char htmlb6[] PROGMEM = "\">.<input type=\"text\" size=\"1\" maxlength=\"3\" name=\"DT10\" value=\"";
const char* const string_table2[] PROGMEM = {htmlb0, htmlb1, htmlb2, htmlb3, htmlb4, htmlb5, htmlb6};

const char htmlc0[] PROGMEM = "\"></td></tr><tr><td><br></td></tr><tr><td><input id=\"button1\"type=\"submit\" value=\"SAVE\" ";
const char htmlc1[] PROGMEM = "></td></tr></form></table></body></html>";
const char* const string_table3[] PROGMEM = {htmlc0, htmlc1};

const char htmld0[] PROGMEM = "Onclick=\"document.getElementById('T2').value ";
const char htmld1[] PROGMEM = "= hex2num(document.getElementById('T1').value);";
const char htmld2[] PROGMEM = "document.getElementById('T4').value = hex2num(document.getElementById('T3').value);";
const char htmld3[] PROGMEM = "document.getElementById('T6').value = hex2num(document.getElementById('T5').value);";
const char htmld4[] PROGMEM = "document.getElementById('T8').value = hex2num(document.getElementById('T7').value);";
const char htmld5[] PROGMEM = "document.getElementById('T10').value = hex2num(document.getElementById('T9').value);";
const char htmld6[] PROGMEM = "document.getElementById('T12').value = hex2num(document.getElementById('T11').value);\"";
const char* const string_table4[] PROGMEM = {htmld0, htmld1, htmld2, htmld3, htmld4, htmld5, htmld6};

////////////////////////////////////////////////////////////////////////////
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String strTopic = String(topic);
  String strPayload = String((char*)payload);
  callback_iobroker(strTopic, strPayload);
}
////////////////////////////////////////////////////////////////////////////
EthernetClient ethClient;
EthernetServer http_server(80);
PubSubClient mqtt(ethClient);

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
const byte ID = 0x91;
char buffer[100];

const int start_DI_pin [] = {8, A1, A0}; // Порты ВВОДА
int n_DI_pin = sizeof(start_DI_pin) / sizeof(start_DI_pin[0]) - 1; //Вычисляем длинну массива
const int start_DO_pin [] = {9, 7, 6, 5, 3}; //Порты ВЫВОДА
int n_DO_pin = sizeof(start_DO_pin) / sizeof(start_DO_pin[0]) - 1; //Вычисляем длинну массива

///////////////////////////////////////////////////////////////////////////

void setup() {
  MCUSR = 0;
  wdt_disable();
  if (EEPROM.read(MEM_R) == 255 && EEPROM.read(MEM_G) == 255 && EEPROM.read(MEM_B) == 255) {
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

  digitalWrite(EMUL, LOW);
  digitalWrite(OFF, HIGH);
  digitalWrite(OPEN, HIGH);
  digitalWrite(UP, HIGH);

  httpSetup();
  mqttSetup();
  delay(1000);
  if (mqtt.connect(ID_CONNECT)) {
    PubTopic();
    mqtt.publish("myhome/Domofon/connection", "true");
    mqtt.subscribe("myhome/Domofon/#");
  }
  wdt_enable(WDTO_8S);
}
/////////////////////////////////////////////////////////////////////////
void loop() {
  wdt_reset();
  checkHttp();
  mqtt.loop();
  if (millis() - prevMillis3 > 10000 && flag_ring) {
    flag_ring = false;
    LedDefault();
  }
  if (flag_ring) {
    uint16_t time = millis() >> 2;
    for (uint16_t i = 0; i < LED_COUNT; i++) {
      byte x = (time >> 2) - (i << 3);
      colors[i] = hsvToRgb((uint32_t)x * 359 / 256, 255, 255);
    }
    ledStrip.write(colors, LED_COUNT);
  }

  fotoValue = analogRead(FOTO_PIN);
  if (digitalRead(RING) == LOW && !flag1 && (millis() - prevMillis5 > 5000)) { // Звонок домофона
    prevMillis5 = millis();
    mqtt.publish("myhome/Domofon/Ring", "true");
    flag1 = true;
  }
  else if (digitalRead(RING) == HIGH && flag1 && (millis() - prevMillis5 > 5000)) { // Звонок домофона
    mqtt.publish("myhome/Domofon/Ring", "false");
    flag1 = false;
  }
  if (digitalRead(RING_BTN) == LOW && !flag2 && (millis() - prevMillis4 > 10000) && ButtonOn) { // Звонок
    prevMillis4 = millis();
    mqtt.publish("myhome/Domofon/Doorbell", "true");
    flag2 = true;
    prevMillis3 = millis();
    flag_ring = true;
  }
  else if (digitalRead(RING_BTN) == HIGH && flag2 && millis() - prevMillis4 > 1000) { // Звонок
    mqtt.publish("myhome/Domofon/Doorbell", "false");
    flag2 = false;
  }
  if (millis() - prevMillis2 > 5000) {
    prevMillis2 = millis();
    mqtt.publish("myhome/Domofon/Lux", IntToChar(fotoValue));
  }

  if (!mqtt.connected()) {
    if (millis() - prevMillis > 5000) {
      prevMillis = millis();
      if (Ethernet.begin(mac) == 0) {
        Reset();
      } else {
        reconnect();
      }
    }
  }
}

void PubTopic() {
  char s[16];
  sprintf(s, "%d.%d.%d.%d", Ethernet.localIP()[0], Ethernet.localIP()[1], Ethernet.localIP()[2], Ethernet.localIP()[3]);
  mqtt.publish("myhome/Domofon/ip", s);
  mqtt.publish("myhome/Domofon/Ring", "false");
  mqtt.publish("myhome/Domofon/Open", "false");
  mqtt.publish("myhome/Domofon/Lux", IntToChar(fotoValue));
  mqtt.publish("myhome/Domofon/Emulation", "false");
  mqtt.publish("myhome/Domofon/Off", "false");
  mqtt.publish("myhome/Domofon/Doorbell", "false");
  mqtt.publish("myhome/Domofon/Led", "");
  mqtt.publish("myhome/Domofon/LedDef", "");
  mqtt.publish("myhome/Domofon/UP", false);
  mqtt.publish("myhome/Domofon/ButtonOff", "false");
}

void reconnect() {
  count++;
  if (mqtt.connect(ID_CONNECT)) {
    count = 0;
    wdt_reset();
    mqtt.publish("myhome/Domofon/connection", "true");
    mqtt.subscribe("myhome/Domofon/#");
  }
  if (count > 10) {
    wdt_enable(WDTO_15MS);
    for (;;) {}
  }
}


void callback_iobroker(String strTopic, String strPayload) {
  if (strTopic == "myhome/Domofon/Open") {
    digitalWrite(UP, LOW);
    delay(1000);
    digitalWrite(OPEN, LOW);
    delay(1500);
    digitalWrite(OPEN, HIGH);
    delay(500);
    digitalWrite(UP, HIGH);
    mqtt.publish("myhome/Domofon/Open", "false");
  }
  else if (strTopic == "myhome/Domofon/UP") {
    if (strPayload == "true") {
      digitalWrite(UP, LOW);
      mqtt.publish("myhome/Domofon/UP", "true");
    } else {
      digitalWrite(UP, HIGH);
      mqtt.publish("myhome/Domofon/UP", "false");
    }
  }
  else if (strTopic == "myhome/Domofon/Off") {
    if (strPayload == "true") {
      digitalWrite(OFF, LOW);
      mqtt.publish("myhome/Domofon/Off", "true");
    } else {
      digitalWrite(OFF, HIGH);
      mqtt.publish("myhome/Domofon/Off", "false");
    }
  }
  else if (strTopic == "myhome/Domofon/Emulation") {
    if (strPayload == "true") {
      digitalWrite(OFF, LOW);
      digitalWrite(EMUL, HIGH);
      mqtt.publish("myhome/Domofon/Emulation", "true");
    } else {
      digitalWrite(EMUL, LOW);
      digitalWrite(OFF, HIGH);
      mqtt.publish("myhome/Domofon/Emulation", "false");
    }
  }
  else if (strTopic == "myhome/Domofon/ButtonOff") {
    if (strPayload == "true") {
      ButtonOn = false;
      mqtt.publish("myhome/Domofon/ButtonOff", "true");
    } else {
      ButtonOn = true;
      mqtt.publish("myhome/Domofon/ButtonOff", "false");
    }
  }
  else if (strTopic == "myhome/Domofon/Led") {
    int R = strPayload.substring(1, strPayload.indexOf('G')).toInt();
    int G = strPayload.substring(strPayload.indexOf('G') + 1, strPayload.lastIndexOf('B')).toInt();
    int B = strPayload.substring(strPayload.lastIndexOf('B') + 1).toInt();
    SetRGB(R, G, B);
    //mqtt.publish("myhome/Domofon/Led", strPayload);
  }
  else if (strTopic == "myhome/Domofon/LedDef") {
    int R = strPayload.substring(1, strPayload.indexOf('G')).toInt();
    int G = strPayload.substring(strPayload.indexOf('G') + 1, strPayload.lastIndexOf('B')).toInt();
    int B = strPayload.substring(strPayload.lastIndexOf('B') + 1).toInt();
    EEPROM.write(MEM_R, byte(R));
    EEPROM.write(MEM_G, byte(G));
    EEPROM.write(MEM_B, byte(B));
    mqtt.publish("myhome/Domofon/Led", "OK");
    LedDefault();
  }
}

void SetRGB(int R, int G, int B) {
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

rgb_color hsvToRgb(uint16_t h, uint8_t s, uint8_t v) {
  uint8_t f = (h % 60) * 255 / 60;
  uint8_t p = (255 - s) * (uint16_t)v / 255;
  uint8_t q = (255 - f * (uint16_t)s / 255) * (uint16_t)v / 255;
  uint8_t t = (255 - (255 - f) * (uint16_t)s / 255) * (uint16_t)v / 255;
  uint8_t r = 0, g = 0, b = 0;
  switch ((h / 60) % 6) {
    case 0: r = v; g = t; b = p; break;
    case 1: r = q; g = v; b = p; break;
    case 2: r = p; g = v; b = t; break;
    case 3: r = p; g = q; b = v; break;
    case 4: r = t; g = p; b = v; break;
    case 5: r = v; g = p; b = q; break;
  }
  return rgb_color(r / 2, g / 2, b / 2);
}

void mqttSetup() {
  int idcheck = EEPROM.read(0);
  if (idcheck == ID) {
    for (int i = 0; i < 4; i++) {
      mqtt_serv[i] = EEPROM.read(i + 7);
    }
  }
  mqtt.setServer(mqtt_serv, 1883);
  mqtt.setCallback(callback);
}

void httpSetup() {
  int idcheck = EEPROM.read(0);
  if (idcheck == ID) {
    for (int i = 0; i < 6; i++) {
      mac[i] = EEPROM.read(i + 1);
    }
  }
  Ethernet.begin(mac);
}

void checkHttp() {
  EthernetClient http = http_server.available();
  if (http) {
    TextFinder  finder(http );
    while (http.connected()) {
      if (http.available()) {
        if ( finder.find("GET /") ) {
          if (finder.findUntil("setup", "\n\r")) {
            if (finder.findUntil("SBM", "\n\r")) {
              byte SET = finder.getValue();
              while (finder.findUntil("DT", "\n\r")) {
                int val = finder.getValue();
                if (val >= 1 && val <= 6) {
                  mac[val - 1] = finder.getValue();
                }
                if (val >= 7 && val <= 10) {
                  mqtt_serv[val - 7] = finder.getValue();
                }
              }
              for (int i = 0 ; i < 6; i++) {
                EEPROM.write(i + 1, mac[i]);
              }
              for (int i = 0 ; i < 4; i++) {
                EEPROM.write(i + 7, mqtt_serv[i]);
              }
              EEPROM.write(0, ID);
              http.println("HTTP/1.1 200 OK");
              http.println("Content-Type: text/html");
              http.println();
              for (int i = 0; i < 4; i++) {
                strcpy_P(buffer, (char*)pgm_read_word(&(string_table0[i])));
                http.print( buffer );
              }
              http.println();
              http.print("Saved!");
              http.println();
              http.print("Restart");
              for (int i = 1; i < 10; i++) {
                http.print(".");
                delay(500);
              }
              http.println("OK");
              Reset(); // ребутим с новыми параметрами
            }
            http.println("HTTP/1.1 200 OK");
            http.println("Content-Type: text/html");
            http.println();
            for (int i = 0; i < 4; i++) {
              strcpy_P(buffer, (char*)pgm_read_word(&(string_table0[i])));
              http.print( buffer );
            }
            for (int i = 0; i < 3; i++) {
              strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[i])));
              http.print( buffer );
            }
            http.print(mac[0], HEX);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[3])));
            http.print( buffer );
            http.print(mac[1], HEX);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[4])));
            http.print( buffer );
            http.print(mac[2], HEX);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[5])));
            http.print( buffer );
            http.print(mac[3], HEX);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[6])));
            http.print( buffer );
            http.print(mac[4], HEX);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[7])));
            http.print( buffer );
            http.print(mac[5], HEX);
            for (int i = 0; i < 4; i++) {
              strcpy_P(buffer, (char*)pgm_read_word(&(string_table2[i])));
              http.print( buffer );
            }
            http.print(mqtt_serv[0], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table2[4])));
            http.print( buffer );
            http.print(mqtt_serv[1], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table2[5])));
            http.print( buffer );
            http.print(mqtt_serv[2], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table2[6])));
            http.print( buffer );
            http.print(mqtt_serv[3], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table3[0])));
            http.print( buffer );
            for (int i = 0; i < 7; i++) {
              strcpy_P(buffer, (char*)pgm_read_word(&(string_table4[i])));
              http.print( buffer );
            }
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table3[1])));
            http.print( buffer );
            break;
          }
        }
        http.println("HTTP/1.1 200 OK");
        http.println("Content-Type: text/html");
        http.println();
        http.print("IOT controller [");
        http.print(ID_CONNECT);
        http.print("]: go to <a href=\"/setup\"> setup</a>");
        break;
      }
    }
    delay(1);
    http.stop();
  } else {
    return;
  }
}

void Reset() {
  for (;;) {}
}
