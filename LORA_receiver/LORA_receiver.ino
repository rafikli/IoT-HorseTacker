#include <SPI.h>
#include <LoRa.h>
#ifdef ESP32
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#error Platform not supported
#endif
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson (use v6.xx)
#include <time.h>
#define emptyString String()
#include "secrets.h"

#if !(ARDUINOJSON_VERSION_MAJOR == 6 and ARDUINOJSON_VERSION_MINOR >= 7)
#error "Install ArduinoJson v6.7.0-beta or higher"
#endif

const int MQTT_PORT = 8883;
const char MQTT_PUB_TOPIC_PREFIX[] =  "Base_Station" "/Fazenda/Alvorada/Cavalo/Sandro";
const char MQTT_PUB_TOPIC_ANGULO[] =  "Base_Station" "/Fazenda/Alvorada/Cavalo/Sandro/Angulo";
const char MQTT_PUB_TOPIC_DILATACAO[] =  "Base_Station" "/Fazenda/Alvorada/Cavalo/Sandro/Dilatacao";

#ifdef USE_SUMMER_TIME_DST
uint8_t DST = 1;
#else
uint8_t DST = 0;
#endif

WiFiClientSecure net;

#ifdef ESP8266
BearSSL::X509List cert(cacert);
BearSSL::X509List client_crt(client_cert);
BearSSL::PrivateKey key(privkey);
#endif

PubSubClient client(net);

unsigned long lastMillis = 0;
time_t now;
time_t nowish = 1510592825;

//define the pins used by the transceiver module
#define ss 18
#define rst 23
#define dio0 26

char receivedMessage[20];  // Adjust the size based on your message length

void NTPConnect(void)
{
  Serial.print("Setting time using SNTP");
  configTime(TIME_ZONE * 3600, DST * 3600, "pool.ntp.org", "time.nist.gov");
  now = time(nullptr);
  while (now < nowish)
  {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("done!");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: ");
  Serial.print(asctime(&timeinfo));
}

void messageReceived(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Received [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void pubSubErr(int8_t MQTTErr)
{
  if (MQTTErr == MQTT_CONNECTION_TIMEOUT)
    Serial.print("Connection tiemout");
  else if (MQTTErr == MQTT_CONNECTION_LOST)
    Serial.print("Connection lost");
  else if (MQTTErr == MQTT_CONNECT_FAILED)
    Serial.print("Connect failed");
  else if (MQTTErr == MQTT_DISCONNECTED)
    Serial.print("Disconnected");
  else if (MQTTErr == MQTT_CONNECTED)
    Serial.print("Connected");
  else if (MQTTErr == MQTT_CONNECT_BAD_PROTOCOL)
    Serial.print("Connect bad protocol");
  else if (MQTTErr == MQTT_CONNECT_BAD_CLIENT_ID)
    Serial.print("Connect bad Client-ID");
  else if (MQTTErr == MQTT_CONNECT_UNAVAILABLE)
    Serial.print("Connect unavailable");
  else if (MQTTErr == MQTT_CONNECT_BAD_CREDENTIALS)
    Serial.print("Connect bad credentials");
  else if (MQTTErr == MQTT_CONNECT_UNAUTHORIZED)
    Serial.print("Connect unauthorized");
}

void connectToMqtt(bool nonBlocking = false)
{
  Serial.print("MQTT connecting ");
  while (!client.connected())
  {
    if (client.connect("Base_Station"))
    {
      Serial.println("connected!");
      // if (!client.subscribe(MQTT_SUB_TOPIC))
      //   pubSubErr(client.state());
    }
    else
    {
      Serial.print("failed, reason -> ");
      pubSubErr(client.state());
      if (!nonBlocking)
      {
        Serial.println(" < try again in 5 seconds");
        delay(5000);
      }
      else
      {
        Serial.println(" <");
      }
    }
    if (nonBlocking)
      break;
  }
}

void connectToWiFi(String init_str)
{
  if (init_str != emptyString)
    Serial.print(init_str);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  if (init_str != emptyString)
    Serial.println("ok!");
}

void checkWiFiThenMQTT(void)
{
  connectToWiFi("Checking WiFi");
  connectToMqtt();
}

unsigned long previousMillis = 0;
const long interval = 5000;

void checkWiFiThenMQTTNonBlocking(void)
{
  connectToWiFi(emptyString);
  if (millis() - previousMillis >= interval && !client.connected()) {
    previousMillis = millis();
    connectToMqtt(true);
  }
}

void checkWiFiThenReboot(void)
{
  connectToWiFi("Checking WiFi");
  Serial.print("Rebooting");
  ESP.restart();
}

void sendData(int value, const char* topic, const char* type)
{
  DynamicJsonDocument jsonBuffer(JSON_OBJECT_SIZE(3) + 100);
  JsonObject root = jsonBuffer.to<JsonObject>();
  JsonObject state = root.createNestedObject("state");
  JsonObject state_reported = state.createNestedObject("reported");
  state_reported["value"] = value;
  state_reported["type"] = type;
  
  Serial.printf("Sending  [%s]: ", topic);
  serializeJson(root, Serial);
  Serial.println();
  char shadow[measureJson(root) + 1];
  serializeJson(root, shadow, sizeof(shadow));
  if (!client.publish(topic, shadow, false))
    pubSubErr(client.state());
}


void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Base Station");
  //setup MQTT and WIFI
 #ifdef ESP32
  WiFi.setHostname("Base_Station");
#else
  WiFi.hostname("Base_Station");
#endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  connectToWiFi(String("Attempting to connect to SSID: ") + String(ssid));
  NTPConnect();
#ifdef ESP32
  net.setCACert(cacert);
  net.setCertificate(client_cert);
  net.setPrivateKey(privkey);
#else
  net.setTrustAnchors(&cert);
  net.setClientRSACert(&client_crt, &key);
#endif
  client.setServer(MQTT_HOST, MQTT_PORT);
  client.setCallback(messageReceived);
  connectToMqtt();
  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  while (!LoRa.begin(915E6)) {
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0xAA);  // Change sync word (0xF3) to match the receiver The sync word assures you don't get LoRa messages from other LoRa transceivers ranges from 0-0xFF
  Serial.println("LoRa Initializing OK!");
}
// String receivedData = "SEN:1024";
String receivedData;

void loop() {
  now = time(nullptr);
  if (!client.connected())
  {
    checkWiFiThenMQTT();
    //checkWiFiThenMQTTNonBlocking();
  }
  else
  {
    client.loop();
    receivedData = receivePacket();
    if (millis() - lastMillis > 1000)
    {
      lastMillis = millis();
      if (receivedData.length() > 0) {
        Serial.println("Sending MQTT");
        parseSendPacket(receivedData);
      }
    }
  }
}
const char type_angle[] = "ANGLE";
const char type_dillation[] = "DILLATION";

void parseSendPacket(String received) {
  if (received.length() > 0) {
    int colonIndex = received.indexOf(':');
    
    if (colonIndex != -1) {
      String id = received.substring(0, colonIndex);
      int val = received.substring(colonIndex + 1).toInt();

      if (id.equals("SEN")) {
        sendData(val, MQTT_PUB_TOPIC_DILATACAO,type_dillation);
        Serial.print("Sending SEN:");
        Serial.println(received);

      } else if (id.equals("ANG")) {
        sendData(val, MQTT_PUB_TOPIC_ANGULO,type_angle);
        Serial.print("Sending ANG:");
        Serial.println(received);
      }
    }
    else{
      Serial.println("Wrong message format");
    }
  }
}

String receivePacket() {
  String LoRaData = "";
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Received packet '");
    while (LoRa.available()) {
      LoRaData = LoRa.readString();
      Serial.print(LoRaData);
    }
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
  return LoRaData;
}