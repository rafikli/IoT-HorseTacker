/*********
  Rui Santos
  Complete instructions at https://RandomNerdTutorials.com/esp32-ble-server-client/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/

#include "BLEDevice.h"
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include "BLEDevice.h"
#include <Wire.h>
//define the pins used by the transceiver module
#define ss 18
#define rst 23
#define dio0 26 
//BLE Server name (the other ESP32 name running the server sketch)
#define bleServerName "HORSE_ESP32"
unsigned long lastMillis = 0;

/* UUID's of the service, characteristic that we want to read*/
// BLE Service
static BLEUUID horseServiceUUID("91bad492-b950-4226-aa2b-4ede9fa42f59");

// BLE Characteristics
static BLEUUID AngleCharacteristicUUID("cba1d466-344c-4be3-ab3f-189f80dd7518");
static BLEUUID dillationCharacteristicUUID("ca73b3ba-39f6-4ab3-91ae-186dc9577d99");

//Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;

//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;
 
//Characteristicd that we want to read
static BLERemoteCharacteristic* AngleCharacteristic;
static BLERemoteCharacteristic* dillationCharacteristic;

//Activate notify
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};

//Variables to store Angle and dillation
char* AngleChar;
char* dillationChar;

//Flags to check whether new Angle and dillation readings are available
boolean newAngle = false;
boolean newdillation = false;

//Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToServer(BLEAddress pAddress) {
  BLEClient* pClient = BLEDevice::createClient();
  // Connect to the remove BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(horseServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(horseServiceUUID.toString().c_str());
    return (false);
  }
  // Obtain a reference to the characteristics in the service of the remote BLE server.
  AngleCharacteristic = pRemoteService->getCharacteristic(AngleCharacteristicUUID);
  dillationCharacteristic = pRemoteService->getCharacteristic(dillationCharacteristicUUID);

  if (AngleCharacteristic == nullptr || dillationCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.println(" - Found our characteristics");
  //Assign callback functions for the Characteristics
  AngleCharacteristic->registerForNotify(AngleNotifyCallback);
  dillationCharacteristic->registerForNotify(dillationNotifyCallback);
  return true;
}

//Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) { //Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      doConnect = true; //Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};
 
//When the BLE Server sends a new Angle reading with the notify property
static void AngleNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                        uint8_t* pData, size_t length, bool isNotify) {
  //store Angle value
  AngleChar = (char*)pData;
  newAngle = true;
}

//When the BLE Server sends a new dillation reading with the notify property
static void dillationNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                    uint8_t* pData, size_t length, bool isNotify) {
  //store dillation value
  dillationChar = (char*)pData;
  newdillation = true;
  Serial.print(newdillation);
}

void setup() {
  //Start serial communication
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender");
  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  while (!LoRa.begin(915E6)) {
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0xAA);// The sync word assures you don't get LoRa messages from other LoRa transceivers ranges from 0-0xFF
  Serial.println("LoRa Initializing OK!");
  Serial.println("Starting Arduino BLE Client application...");
  //Init BLE device
  BLEDevice::init("");
 
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
}

void loop() {
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
    if (millis() - lastMillis > 1000)
  {
    lastMillis = millis();
    if (doConnect == true) {
      if (connectToServer(*pServerAddress)) {
        Serial.println("We are now connected to the BLE Server.");
        //Activate the Notify property of each Characteristic
        AngleCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
        dillationCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
        connected = true;
      } else {
        Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
      }
      doConnect = false;
    }
    //if new Angle readings are available, print in the OLED
    if (newAngle && newdillation){
      newAngle = false;
      newdillation = false;
      Serial.println(AngleChar);
      sendPacket("ANG:" + String(AngleChar));
      Serial.println(dillationChar);
      delay(100);
      sendPacket("SEN:" + String(dillationChar));
    }
  }
}

void sendPacket(String msg){
  Serial.print("Sending packet: ");
  Serial.println(msg);

  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();
}