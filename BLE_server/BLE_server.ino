/*********
  Rui Santos
  Complete instructions at https://RandomNerdTutorials.com/esp32-ble-server-client/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>

#define bleServerName "HORSE_ESP32"
// Variables for sensor readings
float angle;
int dillation;
// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 30000;
bool deviceConnected = false;
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"

// Angle and dillation Characteristic and Descriptor
BLECharacteristic angleCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor angleDescriptor(BLEUUID((uint16_t)0x2902));
BLECharacteristic dillattionCharacteristics("ca73b3ba-39f6-4ab3-91ae-186dc9577d99", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor dillationDescriptor(BLEUUID((uint16_t)0x2903));

//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void setup() {
  Serial.begin(115200);
  // Create the BLE Device
  BLEDevice::init(bleServerName);
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *sensorsService = pServer->createService(SERVICE_UUID);
  // Create BLE Characteristics and Create a BLE Descriptor
  // Angle
  sensorsService->addCharacteristic(&angleCharacteristics);
  angleDescriptor.setValue("BME temperature Celsius");
  angleCharacteristics.addDescriptor(&angleDescriptor);
  // Dillation
  sensorsService->addCharacteristic(&dillattionCharacteristics);
  dillationDescriptor.setValue("BME humidity");
  dillattionCharacteristics.addDescriptor(new BLE2902());
  // Start the service
  sensorsService->start();
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
  pinMode(ledPin, OUTPUT);

}

void loop() {
  if (deviceConnected) {
    if ((millis() - lastTime) > timerDelay) {
      // Read values from sensors
      angle = readAngle();
      dillation = readDillation();
      //Notify readings to connected clients
      static char angleTemp[6]; 
      static char dillationTemp[6];
      dtostrf(angle, 6, 2, angleTemp);
      dtostrf(dillation, 6, 2, dillationTemp);
      angleCharacteristics.setValue(angleTemp);
      dillattionCharacteristics.setValue(dillationTemp);
      angleCharacteristics.notify();
      dillattionCharacteristics.notify();   
      // Print values
      Serial.print("Angle: ");
      Serial.print(angle);
      Serial.print(" º");
      Serial.print(" Dillation: ");
      Serial.print(dillation);
      lastTime = millis();
    }
  }
}

float readAngle(){
  return random(0,360); // Repalce with sensor value
}
int readDillation(){
  return random(0,1024); //Replace with sensor value
}
