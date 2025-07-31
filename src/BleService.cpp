#include <Arduino.h>
#include <ArduinoJson.h>
#include "BleService.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID        "97ec4585-9e94-41b6-8902-1a2db274dfc9"
#define CHARACTERISTIC_UUID "c04c4646-d355-41ab-9097-89c2c6b9932b"

BLECharacteristic *pCharacteristic;

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      Serial.print("Recebido JSON:");
      Serial.println(rxValue.c_str());
    }
  }
};

void setupBLE() {
  BLEDevice::init("LUBRICENSE_Device");

  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_WRITE
  );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->setValue("Inicial");

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
}

void updateBLE(const String& value) {
  if (pCharacteristic != nullptr) {
    pCharacteristic->setValue(value.c_str());
    pCharacteristic->notify();
  }
}
