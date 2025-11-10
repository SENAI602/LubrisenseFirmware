#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <Arduino.h>

// Declaração das funções que serão definidas no BleService.cpp
void setupBLE();
void updateBLE(const String& value);

// Declaração 'extern' para as variáveis globais.
// Isso informa ao compilador que essas variáveis existem em outro arquivo (main.cpp)
// e que não devem ser criadas novamente aqui.
extern volatile bool newDataFromBLE;
extern String bleReceivedValue;

#endif
