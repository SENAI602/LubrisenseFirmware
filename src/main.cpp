#include <Arduino.h>
#include "BleService.h"

unsigned long lastUpdate = 0;

void setup() {
  delay(2000);
  Serial.begin(115200);
  delay(1000);
  Serial.println("Iniciando ESP32 com BLE...");
  setupBLE();
}

void loop() {
  //VARIAVEL GLOBAR FOI ALTERAR?
  //TRATAR JSON
  
  // FALAAAAAAAAA
  //QUANDO QUISEREM EMVIAR DADOS PELO BLUETOOTH FAZ ASSIM:
  //updateBLE(message);

  //ATENÇÃO:
  //BLUETOOH ESTÁ FUNCIONANDO MAS TEM QUE FAZER O ESP 'ACORDAR' SÓ QUANDO RECEBER ALGO BLUETOOTH
  //AINDA NÃO FIZ ISSO. BORAAAAAAAAAAAAAA
}