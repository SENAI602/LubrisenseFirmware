/**
 * =========================================================================================
 * PROJETO: FIRMWARE HÍBRIDO BLINDADO - LUBRISENSE 5.0 (POWER SAVER)
 * PLATAFORMA: ESP32-S3 (Framework Arduino / PlatformIO)
 * VERSÃO: 5.0 FINAL (V4.7 STABLE + DEEP SLEEP + WAKEUP TRIGGERS)
 * AUTOR: Equipe de Desenvolvimento Smart Factory - FREMTEC
 * =========================================================================================
 * * DESCRIÇÃO GERAL:
 * Firmware para controle de lubrificador automático inteligente com gestão avançada de energia.
 * O sistema permanece em modo de baixo consumo (Deep Sleep) na maior parte do tempo,
 * acordando apenas para eventos programados ou interação manual.
 *
 * * LÓGICA DE ENERGIA (NOVO NA v5.0):
 * 1. Sleep: Entra em Deep Sleep após 60s de inatividade (sem conexão BLE e sem motor rodando).
 * 2. Wakeup Botão: O botão manual (GPIO 2) acorda o chip instantaneamente (Interrupção Ext0).
 * 3. Wakeup Timer: O chip acorda sozinho para cumprir o ciclo de lubrificação automática.
 *
 * * FUNCIONALIDADES (Mantidas da v4.7):
 * 1. LoRaMESH: Telemetria e comandos via Gateway.
 * 2. Bluetooth LE: Configuração JSON com buffer para pacotes grandes e resposta rápida (Anti-Timeout).
 * 3. Controle Motor: Dosagem precisa (Encoder) e Proteção (Corrente/Nível).
 * 4. Persistência: SPIFFS para Config e Logs.
 *
 * * PINAGEM CRÍTICA (ESP32-S3):
 * [LoRa]   TX:43 | RX:44
 * [Motor]  EN:38 | IN1:21 | IN2:18 | EncA:15 | EncB:16
 * [IO]     LED:7 | Btn:2 | NívelBaixo:5 | NívelAlto:6
 * [I2C]    SDA:11 | SCL:10
 * [Analog] PT100:1
 * =========================================================================================
 */

// --------------------------------------------------------------------------
// 1. INCLUSÃO DE BIBLIOTECAS
// --------------------------------------------------------------------------
#include <Arduino.h>
#include <vector>
#include <WiFi.h>          // Apenas para NTP no boot (se necessário)
#include <Wire.h>          // I2C
#include <SPIFFS.h>        // Sistema de Arquivos
#include <ArduinoJson.h>   // Manipulação JSON
#include "RTClib.h"        // RTC DS3231
#include <INA226.h>        // Sensor de Corrente
#include <MotorEncoder.h>  // Controle PID/Pulsos Motor
#include "LoRaMESH.h"      // Rádio LoRa

// Bibliotecas Nativas BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Biblioteca de Energia (Deep Sleep)
#include <esp_sleep.h>

// --------------------------------------------------------------------------
// 2. DEFINIÇÕES DE HARDWARE (PINOUT)
// --------------------------------------------------------------------------
// Interface LoRa
#define LORA_TX_PIN         43  
#define LORA_RX_PIN         44  

// Interface de Usuário e Sensores
#define LED_PIN_GPIO7       7   // LED Status
#define SEN_NIVEL_BAIXO     5   // Reservatório Vazio
#define SEN_NIVEL_ALTO      6   // Reservatório Cheio
#define BTN_MANUAL          2   // Botão Ação / Wakeup Source

// Driver Motor
#define MOTOR_IN1           21  
#define MOTOR_IN2           18  
#define MOTOR_EN            38  
#define ENCODER_PIN_A       15  
#define ENCODER_PIN_B       16  

// Barramentos
#define I2C_SDA             11  
#define I2C_SCL             10  
#define PT100_PIN           1   // Entrada Analógica

// --------------------------------------------------------------------------
// 3. CONSTANTES E PARÂMETROS GLOBAIS
// --------------------------------------------------------------------------
// Bluetooth (Compatível com App)
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID           "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"

// Rede Wi-Fi (NTP)
const char* ssid = "smartfactory";      
const char* password = "smartfactory"; 
const char* ntpServer = "a.st1.ntp.br";
const long  gmtOffset_sec = -3 * 3600; 
const int   daylightOffset_sec = 0;

// Arquivos
#define CONFIG_FILE "/config.json" 
#define LOG_FILE    "/log.jsonl"   
#define TEMP_FILE   "/temp.json"   

// LoRa
#define GATEWAY_ID 0 
#define CMD_TO_GATEWAY_EVENT   0x20 
const size_t limiteLinhasLog = 10;
const unsigned long intervaloEnviarGateway = 30000; 
const unsigned long TIMEOUT_ACK = 15000;

// Configuração de Energia (NOVO v5.0)
const unsigned long TEMPO_OCIOSO_MAXIMO = 60000; // 60s sem atividade = Dormir

// Motor
const float CORRENTE_MAXIMA_MOTOR = 2000.0; // mA
const float SHUNT_MOTOR_OHMS = 0.02;        
const int VELOCIDADE_MOTOR = 60;            
const long PULSOS_PARA_UMA_VOLTA = 5650;    
const float GRAMAS_POR_VOLTA = 2.21;        

// Sensores
const float TEMP_MIN = 0.0;       
const float TEMP_MAX = 150.0;
#define FATOR_CONVERSAO_PT100 100.0 
#define NUM_AMOSTRAS_ADC 20  

// --------------------------------------------------------------------------
// 4. OBJETOS E VARIÁVEIS DE ESTADO
// --------------------------------------------------------------------------
HardwareSerial LoRaSerial(2); 
LoRaMESH lora(&LoRaSerial);   
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
RTC_DS3231 rtc;               
INA226 ina226Bateria(0x40);   
INA226 ina226Motor(0x41);     
MotorEncoder Motor(ENCODER_PIN_A, ENCODER_PIN_B, MOTOR_EN, MOTOR_IN1, MOTOR_IN2);

// --- VARIÁVEIS RTC (MEMÓRIA QUE SOBREVIVE AO DEEP SLEEP) ---
RTC_DATA_ATTR int bootCount = 0; 

// --- CONTROLE DE ENERGIA (VOLÁTIL) ---
unsigned long lastActivityTime = 0; // Marca o tempo da última interação (Resetado por botões/BLE/LoRa)

// Estado Bluetooth (Buffer para JSON grandes)
bool deviceConnected = false;      
bool oldDeviceConnected = false;   
String bleBuffer = "";             
bool bleJsonCompleto = false;      
volatile bool estadoLedAtual = false; 
bool precisaNotificarApp = false;  

// Estado LoRa
unsigned long ultimoReenvio = 0;
bool aguardandoAck = false;
unsigned long aguardandoAckDesde = 0;
unsigned long ultimaSincronizacao = 0; 

// Configuração (Runtime)
String varConfigUuid = "";
int    varConfigTipoConfig = 0; 
int    varConfigVolume = 10;          
unsigned long varConfigIntervaloTrigger_ms = 0; 
unsigned long varConfigIntervalo = 0; 
unsigned long varConfigValorIntervalo = 0; 
int varConfigTipoIntervalo = 0;
int varConfigFrequencia = 0;
int varConfigTipoFrequencia = 0;
bool configExiste = false;

// Estado do Sistema (Lógica de Negócio)
bool isClockValid = false;
bool varTempCicloStartado = false;
String varTempHorarioStartado = "";
String varTempUltimaLubrificacao = "";
bool motorLigado = false;
Sentido motorDirecaoAtual = HORARIO;
String fonteDoEventoAtual = "Nenhum";

// Botão Manual
int estadoAnteriorBtnManual = HIGH;
unsigned long ultimoAcionamentoBotaoManual = 0;
const unsigned long intervaloBloqueio = 5000; 

// --------------------------------------------------------------------------
// 5. PROTÓTIPOS DE FUNÇÕES
// --------------------------------------------------------------------------
void salvarConfig(const char* json);
void inicializaConfig();
void inicializarArquivoTemp();
void registrarEvento(const String& fonte, bool sucesso);
String formatarTimestamp(const DateTime& dt);
DateTime stringParaDateTime(const String& timestampStr);
float converterParaDias(unsigned long valor, int tipo);
void iniciarMovimentoMotor(float gramas, Sentido direcao);
void pararMotor();
int lerNivelBateria();
float lerTemperaturaPT100();
void salvarEstadoCiclo(bool cicloStartado, const String& horarioStartado, const String& ultimaLubrificacao);
bool sincronizarViaNTP();
void displayFileContent(const char* filename);
void tentarReenvio();
void sendJsonViaLoRa(const String &json, uint8_t command);
void updateBLE(const String& value);

// Novas Funções v5.0
void resetInactivityTimer();
void entrarEmDeepSleep();

// ==========================================================================
// 6. IMPLEMENTAÇÃO DO BLUETOOTH (CALLBACKS)
// ==========================================================================

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      resetInactivityTimer(); // Conectou? Reseta o timer de sono
      Serial.println(">> [BLE] CELULAR CONECTADO!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println(">> [BLE] DESCONECTADO. Reiniciando Anuncio...");
      pServer->startAdvertising(); 
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      
      if (value.length() > 0) {
        resetInactivityTimer(); // Recebeu dados? Reseta o timer de sono

        String chunk = String(value.c_str());

        // Comando Curto (LED) - Processa Imediato
        if (chunk == "1" || chunk == "0") {
             bleBuffer = chunk;
             bleJsonCompleto = true;
             return;
        }

        // JSON Longo - Acumula no Buffer
        bleBuffer += chunk;
        
        // Proteção de estouro de memória
        if (bleBuffer.length() > 2048) bleBuffer = "";

        String temp = bleBuffer;
        temp.trim();
        if (temp.endsWith("}")) {
            bleJsonCompleto = true; 
        }
      }
    }
};

void setupBLE() {
  BLEDevice::init("LUBRISENSE_Device"); 
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHAR_UUID,
                      BLECharacteristic::PROPERTY_READ | 
                      BLECharacteristic::PROPERTY_WRITE | 
                      BLECharacteristic::PROPERTY_WRITE_NR | // Permite Fire-and-Forget
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); 
  pAdvertising->setMaxPreferred(0x12);
  
  BLEDevice::startAdvertising();
  Serial.println(">> [BLE] PRONTO E VISÍVEL (LUBRISENSE_Device)");
}

// ==========================================================================
// 7. LÓGICA LORA (SNIFFER)
// ==========================================================================
void verificarLoRaCirurgico() {
  if (LoRaSerial.available() > 0) {
    resetInactivityTimer(); // Tráfego LoRa? Reseta timer de sono
    
    static uint8_t buffer[20];
    static int index = 0;
    uint8_t byteLido = LoRaSerial.read();

    if (index == 0 && byteLido != 0x01) return; 
    buffer[index++] = byteLido;

    if (index >= 9) {
      if (buffer[2] == 0xC2) {
         uint8_t nivel = buffer[6]; 
         Serial.printf("[LORA] CMD: %d\n", nivel);
         if (nivel == 1) { digitalWrite(LED_PIN_GPIO7, HIGH); estadoLedAtual = true; } 
         else { digitalWrite(LED_PIN_GPIO7, LOW); estadoLedAtual = false; }
         precisaNotificarApp = true;
      }
      index = 0; 
    }
  }
}

// ==========================================================================
// 8. SETUP DO SISTEMA
// ==========================================================================
void setup() {
  Serial.begin(115200);
  
  // --- LOGICA DE BOOT E SONO (v5.0) ---
  bootCount++;
  Serial.printf("Boot number: %d\n", bootCount);

  // Configura Botão como Despertador (Wakeup Source)
  pinMode(BTN_MANUAL, INPUT_PULLUP);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_2, 0); // Acorda quando GPIO2 for LOW (Apertado)

  // Verifica motivo do despertar
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
      Serial.println("[BOOT] Acordado pelo Botão Manual!");
  } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
      Serial.println("[BOOT] Acordado pelo Timer (Ciclo Automático).");
  }

  // Trava de Segurança (Recuperação de USB)
  // Só entra se o botão estiver apertado E não for um despertar de sleep (Reset físico)
  if (digitalRead(BTN_MANUAL) == LOW && wakeup_reason != ESP_SLEEP_WAKEUP_EXT0) {
      Serial.println("\n!!! MODO DE SEGURANÇA ATIVADO !!!");
      Serial.println("USB Ativa. Código principal cancelado.");
      while(1) { delay(100); }
  }
  
  delay(100); // Estabilização

  // Configuração de Pinos
  pinMode(SEN_NIVEL_BAIXO, INPUT);
  pinMode(SEN_NIVEL_ALTO, INPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(LED_PIN_GPIO7, OUTPUT);

  digitalWrite(MOTOR_EN, LOW);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(LED_PIN_GPIO7, LOW);

  if (PT100_PIN >= 0) analogSetAttenuation(ADC_11db);

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!rtc.begin()) Serial.println("[ERRO] RTC não encontrado");
  
  if (rtc.lostPower()) {
    Serial.println("[RTC] Bateria fraca. Sincronizando NTP...");
    if (sincronizarViaNTP()) ultimaSincronizacao = millis();
  }
  
  DateTime agora = rtc.now();
  isClockValid = (agora.year() >= 2024);

  if (!SPIFFS.begin(true)) Serial.println("[ERRO] SPIFFS falhou");
  else Serial.println("[OK] Sistema de Arquivos montado");
  
  inicializaConfig();
  
  // Configuração Padrão
  if (!configExiste) {
    Serial.println("[INIT] Criando configuração de fábrica...");
    const char* def = "{\"Uuid\":\"novo-uuid\",\"Tag\":\"PONTO-01\",\"Equipamento\":\"MOTOR\",\"TipoConfig\":1,\"Volume\":10,\"Intervalo\":1,\"TipoIntervalo\":3,\"Frequencia\":1,\"TipoFrequencia\":2}";
    salvarConfig(def);
    configExiste = true;
  }

  inicializarArquivoTemp();

  // Inicializa LoRa
  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  lora.begin(false); 
  if(lora.localId != 0) { lora.setnetworkId(0); lora.setpassword(602); }

  // Inicializa Sensores
  ina226Bateria.begin();
  ina226Motor.begin();
  ina226Motor.setAverage(128); 
  Motor.begin(PULSOS_PARA_UMA_VOLTA, GRAMAS_POR_VOLTA);

  // Inicializa Timer de Inatividade
  resetInactivityTimer();

  Serial.println("Iniciando BLE em 2s...");
  delay(2000);
  setupBLE();

  Serial.println("--- SISTEMA HIBRIDO (POWER SAVER) PRONTO ---");
  Serial.println("DIAGNOSTICO: Envie 'c' (config), 'l' (log), 't' (temp)");
}

// ==========================================================================
// 9. LOOP PRINCIPAL (GERENCIADOR DE ENERGIA)
// ==========================================================================
void loop() {
  
  // --- 1. Verificações de Comunicação ---
  verificarLoRaCirurgico();

  if (Serial.available() > 0) {
    resetInactivityTimer(); // Atividade Serial reseta timer
    char command = Serial.read();
    while(Serial.available() > 0) Serial.read(); 
    if (command == 'c') displayFileContent(CONFIG_FILE);
    if (command == 'l') displayFileContent(LOG_FILE);
    if (command == 't') displayFileContent(TEMP_FILE);
  }

  // --- 2. Processamento Bluetooth ---
  if (bleJsonCompleto) {
      resetInactivityTimer(); // Atividade BLE reseta timer
      bleJsonCompleto = false; 
      String msg = bleBuffer;  
      bleBuffer = "";          

      Serial.print("[BLE] Mensagem: ");
      Serial.println(msg);

      if (msg.startsWith("{")) {
          // Timing v4.7: Delay -> ACK -> Delay -> Save
          delay(100);
          updateBLE("OK: Config Salva");
          Serial.println("[BLE] Ack enviado. Gravando...");
          delay(500); 
          salvarConfig(msg.c_str());
      }
      else if (msg == "1") { digitalWrite(LED_PIN_GPIO7, HIGH); estadoLedAtual = true; updateBLE("1"); }
      else if (msg == "0") { digitalWrite(LED_PIN_GPIO7, LOW); estadoLedAtual = false; updateBLE("0"); }
  }

  // --- 3. Sincronia ---
  if (precisaNotificarApp) {
      precisaNotificarApp = false;
      if (deviceConnected) {
          resetInactivityTimer();
          updateBLE(estadoLedAtual ? "1" : "0");
          Serial.println("[SYNC] Status enviado ao App.");
      }
  }

  // --- 4. Lógica de Negócio ---
  if (isClockValid && configExiste) {
      // A. Botão Manual (Acorda e reseta timer)
      int btn = digitalRead(BTN_MANUAL);
      if (configExiste && btn == LOW && estadoAnteriorBtnManual == HIGH && (millis() - ultimoAcionamentoBotaoManual > intervaloBloqueio)){
        resetInactivityTimer(); // Mantém acordado
        Serial.println("[MANUAL] Ciclo Iniciado via Botão");
        fonteDoEventoAtual = "Manual";
        
        // Cálculo de dose (Lógica Completa v4.7)
        float dose = 0.0f;
        if (varConfigTipoConfig == 2) dose = (float)varConfigVolume;
        else if (varConfigTipoConfig == 1) { 
             float diasTotal = converterParaDias(varConfigValorIntervalo, varConfigTipoIntervalo);
             float diasFreq = converterParaDias(varConfigFrequencia, varConfigTipoFrequencia);
             if (diasFreq > 0) {
                float numDoses = diasTotal / diasFreq;
                if (numDoses > 0) dose = (float)varConfigVolume / numDoses;
             }
        } 

        if (dose > 0) iniciarMovimentoMotor(dose, HORARIO);
        ultimoAcionamentoBotaoManual = millis();
        salvarEstadoCiclo(true, formatarTimestamp(rtc.now()), formatarTimestamp(rtc.now()));
      }
      estadoAnteriorBtnManual = btn;

      // B. Automático (Verifica e reseta timer se ativar)
      if (varTempCicloStartado) {
        DateTime agora = rtc.now();
        DateTime ultima = stringParaDateTime(varTempUltimaLubrificacao);
        if (agora.unixtime() >= (ultima.unixtime() + (varConfigIntervaloTrigger_ms/1000))) {
          resetInactivityTimer(); // Acordou para trabalhar
          Serial.println("[AUTO] Hora de Lubrificar!");
          
          // Recálculo da dose para o automático
          float dose = 0.0f;
          if (varConfigTipoConfig == 2) dose = (float)varConfigVolume;
          else if (varConfigTipoConfig == 1) { 
               float diasTotal = converterParaDias(varConfigValorIntervalo, varConfigTipoIntervalo);
               float diasFreq = converterParaDias(varConfigFrequencia, varConfigTipoFrequencia);
               if (diasFreq > 0) {
                  float numDoses = diasTotal / diasFreq;
                  if (numDoses > 0) dose = (float)varConfigVolume / numDoses;
               }
          }
          
          if (dose > 0) iniciarMovimentoMotor(dose, HORARIO);
          varTempUltimaLubrificacao = formatarTimestamp(agora);
          salvarEstadoCiclo(true, varTempHorarioStartado, varTempUltimaLubrificacao);
        }
      }

      // C. Controle do Motor (Não dorme enquanto gira)
      if (motorLigado) {
        resetInactivityTimer(); 
        float correnteMotor = (ina226Motor.getShuntVoltage_mV() / SHUNT_MOTOR_OHMS);
        if (correnteMotor > CORRENTE_MAXIMA_MOTOR) {
          Serial.println("[MOTOR] ERRO: Sobrecorrente!");
          pararMotor();
          registrarEvento(fonteDoEventoAtual, false);
        }
        if (motorDirecaoAtual == HORARIO && digitalRead(SEN_NIVEL_BAIXO) == LOW) {
           Serial.println("[MOTOR] ERRO: Nível de graxa baixo!");
           pararMotor();
           registrarEvento(fonteDoEventoAtual, false);
        }
        if (Motor.atualizar() == MOTOR_ALVO_ATINGIDO) {
          Serial.println("[MOTOR] Dosagem concluída.");
          registrarEvento(fonteDoEventoAtual, true);
          motorLigado = false;
        }
      }
  }

  // --- 5. Manutenção ---
  if (millis() - ultimoReenvio > intervaloEnviarGateway) {
    ultimoReenvio = millis();
    if (isClockValid && !aguardandoAck) tentarReenvio();
  }

  if (!deviceConnected && oldDeviceConnected) {
      delay(500); 
      pServer->startAdvertising(); 
      Serial.println("[BLE] Reiniciando Anuncio...");
      oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
      resetInactivityTimer(); // Conexão nova reseta timer
  }
  
  // Se estiver conectado, reseta o timer constantemente para não dormir no meio da configuração
  if (deviceConnected) resetInactivityTimer();

  // === 6. GESTÃO DE DEEP SLEEP (O CÉREBRO DA BATERIA) ===
  // Se passou 60s sem fazer nada E não tem conexão E motor está parado -> Dorme
  if (millis() - lastActivityTime > TEMPO_OCIOSO_MAXIMO) {
      if (!deviceConnected && !motorLigado) {
          entrarEmDeepSleep();
      }
  }

  delay(20); // Yield para Watchdog
}

// ==========================================================================
// 10. IMPLEMENTAÇÃO DAS FUNÇÕES AUXILIARES
// ==========================================================================

// --- (v5.0) Funções de Energia ---
void resetInactivityTimer() {
    lastActivityTime = millis();
}

void entrarEmDeepSleep() {
    Serial.println("[SLEEP] Entrando em Deep Sleep...");
    Serial.flush();
    
    // 1. Configura Despertador: Botão Manual
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_2, 0); 

    // 2. Configura Despertador: Timer (Próxima Dose)
    if (configExiste && varTempCicloStartado) {
        DateTime agora = rtc.now();
        DateTime ultima = stringParaDateTime(varTempUltimaLubrificacao);
        long intervaloSegundos = varConfigIntervaloTrigger_ms / 1000;
        long proximaDose = ultima.unixtime() + intervaloSegundos;
        long segundosAteAcordar = proximaDose - agora.unixtime();

        if (segundosAteAcordar > 0) {
            // Converte segundos para microsegundos
            esp_sleep_enable_timer_wakeup(segundosAteAcordar * 1000000ULL);
            Serial.printf("[SLEEP] Acordarei em %ld segundos.\n", segundosAteAcordar);
        } else {
             // Se já passou da hora, dorme pouco (10s) só pra resetar e processar
             esp_sleep_enable_timer_wakeup(10 * 1000000ULL); 
        }
    }

    // 3. Desliga tudo e dorme
    esp_deep_sleep_start();
}

// --- Funções v4.7 (Mantidas Completas) ---

void updateBLE(const String& value) {
  if (deviceConnected && pCharacteristic) {
    pCharacteristic->setValue(value.c_str());
    pCharacteristic->notify();
  }
}

void sendJsonViaLoRa(const String &json, uint8_t command) {
  if (lora.PrepareFrameCommand(GATEWAY_ID, command, (uint8_t*)json.c_str(), json.length())) {
    lora.SendPacket(); 
  }
}

void inicializaConfig() {
  if (SPIFFS.exists(CONFIG_FILE)) {
    File file = SPIFFS.open(CONFIG_FILE, "r");
    if (file && file.size() > 2) {
      configExiste = true;
      String content = file.readString();
      file.close();
      salvarConfig(content.c_str());
    } else {
      if(file) file.close();
      configExiste = false;
    }
  }
}

void salvarConfig(const char* json) {
  File f = SPIFFS.open(CONFIG_FILE, "w");
  if (f) {
    f.print(json);
    f.close();
    Serial.println("[CONFIG] Arquivo salvo na Flash."); 
    JsonDocument doc;
    deserializeJson(doc, json);
    if (!doc["payload"].isNull()) { 
        JsonObject payload = doc["payload"];
        if (!payload["Volume"].isNull()) varConfigVolume = payload["Volume"].as<int>();
        
        if (!payload["Intervalo"].isNull()) {
             varConfigValorIntervalo = payload["Intervalo"].as<unsigned long>();
             varConfigTipoIntervalo = payload["TipoIntervalo"].as<int>();
             unsigned long mult = 1000;
             if(varConfigTipoIntervalo == 2) mult = 86400000UL; 
             else if(varConfigTipoIntervalo == 3) mult = 2592000000UL; 
             else if(varConfigTipoIntervalo == 1) mult = 3600000UL; 
             varConfigIntervalo = varConfigValorIntervalo * mult;
        }
        if (!payload["Frequencia"].isNull()) {
             varConfigFrequencia = payload["Frequencia"].as<unsigned long>();
             varConfigTipoFrequencia = payload["TipoFrequencia"].as<int>();
             unsigned long mult = 1000;
             if(varConfigTipoFrequencia == 2) mult = 86400000UL;
             else if(varConfigTipoFrequencia == 3) mult = 2592000000UL;
             else if(varConfigTipoFrequencia == 1) mult = 3600000UL;
             varConfigIntervaloTrigger_ms = varConfigFrequencia * mult;
        }
    } 
    else if (!doc["Volume"].isNull()) {
         varConfigVolume = doc["Volume"].as<int>();
    }
  } else {
      Serial.println("[ERRO] Falha ao gravar Config!");
  }
}

void registrarEvento(const String& fonte, bool sucesso) {
  std::vector<String> lines;
  File readFile = SPIFFS.open(LOG_FILE, "r");
  if (readFile) {
    while (readFile.available()) {
      String line = readFile.readStringUntil('\n');
      if (line.length() > 0) lines.push_back(line);
    }
    readFile.close();
  }
  while (lines.size() >= limiteLinhasLog) lines.erase(lines.begin());
  JsonDocument doc;
  doc["Hora"] = formatarTimestamp(rtc.now());
  doc["Sucesso"] = sucesso;
  doc["Modo"] = fonte;
  doc["Temperatura"] = lerTemperaturaPT100();
  doc["Bateria"] = lerNivelBateria();
  doc["e"] = false; 
  String newLogLine;
  serializeJson(doc, newLogLine);
  lines.push_back(newLogLine);
  File writeFile = SPIFFS.open(LOG_FILE, "w");
  if (writeFile) {
    for (const auto& line : lines) writeFile.println(line);
    writeFile.close();
  }
}

void tentarReenvio() {
  if (!aguardandoAck) {
      File file = SPIFFS.open(LOG_FILE, "r");
      if (file) {
          while (file.available()) {
              String line = file.readStringUntil('\n');
              if(line.length() == 0) continue;
              JsonDocument doc;
              deserializeJson(doc, line);
              if (doc["e"] == false) {
                  doc["e"] = true; 
                  String out;
                  serializeJson(doc, out);
                  sendJsonViaLoRa(out, CMD_TO_GATEWAY_EVENT);
                  aguardandoAck = true;
                  aguardandoAckDesde = millis();
                  break;
              }
          }
          file.close();
      }
  } else {
      if(millis() - aguardandoAckDesde > TIMEOUT_ACK) aguardandoAck = false;
  }
}

void displayFileContent(const char* filename) {
  File file = SPIFFS.open(filename, "r");
  if (file) {
      while (file.available()) Serial.write(file.read());
      file.close();
  }
  Serial.println();
}

bool sincronizarViaNTP() {
  WiFi.begin(ssid, password);
  int tenta = 20;
  while (WiFi.status() != WL_CONNECTED && tenta > 0) { delay(500); tenta--; }
  if (WiFi.status() == WL_CONNECTED) {
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      struct tm timeinfo;
      if (getLocalTime(&timeinfo)) {
          rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, 
                              timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
          WiFi.disconnect(true);
          WiFi.mode(WIFI_OFF);
          return true;
      }
  }
  return false;
}

void salvarEstadoCiclo(bool cicloStartado, const String& horarioStartado, const String& ultimaLubrificacao) {
  JsonDocument doc;
  doc["cicloStartado"] = cicloStartado;
  doc["horarioStartado"] = horarioStartado;
  doc["ultimaLubrificacao"] = ultimaLubrificacao;
  varTempCicloStartado = cicloStartado;
  varTempHorarioStartado = horarioStartado;
  varTempUltimaLubrificacao = ultimaLubrificacao;
  File file = SPIFFS.open(TEMP_FILE, "w");
  if (file) {
      serializeJson(doc, file);
      file.close();
  }
}

void inicializarArquivoTemp() {
  if (SPIFFS.exists(TEMP_FILE)) {
    File file = SPIFFS.open(TEMP_FILE, "r");
    if (file) {
      JsonDocument doc;
      deserializeJson(doc, file);
      varTempCicloStartado = doc["cicloStartado"];
      varTempHorarioStartado = doc["horarioStartado"].as<String>();
      varTempUltimaLubrificacao = doc["ultimaLubrificacao"].as<String>();
      file.close();
    }
  }
}

String formatarTimestamp(const DateTime& dt) {
  char buffer[25];
  sprintf(buffer, "%04d-%02d-%02dT%02d:%02d:%02d", dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
  return String(buffer);
}

DateTime stringParaDateTime(const String& timestampStr) {
  int Y, M, D, h, m, s;
  sscanf(timestampStr.c_str(), "%d-%d-%dT%d:%d:%d", &Y, &M, &D, &h, &m, &s);
  return DateTime(Y, M, D, h, m, s);
}

float converterParaDias(unsigned long valor, int tipo) {
  if (tipo == 1) return valor / 24.0f;
  if (tipo == 2) return (float)valor;
  if (tipo == 3) return valor * 30.0f;
  return 0;
}

void iniciarMovimentoMotor(float gramas, Sentido direcao) {
  if (!Motor.estaGirando()) {
      motorDirecaoAtual = direcao;
      Motor.iniciarGiroPorGramas(gramas, direcao, VELOCIDADE_MOTOR);
      motorLigado = true;
  }
}

void pararMotor() {
  Motor.pararMotor();
  motorLigado = false;
}

int lerNivelBateria() {
  float v = ina226Bateria.getBusVoltage();
  int p = (int)((v - 3.2) / (4.2 - 3.2) * 100.0);
  if(p > 100) p = 100; if(p < 0) p = 0;
  return p;
}

float lerTemperaturaPT100() {
  long total = 0;
  for(int i=0; i<NUM_AMOSTRAS_ADC; i++) {
      total += analogReadMilliVolts(PT100_PIN);
      delay(1);
  }
  float mV = total / (float)NUM_AMOSTRAS_ADC;
  float mA = (mV / 1000.0) / FATOR_CONVERSAO_PT100 * 1000.0;
  mA = constrain(mA, 4.0, 20.0);
  return (mA - 4.0) * (TEMP_MAX - TEMP_MIN) / (20.0 - 4.0) + TEMP_MIN;
}

void marcarEventoComoEnviado() {}