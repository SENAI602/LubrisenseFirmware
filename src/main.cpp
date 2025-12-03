/**
 * =========================================================================================
 * PROJETO: FIRMWARE HÍBRIDO BLINDADO - LUBRISENSE 5.2
 * PLATAFORMA: ESP32-S3 (Framework Arduino / PlatformIO)
 * VERSÃO: 5.2 FINAL (MONOPONTO MASTER + DEEP SLEEP + LÓGICA COMPLETA)
 * AUTOR: Equipe de Desenvolvimento Smart Factory - FREMTEC
 * =========================================================================================
 * * DESCRIÇÃO GERAL:
 * Firmware para controle de lubrificador automático inteligente com gestão avançada de energia.
 * O sistema permanece em modo de baixo consumo (Deep Sleep) na maior parte do tempo,
 * acordando apenas para eventos programados ou interação manual.
 *
 * * DESCRIÇÃO DE REDE (MONOPONTO):
 * - Endpoint (Este dispositivo): Configurado como MESTRE (ID 0).
 * - Gateway (Remoto): Deve ser configurado como ESCRAVO (ID 1).
 * - O Sniffer LoRa aceita apenas comandos vindos do ID 1 (Gateway).
 *
 * * ESTRATÉGIA DE ENERGIA:
 * - Deep Sleep ativo após 60s de inatividade (sem conexão BLE e sem motor rodando).
 * - Acorda por Botão Manual (GPIO 2) ou Timer de Ciclo Automático.
 *
 * * FUNCIONALIDADES (Mantidas da v4.7):
 * - Conexão Híbrida (LoRa + BLE).
 * - Buffer de recepção BLE para evitar erros de JSON fragmentado.
 * - Proteção de Motor (Corrente/Fim de Curso).
 * - Persistência de dados em SPIFFS.
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
#include <WiFi.h>          // Utilizado apenas para sincronização NTP no boot (se necessário)
#include <Wire.h>          // Comunicação I2C
#include <SPIFFS.h>        // Sistema de Arquivos (Config/Logs)
#include <ArduinoJson.h>   // Manipulação de dados JSON
#include "RTClib.h"        // Relógio de Tempo Real (DS3231)
#include <INA226.h>        // Sensor de Tensão e Corrente
#include <MotorEncoder.h>  // Biblioteca Local: Controle PID/Pulsos do Motor
#include "LoRaMESH.h"      // Biblioteca Local: Rádio (Apenas envio/configuração)
#include <esp_sleep.h>     // Gestão de Energia (Deep Sleep)

// Bibliotecas Nativas do ESP32 para Bluetooth (Estabilidade e Compatibilidade)
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --------------------------------------------------------------------------
// 2. DEFINIÇÕES DE HARDWARE (PINOUT)
// --------------------------------------------------------------------------
// Interface LoRa (Validado na PCI v4.0)
#define LORA_TX_PIN         43  
#define LORA_RX_PIN         44  

// Interface de Usuário e Sensores Digitais
#define LED_PIN_GPIO7       7   // LED de Status (Feedback Visual)
#define SEN_NIVEL_BAIXO     5   // Sensor de Fim de Curso (Reservatório Vazio)
#define SEN_NIVEL_ALTO      6   // Sensor de Fim de Curso (Reservatório Cheio)
#define BTN_MANUAL          2   // Botão de Ação Manual e Modo de Segurança

// Driver de Motor (Ponte H e Encoder)
#define MOTOR_IN1           21  
#define MOTOR_IN2           18  
#define MOTOR_EN            38  
#define ENCODER_PIN_A       15  
#define ENCODER_PIN_B       16  

// Barramentos de Comunicação
#define I2C_SDA             11  
#define I2C_SCL             10  
#define PT100_PIN           1   // Entrada Analógica (Temperatura)

// --------------------------------------------------------------------------
// 3. CONSTANTES E PARÂMETROS GLOBAIS
// --------------------------------------------------------------------------
// Identidade Bluetooth (Devem coincidir com o App Mobile)
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID           "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"

// IDs da Rede LoRa (TOPOLOGIA MONOPONTO MESTRE)
#define MEU_ID_LOCAL        0  // Endpoint é Mestre (0)
#define ID_REMOTO_GATEWAY   1  // Gateway é Escravo (1)

// Configuração Wi-Fi (Apenas para NTP)
const char* ssid = "smartfactory";      
const char* password = "smartfactory"; 
const char* ntpServer = "a.st1.ntp.br";
const long  gmtOffset_sec = -3 * 3600; // GMT-3 (Brasil)
const int   daylightOffset_sec = 0;

// Nomes dos Arquivos no Sistema SPIFFS
#define CONFIG_FILE "/config.json" // Configurações de operação
#define LOG_FILE    "/log.jsonl"   // Histórico de eventos
#define TEMP_FILE   "/temp.json"   // Estado temporário (recuperação de falha)

// Parâmetros de Rede LoRa
#define CMD_TO_GATEWAY_EVENT   0x20 
const size_t limiteLinhasLog = 10;
const unsigned long intervaloEnviarGateway = 30000; // Intervalo de reenvio de logs (30s)
const unsigned long TIMEOUT_ACK = 15000;            // Tempo limite para ACK do Gateway

// Configuração de Energia (Deep Sleep)
const unsigned long TEMPO_OCIOSO_MAXIMO = 60000; // 60s sem atividade = Dormir

// Parâmetros de Proteção e Calibração do Motor
const float CORRENTE_MAXIMA_MOTOR = 2000.0; // Limite de corrente em mA (Proteção)
const float SHUNT_MOTOR_OHMS = 0.02;        // Resistor Shunt da placa
const int VELOCIDADE_MOTOR = 60;            // Velocidade PWM (0-255)
const long PULSOS_PARA_UMA_VOLTA = 5650;    // Calibração: Pulsos por volta completa
const float GRAMAS_POR_VOLTA = 2.21;        // Calibração: Gramas dispensadas por volta

// Parâmetros de Sensores
const float TEMP_MIN = 0.0;       
const float TEMP_MAX = 150.0;
#define FATOR_CONVERSAO_PT100 100.0 
#define NUM_AMOSTRAS_ADC 20  

// --------------------------------------------------------------------------
// 4. OBJETOS E VARIÁVEIS DE ESTADO
// --------------------------------------------------------------------------
// Interfaces de Comunicação
HardwareSerial LoRaSerial(2); 
LoRaMESH lora(&LoRaSerial);   
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;

// Periféricos de Hardware
RTC_DS3231 rtc;               
INA226 ina226Bateria(0x40);   
INA226 ina226Motor(0x41);     
MotorEncoder Motor(ENCODER_PIN_A, ENCODER_PIN_B, MOTOR_EN, MOTOR_IN1, MOTOR_IN2);

// Variáveis RTC (Memória que sobrevive ao Deep Sleep)
RTC_DATA_ATTR int bootCount = 0; 

// Controle de Energia (Volátil)
unsigned long lastActivityTime = 0;

// Estado do Bluetooth (Buffer para remontagem de pacotes)
bool deviceConnected = false;      
bool oldDeviceConnected = false;   
String bleBuffer = "";             // Acumula fragmentos de JSON (Bufferização)
bool bleJsonCompleto = false;      // Flag: Indica que um JSON completo chegou
volatile bool estadoLedAtual = false; 
bool precisaNotificarApp = false;  // Flag: Indica necessidade de atualizar o App

// Estado do LoRa
unsigned long ultimoReenvio = 0;
bool aguardandoAck = false;
unsigned long aguardandoAckDesde = 0;
unsigned long ultimaSincronizacao = 0; 

// Variáveis de Configuração (Carregadas do JSON)
String varConfigUuid = "";
int    varConfigTipoConfig = 0;       // 1=Básico, 2=Avançado
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

// Controle do Botão Manual
int estadoAnteriorBtnManual = HIGH;
unsigned long ultimoAcionamentoBotaoManual = 0;
const unsigned long intervaloBloqueio = 5000; 

// --------------------------------------------------------------------------
// 5. PROTÓTIPOS DE FUNÇÕES
// --------------------------------------------------------------------------
// (Declaração antecipada para organização do código)
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
void resetInactivityTimer();
void entrarEmDeepSleep();

// ==========================================================================
// 6. IMPLEMENTAÇÃO DO BLUETOOTH (CALLBACKS)
// ==========================================================================

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      resetInactivityTimer(); // Reseta timer de sono ao conectar
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
        resetInactivityTimer(); // Reseta timer ao receber dados

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
  Serial.println(">> [BLE] PRONTO E VISÍVEL");
}

// ==========================================================================
// 7. LÓGICA LORA (SNIFFER MESTRE - OUVE ID 1)
// ==========================================================================
void verificarLoRaCirurgico() {
  if (LoRaSerial.available() > 0) {
    resetInactivityTimer();
    static uint8_t buffer[20];
    static int index = 0;
    uint8_t byteLido = LoRaSerial.read();

    // FILTRO: Aceita mensagens vindas do Gateway (ID 1)
    if (index == 0 && byteLido != ID_REMOTO_GATEWAY) return;

    buffer[index++] = byteLido;

    if (index >= 9) {
      if (buffer[2] == 0xC2) { 
         uint8_t nivel = buffer[6]; 
         Serial.printf("[LORA] Comando do Gateway (ID 1): %d\n", nivel);
         if (nivel == 1) { digitalWrite(LED_PIN_GPIO7, HIGH); updateBLE("1"); }
         else { digitalWrite(LED_PIN_GPIO7, LOW); updateBLE("0"); }
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
  bootCount++;
  Serial.printf("Boot number: %d\n", bootCount);

  // Configura Botão como Despertador (Wakeup Source)
  pinMode(BTN_MANUAL, INPUT_PULLUP);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_2, 0); 

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) Serial.println("[BOOT] Acordado por Botão.");
  else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) Serial.println("[BOOT] Acordado por Timer.");

  // Trava de Segurança (Recuperação de USB)
  if (digitalRead(BTN_MANUAL) == LOW && wakeup_reason != ESP_SLEEP_WAKEUP_EXT0) {
      Serial.println("\n!!! MODO DE SEGURANÇA ATIVADO !!!");
      while(1) { delay(100); }
  }
  
  delay(100); 

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
  if (!rtc.begin()) Serial.println("[ERRO] RTC fail");
  if (rtc.lostPower()) { if (sincronizarViaNTP()) ultimaSincronizacao = millis(); }
  isClockValid = (rtc.now().year() >= 2024);

  if (!SPIFFS.begin(true)) Serial.println("[ERRO] SPIFFS fail");
  inicializaConfig();
  if (!configExiste) {
    const char* def = "{\"Uuid\":\"novo\",\"Tag\":\"DEFAULT\",\"Equipamento\":\"MOTOR\",\"TipoConfig\":1,\"Volume\":10,\"Intervalo\":1,\"TipoIntervalo\":3,\"Frequencia\":1,\"TipoFrequencia\":2}";
    salvarConfig(def);
    configExiste = true;
  }
  inicializarArquivoTemp();

  // LORA MASTER (ID 0)
  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  lora.begin(true); 
  
  // CORREÇÃO: Atribuição direta da variável pública se o método setLocalId não existir
  if(lora.localId != MEU_ID_LOCAL) {
      Serial.printf("[LORA] Configurando ID %d (Master)...\n", MEU_ID_LOCAL);
      lora.setnetworkId(0);
      lora.setpassword(602);
      // lora.setLocalId(MEU_ID_LOCAL); // REMOVIDO (Método Inexistente)
      lora.localId = MEU_ID_LOCAL;      // ADICIONADO (Acesso Direto)
  }

  ina226Bateria.begin();
  ina226Motor.begin();
  Motor.begin(PULSOS_PARA_UMA_VOLTA, GRAMAS_POR_VOLTA);

  resetInactivityTimer(); 
  setupBLE();
  Serial.printf("--- ENDPOINT MASTER (ID %d) PRONTO ---\n", MEU_ID_LOCAL);
}

// ==========================================================================
// 9. LOOP PRINCIPAL
// ==========================================================================
void loop() {
  verificarLoRaCirurgico();

  if (Serial.available() > 0) {
    resetInactivityTimer();
    char c = Serial.read();
    while(Serial.available() > 0) Serial.read(); 
    if (c == 'c') displayFileContent(CONFIG_FILE);
    if (c == 'l') displayFileContent(LOG_FILE);
    if (c == 't') displayFileContent(TEMP_FILE);
  }

  if (bleJsonCompleto) {
      resetInactivityTimer();
      bleJsonCompleto = false; 
      String msg = bleBuffer;  
      bleBuffer = "";          

      if (msg.startsWith("{")) {
          delay(100);
          updateBLE("OK: Config Salva");
          delay(500); 
          salvarConfig(msg.c_str());
      }
      else if (msg == "1") { digitalWrite(LED_PIN_GPIO7, HIGH); updateBLE("1"); }
      else if (msg == "0") { digitalWrite(LED_PIN_GPIO7, LOW); updateBLE("0"); }
  }

  if (precisaNotificarApp) {
      precisaNotificarApp = false;
      if (deviceConnected) {
          resetInactivityTimer();
          updateBLE(estadoLedAtual ? "1" : "0");
          Serial.println("[SYNC] Status enviado ao App.");
      }
  }

  if (isClockValid && configExiste) {
      int btn = digitalRead(BTN_MANUAL);
      if (configExiste && btn == LOW && estadoAnteriorBtnManual == HIGH && (millis() - ultimoAcionamentoBotaoManual > intervaloBloqueio)){
        resetInactivityTimer();
        fonteDoEventoAtual = "Manual";
        float dose = 0.0f;
        if (varConfigTipoConfig == 2) dose = (float)varConfigVolume;
        else if (varConfigTipoConfig == 1) { 
             float diasTotal = converterParaDias(varConfigValorIntervalo, varConfigTipoIntervalo);
             float diasFreq = converterParaDias(varConfigFrequencia, varConfigTipoFrequencia);
             if (diasFreq > 0) dose = (float)varConfigVolume / (diasTotal / diasFreq);
        } 
        if (dose > 0) iniciarMovimentoMotor(dose, HORARIO);
        ultimoAcionamentoBotaoManual = millis();
        salvarEstadoCiclo(true, formatarTimestamp(rtc.now()), formatarTimestamp(rtc.now()));
      }
      estadoAnteriorBtnManual = btn;

      if (varTempCicloStartado) {
        DateTime agora = rtc.now();
        DateTime ultima = stringParaDateTime(varTempUltimaLubrificacao);
        if (agora.unixtime() >= (ultima.unixtime() + (varConfigIntervaloTrigger_ms/1000))) {
          resetInactivityTimer(); 
          iniciarMovimentoMotor((float)varConfigVolume, HORARIO);
          varTempUltimaLubrificacao = formatarTimestamp(agora);
          salvarEstadoCiclo(true, varTempHorarioStartado, varTempUltimaLubrificacao);
        }
      }

      if (motorLigado) {
        resetInactivityTimer();
        float corrente = (ina226Motor.getShuntVoltage_mV() / SHUNT_MOTOR_OHMS);
        if (corrente > CORRENTE_MAXIMA_MOTOR) { 
            pararMotor(); 
            registrarEvento(fonteDoEventoAtual, false); 
        }
        if (motorDirecaoAtual == HORARIO && digitalRead(SEN_NIVEL_BAIXO) == LOW) { 
            pararMotor(); 
            registrarEvento(fonteDoEventoAtual, false); 
        }
        if (Motor.atualizar() == MOTOR_ALVO_ATINGIDO) {
           registrarEvento(fonteDoEventoAtual, true);
           motorLigado = false;
        }
      }
  }

  if (millis() - ultimoReenvio > intervaloEnviarGateway) {
    ultimoReenvio = millis();
    if (isClockValid && !aguardandoAck) tentarReenvio();
  }

  if (!deviceConnected && oldDeviceConnected) {
      delay(500); pServer->startAdvertising(); oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
      resetInactivityTimer();
  }

  if (millis() - lastActivityTime > TEMPO_OCIOSO_MAXIMO) {
      if (!deviceConnected && !motorLigado) {
          entrarEmDeepSleep();
      }
  }

  delay(20); 
}

// ==========================================================================
// 10. FUNÇÕES AUXILIARES
// ==========================================================================

void resetInactivityTimer() {
    lastActivityTime = millis();
}

void entrarEmDeepSleep() {
    Serial.println("[SLEEP] Entrando em Deep Sleep...");
    Serial.flush();
    
    // Configura despertar por botão
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_2, 0); 

    // Configura despertar por timer
    if (configExiste && varTempCicloStartado) {
        DateTime agora = rtc.now();
        DateTime ultima = stringParaDateTime(varTempUltimaLubrificacao);
        long intervaloSegundos = varConfigIntervaloTrigger_ms / 1000;
        long proximaDose = ultima.unixtime() + intervaloSegundos;
        long segundosAteAcordar = proximaDose - agora.unixtime();

        if (segundosAteAcordar > 0) {
            esp_sleep_enable_timer_wakeup(segundosAteAcordar * 1000000ULL);
            Serial.printf("[SLEEP] Acordarei em %ld segundos.\n", segundosAteAcordar);
        } else {
             // Se já passou da hora, dorme pouco para processar logo
             esp_sleep_enable_timer_wakeup(10 * 1000000ULL); 
        }
    }

    esp_deep_sleep_start();
}

void updateBLE(const String& value) {
    if (deviceConnected && pCharacteristic) {
        pCharacteristic->setValue(value.c_str());
        pCharacteristic->notify();
    }
}

void sendJsonViaLoRa(const String &json, uint8_t command) {
    if (lora.PrepareFrameCommand(ID_REMOTO_GATEWAY, command, (uint8_t*)json.c_str(), json.length())) 
        lora.SendPacket(); 
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

        // Verifica se veio no formato { "comando": "...", "payload": { ... } }
        if (!doc["payload"].isNull()) {
            JsonObject payload = doc["payload"];
            if (!payload["Volume"].isNull()) varConfigVolume = payload["Volume"].as<int>();

            if (!payload["Intervalo"].isNull()) {
                 varConfigValorIntervalo = payload["Intervalo"].as<unsigned long>();
                 varConfigTipoIntervalo = payload["TipoIntervalo"].as<int>();
                 
                 unsigned long mult = 1000;
                 if (varConfigTipoIntervalo == 2) mult = 86400000UL;
                 else if (varConfigTipoIntervalo == 3) mult = 2592000000UL;
                 else if (varConfigTipoIntervalo == 1) mult = 3600000UL;
                 
                 varConfigIntervalo = varConfigValorIntervalo * mult;
            }

            if (!payload["Frequencia"].isNull()) {
                 varConfigFrequencia = payload["Frequencia"].as<unsigned long>();
                 varConfigTipoFrequencia = payload["TipoFrequencia"].as<int>();
                 
                 unsigned long mult = 1000;
                 if (varConfigTipoFrequencia == 2) mult = 86400000UL;
                 else if (varConfigTipoFrequencia == 3) mult = 2592000000UL;
                 else if (varConfigTipoFrequencia == 1) mult = 3600000UL;
                 
                 varConfigIntervaloTrigger_ms = varConfigFrequencia * mult;
            }
        } 
        // Fallback para JSON direto (formato legado)
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
        if (millis() - aguardandoAckDesde > TIMEOUT_ACK) aguardandoAck = false;
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
    if (p > 100) p = 100;
    if (p < 0) p = 0;
    return p;
}

float lerTemperaturaPT100() {
    long total = 0;
    for (int i = 0; i < NUM_AMOSTRAS_ADC; i++) {
        total += analogReadMilliVolts(PT100_PIN);
        delay(1);
    }
    float mV = total / (float)NUM_AMOSTRAS_ADC;
    float mA = (mV / 1000.0) / FATOR_CONVERSAO_PT100 * 1000.0;
    float temp = (mA - 4.0) * (TEMP_MAX - TEMP_MIN) / (20.0 - 4.0) + TEMP_MIN;
    if (temp < TEMP_MIN) temp = TEMP_MIN;
    if (temp > TEMP_MAX) temp = TEMP_MAX;
    return temp;
} 
void marcarEventoComoEnviado() {}