/**
 * =========================================================================================
 * PROJETO: FIRMWARE HÍBRIDO BLINDADO - LUBRISENSE 4.0
 * PLATAFORMA: ESP32-S3 (Framework Arduino)
 * =========================================================================================
 * * DESCRIÇÃO:
 * Firmware para controle de lubrificador automático inteligente com conectividade dupla:
 * 1. LoRaMESH (Longa Distância): Recebe comandos via Gateway e envia telemetria.
 * 2. Bluetooth LE (Local): Permite configuração e controle via App Android/iOS.
 * * DESTAQUES TÉCNICOS:
 * - Arquitetura Híbrida: LoRa e BLE funcionam simultaneamente sem bloqueio (Non-blocking Loop).
 * - LoRa Cirúrgico: Implementação de Sniffer Serial para contornar limitações da biblioteca padrão.
 * - BLE Nativo: Uso direto da stack ESP32 BLE para máxima compatibilidade e reconexão robusta.
 * - Sistema de Arquivos (SPIFFS): Persistência de configurações (JSON) e Logs (JSONL).
 * - Proteção de Hardware: Monitoramento de corrente (INA226) e Watchdog de software.
 * - Modo de Segurança: Trava de boot via botão físico para recuperar USB em caso de crash.
 * * PINAGEM CRÍTICA (ESP32-S3):
 * [LoRa]   TX: GPIO 43 | RX: GPIO 44
 * [Motor]  EN: 38 | IN1: 21 | IN2: 18 | Encoder A: 15 | Encoder B: 16
 * [I/O]    LED Status: 7 | Botão Manual: 2 | Nível Baixo: 5 | Nível Alto: 6
 * [I2C]    SDA: 11 | SCL: 10 (INA226, RTC DS3231)
 * [Analog] PT100: 1
 * * =========================================================================================
 */

// --------------------------------------------------------------------------
// 1. INCLUSÃO DE BIBLIOTECAS
// --------------------------------------------------------------------------
#include <Arduino.h>
#include <vector>
#include <WiFi.h>          // Apenas para NTP (Relógio)
#include <Wire.h>          // I2C
#include <SPIFFS.h>        // Armazenamento Flash
#include <ArduinoJson.h>   // Parser JSON
#include "RTClib.h"        // RTC DS3231
#include <INA226.h>        // Sensor de Corrente
#include <MotorEncoder.h>  // Controle PID/Pulsos do Motor
#include "LoRaMESH.h"      // Rádio (Apenas envio/configuração)

// BLE Nativo (Estabilidade e Compatibilidade)
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --------------------------------------------------------------------------
// 2. DEFINIÇÕES DE HARDWARE (PINOUT)
// --------------------------------------------------------------------------
#define LORA_TX_PIN         43  
#define LORA_RX_PIN         44  

#define LED_PIN_GPIO7       7   // Feedback Visual
#define SEN_NIVEL_BAIXO     5   // Sensor Fim de Curso (Vazio)
#define SEN_NIVEL_ALTO      6   // Sensor Fim de Curso (Cheio)
#define BTN_MANUAL          2   // Botão de Ação / Modo Segurança

// Driver Motor (Ponte H)
#define MOTOR_IN1           21  
#define MOTOR_IN2           18  
#define MOTOR_EN            38  
#define ENCODER_PIN_A       15  
#define ENCODER_PIN_B       16  

// Barramentos
#define I2C_SDA             11  
#define I2C_SCL             10  
#define PT100_PIN           1   

// --------------------------------------------------------------------------
// 3. CONSTANTES E PARÂMETROS
// --------------------------------------------------------------------------
// Identidade Bluetooth (Deve coincidir com o App)
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID           "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"

// Wi-Fi (Apenas para NTP no boot)
const char* ssid = "smartfactory";      
const char* password = "smartfactory"; 
const char* ntpServer = "a.st1.ntp.br";
const long  gmtOffset_sec = -3 * 3600; // GMT-3 (Brasil)
const int   daylightOffset_sec = 0;

// Sistema de Arquivos
#define CONFIG_FILE "/config.json" 
#define LOG_FILE    "/log.jsonl"   
#define TEMP_FILE   "/temp.json"   

// LoRa e Rede
#define GATEWAY_ID 0 
#define CMD_TO_GATEWAY_EVENT   0x20 
const size_t limiteLinhasLog = 10;
const unsigned long intervaloEnviarGateway = 30000; // 30s
const unsigned long TIMEOUT_ACK = 15000;

// Proteção do Motor
const float CORRENTE_MAXIMA_MOTOR = 2000.0; // mA
const float SHUNT_MOTOR_OHMS = 0.02;        
const int VELOCIDADE_MOTOR = 60;            // PWM
const long PULSOS_PARA_UMA_VOLTA = 5650;    
const float GRAMAS_POR_VOLTA = 2.21;        

// Sensores
const float TEMP_MIN = 0.0;       
const float TEMP_MAX = 150.0;
#define FATOR_CONVERSAO_PT100 100.0 
#define NUM_AMOSTRAS_ADC 20  

// --------------------------------------------------------------------------
// 4. OBJETOS GLOBAIS E VARIÁVEIS DE ESTADO
// --------------------------------------------------------------------------
// Comunicação
HardwareSerial LoRaSerial(2); 
LoRaMESH lora(&LoRaSerial);   
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;

// Hardware
RTC_DS3231 rtc;               
INA226 ina226Bateria(0x40);   
INA226 ina226Motor(0x41);     
MotorEncoder Motor(ENCODER_PIN_A, ENCODER_PIN_B, MOTOR_EN, MOTOR_IN1, MOTOR_IN2);

// Controle de Estado BLE
bool deviceConnected = false;      
bool oldDeviceConnected = false;   
String bleComandoRecebido = "";    
volatile bool bleTemDados = false; 
volatile bool estadoLedAtual = false; 
bool precisaNotificarApp = false;  

// Controle de Estado LoRa
unsigned long ultimoReenvio = 0;
bool aguardandoAck = false;
unsigned long aguardandoAckDesde = 0;
unsigned long ultimaSincronizacao = 0; 

// Configuração (Carregada do JSON)
String varConfigUuid = "";
int    varConfigTipoConfig = 0; // 1=Básico, 2=Avançado
int    varConfigVolume = 10;          
unsigned long varConfigIntervaloTrigger_ms = 0; 
unsigned long varConfigIntervalo = 0; 
unsigned long varConfigValorIntervalo = 0; 
int varConfigTipoIntervalo = 0;
int varConfigFrequencia = 0;
int varConfigTipoFrequencia = 0;
bool configExiste = false;

// Estado do Sistema
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

// ==========================================================================
// 6. IMPLEMENTAÇÃO DO BLUETOOTH (CALLBACKS)
// ==========================================================================

// Gerencia conexão/desconexão
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println(">> [BLE] CELULAR CONECTADO!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println(">> [BLE] DESCONECTADO. Reiniciando Anuncio...");
      pServer->startAdvertising(); // Reinício imediato para reconexão rápida
    }
};

// Gerencia escrita de dados pelo celular
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      String rxValue = String(value.c_str());

      if (rxValue.length() > 0) {
        bleComandoRecebido = rxValue;
        bleTemDados = true; // Sinaliza ao loop principal
      }
    }
};

// Inicialização do Stack BLE
void setupBLE() {
  BLEDevice::init("LUBRICENSE_Device"); // Nome visível no scan
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHAR_UUID,
                      BLECharacteristic::PROPERTY_READ | 
                      BLECharacteristic::PROPERTY_WRITE | 
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  
  // Configuração de Advertising para melhor compatibilidade Android/iOS
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); 
  pAdvertising->setMaxPreferred(0x12);
  
  BLEDevice::startAdvertising();
  
  Serial.println(">> [BLE] PRONTO E VISÍVEL (LUBRICENSE_Device)");
}

// ==========================================================================
// 7. IMPLEMENTAÇÃO DO LORA (SNIFFER CIRÚRGICO)
// ==========================================================================
/**
 * Lê o buffer serial byte a byte procurando pelo padrão de comando GPIO (0xC2).
 * Resolve o problema de travamento da biblioteca oficial em loops rápidos.
 */
void verificarLoRaCirurgico() {
  if (LoRaSerial.available() > 0) {
    static uint8_t buffer[20];
    static int index = 0;
    
    uint8_t byteLido = LoRaSerial.read();

    // Sincronia: Espera cabeçalho ID 1
    if (index == 0 && byteLido != 0x01) return; 

    buffer[index++] = byteLido;

    // Tamanho do pacote de comando = 9 bytes
    if (index >= 9) {
      // Byte 2 = Comando (0xC2 = GPIO)
      if (buffer[2] == 0xC2) {
         // Byte 6 = Nível Lógico (1=High, 0=Low)
         uint8_t nivel = buffer[6]; 
         
         Serial.printf("[LORA] Comando Recebido! Nivel: %d\n", nivel);
         
         // Ação Imediata no Hardware
         if (nivel == 1) { 
             digitalWrite(LED_PIN_GPIO7, HIGH); 
             estadoLedAtual = true; 
         } else { 
             digitalWrite(LED_PIN_GPIO7, LOW); 
             estadoLedAtual = false; 
         }
         
         // Solicita notificação ao App (Processado no Loop)
         precisaNotificarApp = true;
      }
      index = 0; 
    }
  }
}

// ==========================================================================
// 8. SETUP (INICIALIZAÇÃO)
// ==========================================================================
void setup() {
  Serial.begin(115200);
  
  // Configura botão manual primeiro para permitir Modo de Segurança
  pinMode(BTN_MANUAL, INPUT_PULLUP);

  // --- TRAVA DE SEGURANÇA (RECUPERAÇÃO DE USB) ---
  // Se ligar segurando o botão, entra em loop infinito seguro
  if (digitalRead(BTN_MANUAL) == LOW) {
      Serial.println("\n!!! MODO DE SEGURANÇA ATIVADO !!!");
      Serial.println("Codigo principal cancelado. USB ativa para upload.");
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

  // Estado Inicial
  digitalWrite(MOTOR_EN, LOW);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(LED_PIN_GPIO7, LOW);

  // Correção ADC ESP32-S3
  if (PT100_PIN >= 0) analogSetAttenuation(ADC_11db);

  // Inicialização I2C e RTC
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!rtc.begin()) Serial.println("[ERRO] RTC não encontrado");
  
  // Recuperação de Hora via NTP se necessário
  if (rtc.lostPower()) {
    Serial.println("RTC sem hora. Tentando sincronizar via Wi-Fi...");
    if (sincronizarViaNTP()) ultimaSincronizacao = millis();
  }
  
  // Validação de Relógio
  DateTime agora = rtc.now();
  if (agora.year() < 2024) isClockValid = false;
  else isClockValid = true;

  // Inicialização SPIFFS
  if (!SPIFFS.begin(true)) Serial.println("[ERRO] SPIFFS falhou");
  else Serial.println("[OK] Sistema de Arquivos montado");
  
  inicializaConfig();
  
  // Configuração de Fábrica (Se novo)
  if (!configExiste) {
    Serial.println("[INIT] Criando configuração de fábrica...");
    const char* def = "{\"Uuid\":\"novo-uuid\",\"Tag\":\"PONTO-01\",\"Equipamento\":\"MOTOR\",\"TipoConfig\":1,\"Volume\":10,\"Intervalo\":1,\"TipoIntervalo\":3,\"Frequencia\":1,\"TipoFrequencia\":2}";
    salvarConfig(def);
    configExiste = true;
  }

  inicializarArquivoTemp();

  // Inicialização LoRa (Apenas TX configurado via Lib)
  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  lora.begin(false); 
  
  // Força ID Mestre
  if(lora.localId != 0) {
      lora.setnetworkId(0);
      lora.setpassword(602);
  }

  // Inicialização Sensores e Motor
  ina226Bateria.begin();
  ina226Motor.begin();
  ina226Motor.setAverage(128); 
  Motor.begin(PULSOS_PARA_UMA_VOLTA, GRAMAS_POR_VOLTA);

  // Inicialização BLE com Delay de Segurança
  Serial.println("Iniciando BLE em 2s...");
  delay(2000);
  setupBLE();

  Serial.println("--- SISTEMA HIBRIDO PRONTO ---");
  Serial.println("DIAGNOSTICO: Envie 'c' (config), 'l' (log), 't' (temp)");
}

// ==========================================================================
// 9. LOOP PRINCIPAL (GERENCIADOR DE TAREFAS)
// ==========================================================================
void loop() {
  
  // --- TAREFA 1: VERIFICAR RÁDIO LORA ---
  verificarLoRaCirurgico();

  // --- TAREFA 2: DIAGNÓSTICO SERIAL (PC) ---
  if (Serial.available() > 0) {
    char command = Serial.read();
    while(Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
        Serial.read(); // Limpa buffer
    }

    if (command == 'c' || command == 'C') displayFileContent(CONFIG_FILE);
    else if (command == 'l' || command == 'L') displayFileContent(LOG_FILE);
    else if (command == 't' || command == 'T') displayFileContent(TEMP_FILE);
  }

  // --- TAREFA 3: PROCESSAR COMANDOS BLUETOOTH ---
  if (bleTemDados) {
      bleTemDados = false; 
      Serial.print("[BLE] Recebido: ");
      Serial.println(bleComandoRecebido);

      if (bleComandoRecebido.startsWith("{")) {
          salvarConfig(bleComandoRecebido.c_str());
          updateBLE("OK: Config Salva");
      }
      else if (bleComandoRecebido == "1") {
          digitalWrite(LED_PIN_GPIO7, HIGH);
          estadoLedAtual = true;
          updateBLE("1"); 
      }
      else if (bleComandoRecebido == "0") {
          digitalWrite(LED_PIN_GPIO7, LOW);
          estadoLedAtual = false;
          updateBLE("0"); 
      }
  }

  // --- TAREFA 4: SINCRONIZAR ESTADO (LoRa -> App) ---
  if (precisaNotificarApp) {
      precisaNotificarApp = false;
      if (deviceConnected) {
          updateBLE(estadoLedAtual ? "1" : "0");
          Serial.println("[SYNC] Status enviado ao App.");
      }
  }

  // --- TAREFA 5: LÓGICA DE NEGÓCIO AUTOMÁTICA ---
  if (!isClockValid) return;

  // 5.1 Cálculo de Dose
  float doseCalculada = 0.0f;
  if (configExiste) {
    if (varConfigTipoConfig == 2) {
      doseCalculada = (float)varConfigVolume;
    }
    else if (varConfigTipoConfig == 1) { 
      float diasTotal = converterParaDias(varConfigValorIntervalo, varConfigTipoIntervalo);
      float diasFreq = converterParaDias(varConfigFrequencia, varConfigTipoFrequencia);
      if (diasFreq > 0) {
        float numDoses = diasTotal / diasFreq;
        if (numDoses > 0) doseCalculada = (float)varConfigVolume / numDoses;
      }
    } 
  }

  // 5.2 Botão Manual
  int btn = digitalRead(BTN_MANUAL);
  if (configExiste && btn == LOW && estadoAnteriorBtnManual == HIGH && (millis() - ultimoAcionamentoBotaoManual > intervaloBloqueio)){
    Serial.println("[MANUAL] Ciclo Iniciado via Botão");
    fonteDoEventoAtual = "Manual";
    if (doseCalculada > 0) iniciarMovimentoMotor(doseCalculada, HORARIO);
    ultimoAcionamentoBotaoManual = millis();
    salvarEstadoCiclo(true, formatarTimestamp(rtc.now()), formatarTimestamp(rtc.now()));
  }
  estadoAnteriorBtnManual = btn;

  // 5.3 Ciclo Automático (Relógio)
  if (configExiste && varTempCicloStartado) {
    DateTime agora = rtc.now();
    DateTime ultima = stringParaDateTime(varTempUltimaLubrificacao);
    
    if (agora.unixtime() >= (ultima.unixtime() + (varConfigIntervaloTrigger_ms/1000))) {
      Serial.println("[AUTO] Hora de Lubrificar!");
      fonteDoEventoAtual = "Auto";
      if (doseCalculada > 0) iniciarMovimentoMotor(doseCalculada, HORARIO);
      varTempUltimaLubrificacao = formatarTimestamp(agora);
      salvarEstadoCiclo(true, varTempHorarioStartado, varTempUltimaLubrificacao);
    }
  }

  // 5.4 Gerenciamento do Motor (Proteções)
  if (motorLigado) {
    float correnteMotor = (ina226Motor.getShuntVoltage_mV() / SHUNT_MOTOR_OHMS);
    if (correnteMotor > CORRENTE_MAXIMA_MOTOR) {
      Serial.println("[MOTOR] ERRO: Sobrecorrente!");
      pararMotor();
      registrarEvento(fonteDoEventoAtual, false);
    }
    if (motorDirecaoAtual == HORARIO && digitalRead(SEN_NIVEL_BAIXO) == LOW) {
       Serial.println("[MOTOR] ERRO: Nível Baixo!");
       pararMotor();
       registrarEvento(fonteDoEventoAtual, false);
    }
    if (Motor.atualizar() == MOTOR_ALVO_ATINGIDO) {
      Serial.println("[MOTOR] Dosagem concluída.");
      registrarEvento(fonteDoEventoAtual, true);
      motorLigado = false;
    }
  }

  // --- TAREFA 6: MANUTENÇÃO ---
  // Reenvio LoRa
  if (millis() - ultimoReenvio > intervaloEnviarGateway) {
    ultimoReenvio = millis();
    if (isClockValid && !aguardandoAck) tentarReenvio();
  }

  // Reconexão BLE
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); 
      pServer->startAdvertising(); 
      Serial.println("[BLE] Reiniciando Anuncio...");
      oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
  }

  delay(5); // Yield para Watchdog
}

// ==========================================================================
// 10. FUNÇÕES AUXILIARES
// ==========================================================================

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
    JsonDocument doc;
    deserializeJson(doc, json);
    if (!doc["Volume"].isNull()) varConfigVolume = doc["Volume"].as<int>();
    if (!doc["Intervalo"].isNull()) {
        varConfigValorIntervalo = doc["Intervalo"].as<unsigned long>();
        varConfigTipoIntervalo = doc["TipoIntervalo"].as<int>();
        unsigned long mult = 1000;
        if(varConfigTipoIntervalo == 2) mult = 86400000UL; 
        else if(varConfigTipoIntervalo == 3) mult = 2592000000UL; 
        else if(varConfigTipoIntervalo == 1) mult = 3600000UL; 
        varConfigIntervalo = varConfigValorIntervalo * mult;
    }
    if (!doc["Frequencia"].isNull()) {
        varConfigFrequencia = doc["Frequencia"].as<unsigned long>();
        varConfigTipoFrequencia = doc["TipoFrequencia"].as<int>();
        unsigned long mult = 1000;
        if(varConfigTipoFrequencia == 2) mult = 86400000UL;
        else if(varConfigTipoFrequencia == 3) mult = 2592000000UL;
        else if(varConfigTipoFrequencia == 1) mult = 3600000UL;
        varConfigIntervaloTrigger_ms = varConfigFrequencia * mult;
    }
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