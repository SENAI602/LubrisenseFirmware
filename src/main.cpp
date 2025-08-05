#include <Arduino.h>
#include "LoRaMESH.h"      // Biblioteca de comunicação LoRa
#include <SPIFFS.h>       // Para o sistema de arquivos do ESP32
#include <ArduinoJson.h>  // Para manipulação de JSON
#include <vector>         // Adicionado para gerenciar o log em memória
#include "BleService.h"    // Inclui a "ponte" para o nosso serviço BLE

// --- Configuração dos Pinos ---
#define LORA_TX_PIN 17
#define LORA_RX_PIN 16
#define TEMP_PIN    34         // Pino para ler a temperatura
#define DIGITAL_OUT 26         // Led para teste
#define MANUAL_TRIGGER_PIN 4   // Novo pino para o botão de disparo manual

// --- Configurações da Rede LoRa ---
#define GATEWAY_ID 0

// --- Definições dos Comandos LoRa ---
#define CMD_FROM_MASTER_CONFIG 0x10
#define CMD_FROM_MASTER_ACK    0x11
#define CMD_TO_GATEWAY_EVENT   0x20

// --- Configuração dos Arquivos e Constantes ---
#define CONFIG_FILE "/config.json"
#define LOG_FILE    "/log.jsonl"
const size_t LIMITE_LINHAS_LOG = 10;
const char* TIMESTAMP_CONSTANT = "2025-07-02T15:00:00Z";
unsigned long intervaloEnviarGateway = 15000;

// Valor padrão:
unsigned long intervalo = 15000;
int configVolume = 10; 

// --- Variáveis Globais de Controle ---
unsigned long ultimaLubrificacao = 0;
unsigned long ultimoReenvio = 0;
bool aguardandoAck = false;
unsigned long aguardandoAckDesde = 0;
const unsigned long TIMEOUT_ACK = 15000;
int lastManualButtonState = HIGH;

// --- Variáveis Globais para Comunicação com BLE (Definidas aqui) ---
volatile bool newDataFromBLE = false;
String bleReceivedValue = "";

// --- Objetos e Instâncias ---
HardwareSerial LoRaSerial(2);
LoRaMESH lora(&LoRaSerial);

// --- Protótipos de Funções (do próprio arquivo) ---
void sendJsonViaLoRa(const String &json, uint8_t command);
void salvarConfig(const char* json);
void registrarEvento(const String& fonte);
void tentarReenvio();
void marcarEventoComoEnviado();
void displayFileContent(const char* filename);
void loadInitialConfig();

void setup() {
  delay(1000);
  Serial.begin(115200);
  delay(1000);
  pinMode(DIGITAL_OUT, OUTPUT);
  digitalWrite(DIGITAL_OUT, LOW);
  pinMode(MANUAL_TRIGGER_PIN, INPUT_PULLUP);

  Serial.println("\n--- [SLAVE] Inicializando EndPoint com LoRa e BLE ---");
  Serial.println("--- Envie 'c' para ver config.json ou 'l' para ver log.jsonl ---");

  if (!SPIFFS.begin(true)) {
    Serial.println("[ERRO] Falha ao montar o SPIFFS.");
    while (true);
  }
  Serial.println("[OK] Sistema de arquivos montado.");
  loadInitialConfig();

  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  lora.begin();
  lora.deviceId = lora.localId;
  lora.debug_serial = false;
  Serial.print("[OK] Módulo LoRa iniciado. ID Local: ");
  Serial.println(lora.localId);
  
  Serial.println("Iniciando ESP32 com BLE...");
  setupBLE(); // Chama a função de setup que está no BleService.cpp
}


void loop() {
  // Seção de verificação de dados do Bluetooth
  if (newDataFromBLE) {
    newDataFromBLE = false; // Reseta a flag para não processar de novo
    Serial.println("\n[BLE] Novos dados de configuração recebidos via Bluetooth!");
    salvarConfig(bleReceivedValue.c_str());
  }

  // 1. Lógica de Recepção de Comandos LoRa
  uint16_t senderId;
  uint8_t receivedCommand;
  uint8_t payload[MAX_PAYLOAD_SIZE];
  uint8_t payloadSize;

  if (lora.ReceivePacketCommand(&senderId, &receivedCommand, payload, &payloadSize, 100)) {
    payload[payloadSize] = '\0';
    String msg = (char*)payload;
    Serial.print("[LORA] Comando recebido do ID ");
    Serial.print(senderId);
    Serial.print(" - CMD: 0x");
    Serial.print(receivedCommand, HEX);
    Serial.print(" - Payload: ");
    Serial.println(msg);

    switch (receivedCommand) {
      case CMD_FROM_MASTER_ACK:
        marcarEventoComoEnviado();
        break;
      case CMD_FROM_MASTER_CONFIG:
        salvarConfig(msg.c_str());
        break;
      default:
        Serial.println("[AVISO] Comando LoRa desconhecido.");
        break;
    }
  }

  // 2. Lógica de Registro Automático por tempo
  if (millis() - ultimaLubrificacao >= intervalo) {
    Serial.println("\n[EVENTO] Intervalo de tempo atingido. Registrando (auto)...");
    digitalWrite(DIGITAL_OUT, HIGH);
    delay(500);
    digitalWrite(DIGITAL_OUT, LOW);
    registrarEvento("auto");
    ultimaLubrificacao = millis();
  }

  // 3. Lógica de Registro Manual por botão
  int manualButtonState = digitalRead(MANUAL_TRIGGER_PIN);
  if (manualButtonState == LOW && lastManualButtonState == HIGH) {
    Serial.println("\n[EVENTO] Botão manual pressionado. Registrando (manual)...");
    digitalWrite(DIGITAL_OUT, HIGH);
    delay(500);
    digitalWrite(DIGITAL_OUT, LOW);
    registrarEvento("manual");
    delay(200); // Debounce
  }
  lastManualButtonState = manualButtonState;

  // 4. Lógica de Reenvio de Eventos
  if (millis() - ultimoReenvio > intervaloEnviarGateway) {
    tentarReenvio();
    ultimoReenvio = millis();
  }

  // 5. Lógica para exibir arquivos via Monitor Serial
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'c' || command == 'C') {
      displayFileContent(CONFIG_FILE);
    } else if (command == 'l' || command == 'L') {
      displayFileContent(LOG_FILE);
    }
  }
}

void loadInitialConfig() {
  if (SPIFFS.exists(CONFIG_FILE)) {
    File file = SPIFFS.open(CONFIG_FILE, "r");
    if (file) {
      Serial.println("[INIT] Arquivo de configuração encontrado. Carregando dados.");
      String configContent = file.readString();
      file.close();
      salvarConfig(configContent.c_str());
    } else {
      Serial.println("[ERRO] Não foi possível ler o config na inicialização.");
    }
  } else {
    Serial.println("[INIT] Nenhum config encontrado.");
  }
}

void sendJsonViaLoRa(const String &json, uint8_t command) {
  Serial.print("[LORA] Enviando para Gateway - CMD: 0x");
  Serial.print(command, HEX);
  Serial.print(" - JSON: ");
  Serial.println(json);
  if (lora.PrepareFrameCommand(GATEWAY_ID, command, (uint8_t*)json.c_str(), json.length())) {
    lora.SendPacket();
  } else {
    Serial.println("[ERRO] Falha ao preparar frame LoRa.");
  }
}

void salvarConfig(const char* json) {
  Serial.println("[CONFIG] Aplicando e salvando configuração...");
  File f = SPIFFS.open(CONFIG_FILE, "w");
  if (!f) {
    Serial.println("[ERRO] Falha ao abrir config para escrita.");
    return;
  }
  f.print(json);
  f.close();
  Serial.println("[CONFIG] Arquivo salvo com sucesso.");

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    Serial.print("[ERRO] Falha ao interpretar JSON de config: ");
    Serial.println(err.c_str());
    return;
  }
}

void registrarEvento(const String& fonte) {
  std::vector<String> lines;
  File readFile = SPIFFS.open(LOG_FILE, "r");
  if (readFile) {
    while (readFile.available()) {
      String line = readFile.readStringUntil('\n');
      if (line.length() > 0) lines.push_back(line);
    }
    readFile.close();
  }

  while (lines.size() >= LIMITE_LINHAS_LOG) {
    lines.erase(lines.begin());
  }

  JsonDocument doc;
  doc["t"] = analogRead(TEMP_PIN);
  doc["v"] = configVolume;
  doc["d"] = TIMESTAMP_CONSTANT;
  doc["f"] = fonte;
  doc["e"] = false;

  String newLogLine;
  serializeJson(doc, newLogLine);
  lines.push_back(newLogLine);

  File writeFile = SPIFFS.open(LOG_FILE, "w");
  if (!writeFile) {
    Serial.println("[ERRO] Falha ao abrir log para escrita.");
    return;
  }
  for (const auto& line : lines) {
    writeFile.println(line);
  }
  writeFile.close();

  Serial.print("[LOG] Novo evento registrado. O log agora contém ");
  Serial.print(lines.size());
  Serial.println(" linhas.");
  Serial.print("   > Novo dado: ");
  Serial.println(newLogLine);
}

void tentarReenvio() {
  if (aguardandoAck && (millis() - aguardandoAckDesde < TIMEOUT_ACK)) {
    return;
  }
  aguardandoAck = false;

  File file = SPIFFS.open(LOG_FILE, "r");
  if (!file || file.size() == 0) {
    if(file) file.close();
    return;
  }

  while (file.available()) {
    String line = file.readStringUntil('\n');
    if (line.length() == 0) continue;

    JsonDocument doc;
    deserializeJson(doc, line);

    if (doc["e"] == false) {
      doc["e"] = true;
      String jsonOutput;
      serializeJson(doc, jsonOutput);
      sendJsonViaLoRa(jsonOutput, CMD_TO_GATEWAY_EVENT);
      aguardandoAck = true;
      aguardandoAckDesde = millis();
      break;
    }
  }
  file.close();
}

void marcarEventoComoEnviado() {
    if (!aguardandoAck) return;
    Serial.println("[ACK] ACK recebido! Marcando evento como enviado.");
    aguardandoAck = false;

    File logFile = SPIFFS.open(LOG_FILE, "r");
    File tempFile = SPIFFS.open("/temp_log.jsonl", "w");
    if (!logFile || !tempFile) {
        Serial.println("[ERRO] Falha ao abrir arquivos para atualização do log.");
        if(logFile) logFile.close();
        if(tempFile) tempFile.close();
        return;
    }

    bool ackAplicado = false;
    while (logFile.available()) {
        String line = logFile.readStringUntil('\n');
        if (line.length() == 0) continue;
        JsonDocument doc;
        deserializeJson(doc, line);
        if (!ackAplicado && doc["e"] == false) {
            doc["e"] = true;
            ackAplicado = true;
        }
        serializeJson(doc, tempFile);
        tempFile.println();
    }
    logFile.close();
    tempFile.close();

    SPIFFS.remove(LOG_FILE);
    SPIFFS.rename("/temp_log.jsonl", LOG_FILE);
    Serial.println("[LOG] Arquivo de log atualizado.");
}

void displayFileContent(const char* filename) {
  Serial.println();
  Serial.print("--- Conteúdo do arquivo: ");
  Serial.println(filename);
  Serial.println("---");
  File file = SPIFFS.open(filename, "r");
  if (!file) {
    Serial.print("[ERRO] Falha ao abrir o arquivo '");
    Serial.print(filename);
    Serial.println("' para leitura.");
    return;
  }
  if (file.size() == 0) {
      Serial.println("(Arquivo vazio)");
  } else {
      while (file.available()) {
        Serial.write(file.read());
      }
  }
  file.close();
  Serial.println("\n--- Fim do arquivo ---");
}
