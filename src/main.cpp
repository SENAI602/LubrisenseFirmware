//Informações do Visual Studio (Aplicativo)
//public enum CONFIGTYPE { BASICO, AVANCADO } Basico = 0 / Avançado = 1
//public enum INTERVALTYPE {NONE, HORA, DIA, MES } Hora = 1 / Dia = 2 / Mês = 3

#include <Arduino.h>
#include "LoRaMESH.h"      // Biblioteca de comunicação LoRa
#include <SPIFFS.h>       // Para o sistema de arquivos do ESP32
#include <ArduinoJson.h>  // Para manipulação de JSON
#include <vector>         // Adicionado para gerenciar o log em memória
#include "BleService.h"    // Inclui a "ponte" para o nosso serviço BLE
#include <WiFi.h>         // Conexão wifi
#include <Wire.h>
#include "RTClib.h"        // Módulo RC

// WIFI para sincronizar RTC
const char* ssid = "Leonardo2"; 
const char* password = "senaisp602";

// --- Configuração dos Arquivos e Constantes ---
#define CONFIG_FILE "/config.json"
#define LOG_FILE    "/log.jsonl"
#define TEMP_FILE    "/temp.json"
const size_t LIMITE_LINHAS_LOG = 10;
const char* TIMESTAMP_CONSTANT = "2025-07-02T15:00:00Z";
unsigned long intervaloEnviarGateway = 15000;
const unsigned long INTERVALO_PADRAO_SEGURO = 15000; // 15 segundos

// --- Configuração dos Pinos ---
#define LORA_TX_PIN 17
#define LORA_RX_PIN 16
#define SEN_NIVEL_1 4
#define SEN_NIVEL_2 5
#define BTN_MANUAL 19
#define BTN_EXTRA 20
#define MOTOR 32
#define I2C_SDA 22
#define I2C_SCL 23
//#define TEMP_PIN    34         // Pino para ler a temperatura
//#define MANUAL_TRIGGER_PIN 4   // Novo pino para o botão de disparo manual

// --- Configurações da Rede LoRa ---
#define GATEWAY_ID 0

// --- CONFIGURAÇÕES DE NTP do RTC ---
const char* ntpServer = "a.st1.ntp.br";
const long  gmtOffset_sec = -3 * 3600;
const int   daylightOffset_sec = 0;
RTC_DS3231 rtc;
unsigned long ultimaSincronizacao = 0;

// --- Definições dos Comandos LoRa ---
#define CMD_FROM_MASTER_CONFIG 0x10
#define CMD_FROM_MASTER_ACK    0x11
#define CMD_TO_GATEWAY_EVENT   0x20


// --- Variáveis Globais de Controle ---
unsigned long intervalo = INTERVALO_PADRAO_SEGURO;
int configVolume = 10; 
unsigned long ultimaLubrificacao = 0;
unsigned long ultimoReenvio = 0;
bool aguardandoAck = false;
bool varCicloAtivo = false;
unsigned long aguardandoAckDesde = 0;
const unsigned long TIMEOUT_ACK = 15000;
int lastManualButtonState = HIGH;
unsigned long lastPrintTime = 0; // Variável para controlar a impressão a cada 1s
int lastBtnManualState = HIGH; // Armazena o último estado do botão manual

// --- Variáveis de Controle para Bloqueio de Tempo ---
unsigned long ultimoAcionamentoBotao1 = 0;
unsigned long ultimoAcionamentoBotao2 = 0;
const unsigned long intervaloBloqueio = 5000; // 5 segundos em milissegundos

// --- Variáveis Globais para o Estado do Ciclo (lidas do temp.json) ---
bool varTempCicloStartado = false;
String varTempHorarioStartado = "";
String varTempUltimaLubrificacao = "";

// --- Variáveis Globais para a Configuração ---
String varConfigUuid = "";
String varConfigTag = "";
String varConfigEquipamento = "";
String varConfigSetor = "";
String varConfigLubrificante = "";
int varConfigTipoConfig = 0;
int varConfigTipoIntervalo = 0;
String varConfigUltimaConexao = "";

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
bool sincronizarViaNTP();
void salvarEstadoCiclo(bool cicloStartado, const String& horarioStartado, const String& ultimaLubrificacao);
String formatarTimestamp(const DateTime& dt);
void carregarEstadoCiclo();

void setup() {
  delay(500);
  Serial.begin(115200);
  delay(500);
  pinMode(BTN_MANUAL, INPUT_PULLUP);
  pinMode(BTN_EXTRA, INPUT_PULLUP);
  pinMode(SEN_NIVEL_1, INPUT);
  pinMode(SEN_NIVEL_2, INPUT);
  pinMode(MOTOR, OUTPUT);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!rtc.begin()) {
    Serial.println("Não foi possível encontrar o RTC! Verifique as conexões.");
    while (1);
  }

  // LÓGICA DE SINCRONIZAÇÃO INTELIGENTE
  if (rtc.lostPower()) {
    Serial.println("RTC perdeu energia ou é a primeira inicialização. Sincronizando via NTP...");
    if (sincronizarViaNTP()) {
      ultimaSincronizacao = millis(); // Atualiza o marcador de tempo da última sincronização
    }
  } else {
    Serial.println("RTC manteve a energia. A hora atual é:");
    // Se a energia não foi perdida, consideramos que a última sincronização "válida" foi agora,
    // para que a contagem do loop comece a partir deste momento.
    ultimaSincronizacao = millis(); 
  }
  Serial.println("\n--- [SLAVE] Inicializando EndPoint com LoRa e BLE ---");
  Serial.println("--- Envie 'c' para ver config.json ou 'l' para ver log.jsonl ---");

  if (!SPIFFS.begin(true)) {
    Serial.println("[ERRO] Falha ao montar o SPIFFS.");
    while (true);
  }
  Serial.println("[OK] Sistema de arquivos montado.");
  loadInitialConfig();

   carregarEstadoCiclo();

  //Preparação do Lora
  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  lora.begin();
  lora.deviceId = lora.localId;
  lora.debug_serial = false;
  Serial.print("[OK] Módulo LoRa iniciado. ID Local: ");
  Serial.println(lora.localId);
  
  Serial.println("Iniciando ESP32 com BLE...");
  setupBLE();
}


void loop() {
  //--------------------------------------------------------------------------------------------  
  //--------------------------------------------------------------------------------------------  
  //--------------------------------------------------------------------------------------------  
  // Lê o estado atual do botão Manual
  int estadoAtualBtnManual = digitalRead(BTN_MANUAL);
  // Verifica se o botão FOI pressionado (transição de HIGH para LOW)
  if (estadoAtualBtnManual == LOW && lastBtnManualState == HIGH && (millis() - ultimoAcionamentoBotao1 > intervaloBloqueio)){
    Serial.println("Botão manual pressionado!");
    ultimoAcionamentoBotao1 = millis(); // Atualiza o tempo do último acionamento
    String timestampAtual = formatarTimestamp(rtc.now());
    salvarEstadoCiclo(true,timestampAtual,timestampAtual);
    varCicloAtivo = true;
  }
  // Atualiza o estado anterior para a próxima verificação no loop
  lastBtnManualState = estadoAtualBtnManual;
  //--------------------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------------------  
  //--------------------------------------------------------------------------------------------  

  // Recebendo arquivo de config pelo Bluetooth
  if (newDataFromBLE) {
    newDataFromBLE = false; // Reseta a flag para não processar de novo
    Serial.println("\n[BLE] Novos dados de configuração recebidos via Bluetooth!");
    salvarConfig(bleReceivedValue.c_str());
  }

  // Lógica para exibir arquivos via Monitor Serial
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'c' || command == 'C') {
      displayFileContent(CONFIG_FILE);
    } else if (command == 'l' || command == 'L') {
      displayFileContent(LOG_FILE);
    } else if (command == 't' || command == 'T') {
      displayFileContent(TEMP_FILE);
    }
  }

  // // 1. Lógica de Recepção de Comandos LoRa
  // uint16_t senderId;
  // uint8_t receivedCommand;
  // uint8_t payload[MAX_PAYLOAD_SIZE];
  // uint8_t payloadSize;

  // if (lora.ReceivePacketCommand(&senderId, &receivedCommand, payload, &payloadSize, 100)) {
  //   payload[payloadSize] = '\0';
  //   String msg = (char*)payload;
  //   Serial.print("[LORA] Comando recebido do ID ");
  //   Serial.print(senderId);
  //   Serial.print(" - CMD: 0x");
  //   Serial.print(receivedCommand, HEX);
  //   Serial.print(" - Payload: ");
  //   Serial.println(msg);

  //   switch (receivedCommand) {
  //     case CMD_FROM_MASTER_ACK:
  //       marcarEventoComoEnviado();
  //       break;
  //     case CMD_FROM_MASTER_CONFIG:
  //       salvarConfig(msg.c_str());
  //       break;
  //     default:
  //       Serial.println("[AVISO] Comando LoRa desconhecido.");
  //       break;
  //   }
  // }

  // // 2. Lógica de Registro Automático por tempo
  // if (millis() - ultimaLubrificacao >= intervalo) {
  //   Serial.println("\n[EVENTO] Intervalo de tempo atingido. Registrando (auto)...");
  //   digitalWrite(DIGITAL_OUT, HIGH);
  //   delay(500); // Este delay é curto e aceitável para acionar algo físico
  //   digitalWrite(DIGITAL_OUT, LOW);
  //   registrarEvento("auto");
  //   ultimaLubrificacao = millis();
  // }

  // // 3. Lógica de Registro Manual por botão
  // int manualButtonState = digitalRead(MANUAL_TRIGGER_PIN);
  // if (manualButtonState == LOW && lastManualButtonState == HIGH) {
  //   Serial.println("\n[EVENTO] Botão manual pressionado. Registrando (manual)...");
  //   digitalWrite(DIGITAL_OUT, HIGH);
  //   delay(500); // Este delay é curto e aceitável
  //   digitalWrite(DIGITAL_OUT, LOW);
  //   registrarEvento("manual");
  //   delay(200); // Debounce
  // }
  // lastManualButtonState = manualButtonState;

  // // 4. Lógica de Reenvio de Eventos
  // if (millis() - ultimoReenvio > intervaloEnviarGateway) {
  //   tentarReenvio();
  //   ultimoReenvio = millis();
  // }
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

  if (!doc["Uuid"].isNull()) varConfigUuid = doc["Uuid"].as<String>();
  if (!doc["Tag"].isNull()) varConfigTag = doc["Tag"].as<String>();
  if (!doc["Equipamento"].isNull()) varConfigEquipamento = doc["Equipamento"].as<String>();
  if (!doc["Setor"].isNull()) varConfigSetor = doc["Setor"].as<String>();
  if (!doc["Lubrificante"].isNull()) varConfigLubrificante = doc["Lubrificante"].as<String>();
  if (!doc["TipoConfig"].isNull()) varConfigTipoConfig = doc["TipoConfig"].as<int>();
  if (!doc["TipoIntervalo"].isNull()) varConfigTipoIntervalo = doc["TipoIntervalo"].as<int>();
  if (!doc["Volume"].isNull()) configVolume = doc["Volume"].as<int>();
  if (!doc["Intervalo"].isNull()) {
    unsigned long novoIntervalo = doc["Intervalo"].as<unsigned long>() * 1000;
    if (novoIntervalo < 1000) { // Se o intervalo for menor que 1 segundo
      Serial.println("[AVISO] Intervalo recebido é muito baixo. Usando valor padrão para evitar travamento.");
      intervalo = INTERVALO_PADRAO_SEGURO; // Usa o valor padrão seguro
    } else {
      intervalo = novoIntervalo;
    }
  }
  if (!doc["UltimaConexao"].isNull()) varConfigUltimaConexao = doc["UltimaConexao"].as<String>();

  //Visualização para Debug
  Serial.println("[CONFIG] Variáveis globais carregadas:");
  Serial.print("  > Uuid: "); Serial.println(varConfigUuid);
  Serial.print("  > Tag: "); Serial.println(varConfigTag);
  Serial.print("  > Equipamento: "); Serial.println(varConfigEquipamento);
  Serial.print("  > Setor: "); Serial.println(varConfigSetor);
  Serial.print("  > Lubrificante: "); Serial.println(varConfigLubrificante);
  Serial.print("  > TipoConfig: "); Serial.println(varConfigTipoConfig);
  Serial.print("  > TipoIntervalo: "); Serial.println(varConfigTipoIntervalo);
  Serial.print("  > Volume: "); Serial.println(configVolume);
  Serial.print("  > Intervalo: "); Serial.print(intervalo); Serial.println(" ms");
  Serial.print("  > UltimaConexao: "); Serial.println(varConfigUltimaConexao);
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
  // doc["t"] = analogRead(TEMP_PIN);
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

// Função para conectar e sincronizar o RTC
bool sincronizarViaNTP() {
  Serial.print("Conectando a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int tentativas = 30; 
  while (WiFi.status() != WL_CONNECTED && tentativas > 0) {
    delay(500);
    Serial.print(".");
    tentativas--;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFalha ao conectar ao Wi-Fi.");
    return false;
  }
  
  Serial.println("\nWi-Fi conectado!");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());

  Serial.println("Configurando a hora via NTP...");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Falha ao obter a hora do servidor NTP.");
    return false;
  }

  Serial.println("Hora NTP recebida. Sincronizando o RTC...");
  rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, 
                      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));

  Serial.println("RTC SINCRONIZADO com sucesso!");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("Wi-Fi desconectado.");
  return true;
}

void salvarEstadoCiclo(bool cicloStartado, const String& horarioStartado, const String& ultimaLubrificacao) {
  // 1. Cria um objeto JSON na memória.
  JsonDocument doc;

  // 2. Adiciona as três variáveis ao objeto JSON.
  doc["cicloStartado"] = cicloStartado;
  doc["horarioStartado"] = horarioStartado;
  doc["ultimaLubrificacao"] = ultimaLubrificacao;
  varTempCicloStartado = cicloStartado;
  varTempHorarioStartado = horarioStartado;
  varTempUltimaLubrificacao = ultimaLubrificacao;

  // 3. Abre o arquivo temp.json em modo de escrita ("w").
  // Isso apaga o conteúdo anterior e escreve o novo.
  File file = SPIFFS.open(TEMP_FILE, "w");
  if (!file) {
    Serial.println("[ERRO] Falha ao abrir /temp.json para escrita.");
    return;
  }

  // 4. Converte o objeto JSON em texto e o escreve no arquivo.
  if (serializeJson(doc, file) > 0) {
    Serial.println("[ESTADO] Arquivo temp.json salvo com sucesso.");
  } else {
    Serial.println("[ERRO] Falha ao escrever JSON no arquivo temp.json.");
  }

  // 5. Fecha o arquivo para garantir que os dados sejam gravados.
  file.close();
}


void carregarEstadoCiclo() {
  if (SPIFFS.exists(TEMP_FILE)) {
    File file = SPIFFS.open(TEMP_FILE, "r");
    if (file) {
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, file);
      file.close();

      if (error) {
        Serial.println("[ERRO] Falha ao ler o arquivo temp.json, o arquivo pode estar corrompido.");
        return;
      }

      Serial.println("[ESTADO] Lendo arquivo temp.json...");
      varTempCicloStartado = doc["cicloStartado"].as<bool>();
      varTempHorarioStartado = doc["horarioStartado"].as<String>();
      varTempUltimaLubrificacao = doc["ultimaLubrificacao"].as<String>();

      Serial.println("[ESTADO] Variáveis de estado carregadas:");
      Serial.print("  > Ciclo Startado: "); Serial.println(varTempCicloStartado ? "true" : "false");
      Serial.print("  > Horário Startado: "); Serial.println(varTempHorarioStartado);
      Serial.print("  > Última Lubrificação: "); Serial.println(varTempUltimaLubrificacao);

    } else {
      Serial.println("[ERRO] Não foi possível abrir o arquivo temp.json para leitura.");
    }
  } else {
    Serial.println("[ESTADO] Nenhum arquivo temp.json encontrado. Iniciando com estado padrão.");
  }
}


String formatarTimestamp(const DateTime& dt) {
  // Cria um buffer de caracteres para armazenar a string formatada.
  // O tamanho 20 é suficiente para "YYYY-MM-DDTHH:MM:SS" mais o caractere nulo de terminação.
  char buffer[20];

  // Usa sprintf para formatar os dados no buffer.
  // %04d = ano com 4 dígitos
  // %02d = mês/dia/hora/minuto/segundo com 2 dígitos (adiciona zero à esquerda se necessário)
  sprintf(buffer, "%04d-%02d-%02dT%02d:%02d:%02d",
          dt.year(), dt.month(), dt.day(),
          dt.hour(), dt.minute(), dt.second());

  // Retorna o buffer como um objeto String do Arduino.
  return String(buffer);
}