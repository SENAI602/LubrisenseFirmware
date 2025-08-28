// Definições do aplicativo:
// public enum CONFIGTYPE { BASICO, AVANCADO } Basico = 0 / Avançado = 1
// public enum INTERVALTYPE {NONE, HORA, DIA, MES } Hora = 1 / Dia = 2 / Mês = 3

// ==========================================================================
// --- INCLUSÃO DE BIBLIOTECAS (DEPENDÊNCIAS) ---
// ==========================================================================

#include <Arduino.h>       // Framework principal do Arduino para ESP32.
#include <vector>          // Biblioteca C++ para usar listas dinâmicas (vetores).
#include "LoRaMESH.h"      // Driver para o módulo de rádio LoRa em modo Mesh.
#include "BleService.h"    // Módulo personalizado para o serviço Bluetooth Low Energy (BLE).
#include <WiFi.h>          // Gerenciamento de conexão Wi-Fi para sincronização de tempo.
#include <Wire.h>          // Protocolo de comunicação I2C, usado pelo módulo RTC.
#include <SPIFFS.h>        // Gerenciador do sistema de arquivos na memória flash do ESP32.
#include <ArduinoJson.h>   // Para codificar e decodificar dados no formato JSON.
#include "RTClib.h"        // Driver para o módulo de Relógio de Tempo Real (RTC DS3231).
#include <INA226.h>        // Leitura do nível de bateria

// ==========================================================================
// --- CREDENCIAIS DE WI-FI (PARA SINCRONIZAÇÃO NTP) ---
// ==========================================================================
// Estas credenciais são usadas apenas na inicialização para conectar à internet
// e sincronizar o relógio do módulo RTC com um servidor de tempo mundial (NTP).
// Após a sincronização, o Wi-Fi é desligado.
// ==========================================================================
const char* ssid = "Leonardo2";      // Nome da rede Wi-Fi (SSID).
const char* password = "senaisp602"; // Senha da rede Wi-Fi.

// ==========================================================================
// --- ARQUIVOS E CONSTANTES DE CONTROLE ---
// ==========================================================================

// --- Nomes dos Arquivos no Sistema SPIFFS ---
#define CONFIG_FILE "/config.json" // Armazena a configuração principal recebida via LoRa ou BLE.
#define LOG_FILE    "/log.jsonl"   // Armazena o histórico das últimas lubrificações (eventos).
#define TEMP_FILE   "/temp.json"   // Salva o estado atual do ciclo (iniciado, última lubrificação).

// --- Parâmetros de Funcionamento ---
const size_t limiteLinhasLog = 10;                        // Define o número máximo de registros a serem mantidos no log.jsonl.
unsigned long intervaloEnviarGateway = 15000;             // Frequência (em ms) com que o dispositivo tenta enviar logs pendentes.

// ==========================================================================
// --- MAPEAMENTO DE PINOS (HARDWARE) ---
// ==========================================================================

#define LORA_TX_PIN 17 // Conecta ao pino RX do módulo LoRa.
#define LORA_RX_PIN 16 // Conecta ao pino TX do módulo LoRa.
#define SEN_NIVEL_BAIXO 4  // Sensor de nível baixo de lubrificante.
#define SEN_NIVEL_CRITICO 5  // Sensor de nível crítico de lubrificante.
#define MOTOR       32 // Pino que aciona o relé/driver do motor de lubrificação.
#define BTN_MANUAL  19 // Botão para iniciar um ciclo de lubrificação manual.
#define BTN_EXTRA   20 // Botão para funções futuras ou testes.
#define I2C_SDA     22 // Pino de dados (SDA) para o RTC DS3231.
#define I2C_SCL     23 // Pino de clock (SCL) para o RTC DS3231.
#define PT100_PIN   34 // Pino ADC para ler a saída do conversor 4-20mA.
INA226 ina226(0x40);   // <-- Cria o objeto com o endereço I2C correto

// ==========================================================================
// --- CONSTANTES PARA O SENSOR PT100 4-20mA ---
// ==========================================================================

const float R_SHUNT = 249.0;      // Valor do resistor de precisão (R11) em Ohms.
const float TEMP_MIN = 0.0;       // Temperatura correspondente a 4mA (conforme o transmissor).
const float TEMP_MAX = 150.0;     // Temperatura correspondente a 20mA (conforme o transmissor).

// ==========================================================================
// --- CONFIGURAÇÕES DO LORA ---
// ==========================================================================

// --- Rede LoRa ---
#define GATEWAY_ID 0 // Define o endereço do Gateway na rede LoRa.

// --- Protocolo de Comandos LoRa (Definições Hexadecimais) ---
// Comandos recebidos do Gateway:
#define CMD_FROM_MASTER_CONFIG 0x10 // Comando para receber um novo arquivo de configuração.
#define CMD_FROM_MASTER_ACK    0x11 // Comando de confirmação (ACK) de que um evento foi recebido pelo Gateway.
#define CMD_TO_GATEWAY_EVENT   0x20 // Comando para enviar um novo registro de evento (lubrificação).

HardwareSerial LoRaSerial(2); // Objeto para a comunicação Serial na UART2 com o módulo LoRa.
LoRaMESH lora(&LoRaSerial);   // Instância da biblioteca LoRaMESH.

// ==========================================================================
// --- Sincronização de Tempo (RTC via NTP) ---
// ==========================================================================
const char* ntpServer = "a.st1.ntp.br";       // Servidor NTP brasileiro para obter a hora mundial.
const long  gmtOffset_sec = -3 * 3600;        // Fuso horário de Brasília (GMT-3) em segundos.
const int   daylightOffset_sec = 0;           // Ajuste para horário de verão (desativado).
RTC_DS3231 rtc;                               // Cria o objeto que representa o módulo RTC DS3231.
unsigned long ultimaSincronizacao = 0;        // Armazena o tempo (em millis) da última sincronização bem-sucedida.

// ==========================================================================
// --- VARIÁVEIS ---
// ==========================================================================

// Variaveis do arquivo config.json
String varConfigUuid = "";
String varConfigTag = "";
String varConfigEquipamento = "";
String varConfigSetor = "";
String varConfigLubrificante = "";
int    varConfigTipoConfig = 0;
int    varConfigTipoIntervalo = 0;
int    varConfigVolume = 10;          
unsigned long varConfigIntervalo = 0; 
String varConfigUltimaConexao = "";

// Variaveis do arquivo temp.json
bool   varTempCicloStartado = false;      // Indica se um ciclo (manual ou automático) está ativo.
String varTempHorarioStartado = "";     // Timestamp de quando o ciclo atual foi iniciado.
String varTempUltimaLubrificacao = "";  // Timestamp da última lubrificação executada.

// Outras
unsigned long ultimoReenvio = 0;        // Registra o tempo do último envio de log para o Gateway.
bool          aguardandoAck = false;      // Flag que indica se o dispositivo está esperando uma confirmação (ACK) do Gateway.
unsigned long aguardandoAckDesde = 0;   // Registra quando a espera pelo ACK começou.
const unsigned long TIMEOUT_ACK = 15000;  // Tempo máximo (em ms) de espera por um ACK.
int           estadoAnteriorBtnManual = HIGH;  // Armazena o estado anterior do botão manual para detectar a borda de subida.
int   estadoAnteriorNivelBaixo = HIGH;     // Armazena o último estado do sensor de nível baixo.
int   estadoAnteriorNivelCritico = HIGH;     // Armazena o último estado do sensor de nível crítico.
unsigned long ultimoAcionamentoBotaoManual = 0;    // Registra o tempo do último acionamento do botão manual.
unsigned long ultimoAcionamentoBotao2 = 0;    // Registra o tempo do último acionamento do botão extra.
const unsigned long intervaloBloqueio = 5000; // Intervalo de tempo (em ms) que um botão fica bloqueado após ser pressionado.

// --- Comunicação Bluetooth (BLE) ---
// Variáveis para a troca de dados entre o serviço BLE e o loop principal.
// 'volatile' é usado para garantir que a variável seja lida corretamente,
// pois ela pode ser modificada por uma interrupção do BLE.
// --------------------------------------------------------------------------
volatile bool newDataFromBLE = false;     // Flag que sinaliza a chegada de novos dados via BLE.
String        bleReceivedValue = "";      // Armazena a string de configuração recebida do BLE.


// ==========================================================================
// --- PROTÓTIPOS DE FUNÇÕES (DECLARAÇÕES) ---
// ==========================================================================
// Informa ao compilador sobre a existência dessas funções antes de serem usadas.

// --- Funções de Comunicação ---
void sendJsonViaLoRa(const String &json, uint8_t command);
bool sincronizarViaNTP();
void salvarConfig(const char* json);
void loadInitialConfig();
void salvarEstadoCiclo(bool cicloStartado, const String& horarioStartado, const String& ultimaLubrificacao);
void carregarEstadoCiclo();
void displayFileContent(const char* filename);
void registrarEvento(const String& fonte);
void tentarReenvio();
void marcarEventoComoEnviado();
String formatarTimestamp(const DateTime& dt);
DateTime stringParaDateTime(const String& timestampStr);

// =======================================================================================================================================

// ==========================================================================
// --- SETUP ---
// ==========================================================================
void setup() {
  // Inicia a serial
  Serial.begin(115200);
  delay(100);

  // Configura os pinos do esp
  pinMode(BTN_MANUAL, INPUT_PULLUP);
  pinMode(BTN_EXTRA, INPUT_PULLUP);
  pinMode(SEN_NIVEL_BAIXO, INPUT);
  pinMode(SEN_NIVEL_CRITICO, INPUT);
  pinMode(MOTOR, OUTPUT);

  // Configura o RTC
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!rtc.begin()) {
    Serial.println("Não foi possível encontrar o RTC! Verifique as conexões.");
    while (1);
  }  
  if (rtc.lostPower()) {
    Serial.println("RTC perdeu energia ou é a primeira inicialização. Sincronizando via NTP...");
    if (sincronizarViaNTP()) {
      ultimaSincronizacao = millis();
    }
  } else {
    Serial.println("RTC manteve a energia. A hora atual é:");
    ultimaSincronizacao = millis(); 
  }

  // Prepara o sistema SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("[ERRO] Falha ao montar o SPIFFS.");
    while (true);
  }
  Serial.println("[OK] Sistema de arquivos montado.");
  
  // Carrega o arquivo config assim que o esp liga
  loadInitialConfig();

  // Carrega o arquivo temp assim que o esp liga
  carregarEstadoCiclo();

  //Preparação do Lora
  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  lora.begin();
  lora.deviceId = lora.localId;
  lora.debug_serial = false;
  Serial.print("[OK] Módulo LoRa iniciado. ID Local: ");
  Serial.println(lora.localId);

  // Inicia o sensor medidor de bateria INA226
  ina226.begin();
  if (!ina226.isConnected()) {
    Serial.println("[ERRO] Falha ao encontrar o sensor INA226! Verifique a fiação.");
  } else {
    Serial.println("[OK] Sensor INA226 encontrado e iniciado.");
  }
  
  // Inicia o BLE
  Serial.println("Iniciando ESP32 com BLE...");
  setupBLE();

  Serial.println("\n--- [SLAVE] Inicializado EndPoint com LoRa e BLE ---");
  Serial.println("--- Envie 'c' para ver config.json, 'l' para ver log.jsonl ou 't' oara ver o temp.json ---");
}

// =======================================================================================================================================

// ==========================================================================
// --- LOOP ---
// ==========================================================================
void loop() {
  //-----------------------------------------------------------------------------------------------------------------------------------  
  // CICLO MANUAL (BOTÃO)
  //-----------------------------------------------------------------------------------------------------------------------------------  
  // Lê o estado atual do botão Manual
  int estadoAtualBtnManual = digitalRead(BTN_MANUAL);
  // Verifica se o botão FOI pressionado (transição de HIGH para LOW)
  if (estadoAtualBtnManual == LOW && estadoAnteriorBtnManual == HIGH && (millis() - ultimoAcionamentoBotaoManual > intervaloBloqueio)){
    Serial.println("Botão manual pressionado!");

    //  \/ LÓGICA DE DOSAGEM AQUI \/
    digitalWrite(MOTOR, HIGH);
    delay(500);
    digitalWrite(MOTOR, LOW);
    // ---

    // Salva o horario do ciclo atual para calcular quando será o próximo. Registra o evento no log como manual
    ultimoAcionamentoBotaoManual = millis(); // Atualiza o tempo do último acionamento
    String timestampAtual = formatarTimestamp(rtc.now());
    salvarEstadoCiclo(true,timestampAtual,timestampAtual);
    registrarEvento("Manual");
  }
  // Atualiza o estado anterior para a próxima verificação nlo loop
  estadoAnteriorBtnManual = estadoAtualBtnManual;
  //----------------------------------------------------------------------------------------------------------------------------------- 
  // CICLO AUTOMÁTICO
  //-----------------------------------------------------------------------------------------------------------------------------------
  // Verifica se a variável está como true. Pois, o ciclo automático só começa a contar depois que houver a primeira dosagem manual
  if (varTempCicloStartado) {    
    // Obtém a hora atual do RTC e Converte a string da última lubrificação para um objeto DateTime
    DateTime agora = rtc.now();
    DateTime ultimaLubrificacaoDT = stringParaDateTime(varTempUltimaLubrificacao);

    // Converte os tempos para Unix Timestamps (um número total de segundos)
    uint32_t agora_ts = agora.unixtime();
    uint32_t ultima_lub_ts = ultimaLubrificacaoDT.unixtime();

    // Converte o intervalo de configuração (que está em milissegundos) para segundos
    uint32_t intervalo_s = varConfigIntervalo / 1000;

      // Verifica se o tempo atual for maior ou igual ao tempo da última lubrificação + o intervalo...
      if (agora_ts >= (ultima_lub_ts + intervalo_s)) {
        Serial.println("\n[CICLO AUTOMÁTICO] Tempo de lubrificação atingido!");       

        //Modo Básico
        if(varConfigTipoConfig==1){       
          Serial.println("\n[CICLO AUTOMÁTICO] Realizando dosagem no modo básico"); 
          //  \/ LÓGICA DE DOSAGEM AQUI \/
          digitalWrite(MOTOR, HIGH);
          delay(500);
          digitalWrite(MOTOR, LOW);

        //Modo Avançado
        }else if (varConfigTipoConfig==2){ 
          Serial.println("\n[CICLO AUTOMÁTICO] Realizando dosagem no modo avançado");        
          //  \/ LÓGICA DE DOSAGEM AQUI \/
          digitalWrite(MOTOR, HIGH);
          delay(500);
          digitalWrite(MOTOR, LOW);
        }
        
        // Escreve no arquivo log
        registrarEvento("Auto");
        // Atualiza a variável da última lubrificação com a hora atual para reiniciar o timer
        varTempUltimaLubrificacao = formatarTimestamp(agora);          
        // Salva o novo estado no arquivo para que ele não se perca se o dispositivo reiniciar
        salvarEstadoCiclo(true, varTempHorarioStartado, varTempUltimaLubrificacao);
      }    
  }

  //----------------------------------------------------------------------------------------------------------------------------------- 
  // Recebendo arquivo de config pelo Bluetooth
  //-----------------------------------------------------------------------------------------------------------------------------------
  if (newDataFromBLE) {
    newDataFromBLE = false; // Reseta a flag para não processar de novo
    Serial.println("\n[BLE] Novos dados de configuração recebidos via Bluetooth!");
    salvarConfig(bleReceivedValue.c_str());
  }

  
  //----------------------------------------------------------------------------------------------------------------------------------- 
  // LEITURA DOS SENSORES DE NÍVEL
  //-----------------------------------------------------------------------------------------------------------------------------------
  // Lê o estado atual do sensor de nível 1 (baixo)
  int estadoAtualNivelBaixo = digitalRead(SEN_NIVEL_BAIXO);
  // Se o estado mudou de ALTO para BAIXO, significa que o nível caiu
  if (estadoAtualNivelBaixo == LOW && estadoAnteriorNivelBaixo == HIGH) {
    Serial.println("[ALERTA] Nível baixo de lubrificante detectado!");
    // ADICIONAR LÓGICA FUTURAMENTE
  }
  // Atualiza o estado anterior para a próxima verificação
  estadoAnteriorNivelBaixo = estadoAtualNivelBaixo;

  // Lê o estado atual do sensor de nível 2 (crítico)
  int estadoAtualNivelCritico = digitalRead(SEN_NIVEL_CRITICO);
  // Se o estado mudou de ALTO para BAIXO, significa que o nível está crítico
  if (estadoAtualNivelCritico == LOW && estadoAnteriorNivelCritico == HIGH) {
    Serial.println("[ALERTA CRÍTICO] Nível crítico de lubrificante detectado!");
    // ADICIONAR LÓGICA FUTURAMENTE
  }
  // Atualiza o estado anterior para a próxima verificação
  estadoAnteriorNivelCritico = estadoAtualNivelCritico;
  //-----------------------------------------------------------------------------------------------------------------------------------
  
  //----------------------------------------------------------------------------------------------------------------------------------- 
  // Lógica para exibir os arquivos via Monitor Serial
  //-----------------------------------------------------------------------------------------------------------------------------------
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

  // // Lógica de Recepção de Comandos LoRa
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
  
  // // Lógica de Reenvio de Eventos
  // if (millis() - ultimoReenvio > intervaloEnviarGateway) {
  //   tentarReenvio();
  //   ultimoReenvio = millis();
  // }
}

// Envia o json para o master
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

//Lê o arquivo config.json para puxar as informações dele para a variável
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

// Ao receber o arquivo config.json do Bluetooth, ele salva
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
  if (!doc["TipoIntervalo"].isNull()) varConfigTipoIntervalo = doc["TipoIntervalo"].as<int>(); // Lemos o TipoIntervalo PRIMEIRO
  if (!doc["Volume"].isNull()) varConfigVolume = doc["Volume"].as<int>();
  if (!doc["UltimaConexao"].isNull()) varConfigUltimaConexao = doc["UltimaConexao"].as<String>();

  if (!doc["Intervalo"].isNull()) {
    unsigned long valorIntervalo = doc["Intervalo"].as<unsigned long>();
    unsigned long multiplicador = 1000; // Padrão é segundos

    switch (varConfigTipoIntervalo) {
      case 1: // Hora
        multiplicador = 3600000UL;
        break;
      case 2: // Dia
        multiplicador = 86400000UL;
        break;
      case 3: // Mês
        multiplicador = 2592000000UL;
        break;
      default: // Segundos ou None
        multiplicador = 1000;
        break;
    }

    unsigned long novoIntervaloCalculado = valorIntervalo * multiplicador;
    varConfigIntervalo = novoIntervaloCalculado;  
  }

  // Visualização para Debug (corrigido para mostrar a variável certa)
  Serial.println("[CONFIG] Variáveis globais carregadas:");
  Serial.print("  > Uuid: "); Serial.println(varConfigUuid);
  Serial.print("  > Tag: "); Serial.println(varConfigTag);
  Serial.print("  > Equipamento: "); Serial.println(varConfigEquipamento);
  Serial.print("  > Setor: "); Serial.println(varConfigSetor);
  Serial.print("  > Lubrificante: "); Serial.println(varConfigLubrificante);
  Serial.print("  > TipoConfig: "); Serial.println(varConfigTipoConfig);
  Serial.print("  > TipoIntervalo: "); Serial.println(varConfigTipoIntervalo);
  Serial.print("  > Volume: "); Serial.println(varConfigVolume);
  Serial.print("  > Intervalo: "); Serial.print(varConfigIntervalo); Serial.println(" ms"); // Mostra a variável correta
  Serial.print("  > UltimaConexao: "); Serial.println(varConfigUltimaConexao);
}

// Regista a lubrificação no log.json
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

  while (lines.size() >= limiteLinhasLog) {
    lines.erase(lines.begin());
  }

  JsonDocument doc;
  doc["Hora"] = formatarTimestamp(rtc.now());   // Data/Hora atual
  doc["Sucesso"] = true;                        // FUTURAMENTE: Implementar a verificação do motor para garantir que tenha feito a lubrificação
  doc["Modo"] = fonte;                          // Manual ou Automático
  doc["Temperatura"] = lerTemperaturaPT100();   // Temperatura
  doc["Bateria"] = lerNivelBateria();           // Nivel da bateria
  doc["e"] = false;                             // Variável que fala se a linha foi enviada para o gateway

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

// Tenta enviar a uma linha que ainda não foi enviada do arquivo log.json para o master
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

// Caso receba ACK do master, marca a linha como enviada
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

// Exibe no serial monitor as informações dos arquivos .json que estão salvos no esp
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

// Função para salvar os parâmetros que não podem se perder caso o dispositivo seja reiniciado
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

// Carrega os valores do arquivo local para as variáveis do programa
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

// Formata o timestamp para o padrão utilizado no Visual Studio
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

// Converte a string salva no arquivo local em uma time stamp usável no programa
DateTime stringParaDateTime(const String& timestampStr) {
  int ano, mes, dia, hora, minuto, segundo;
  
  // sscanf é uma funçãopara extrair números de uma string formatada.
  // Ela lê a string e preenche as variáveis com os valores encontrados.
  sscanf(timestampStr.c_str(), "%d-%d-%dT%d:%d:%d", &ano, &mes, &dia, &hora, &minuto, &segundo);
  
  return DateTime(ano, mes, dia, hora, minuto, segundo);
}

int lerNivelBateria() {
  // getBusVoltage_V() retorna a voltagem diretamente em Volts
  float voltagem = ina226.getBusVoltage();
  // Mapeia a faixa de voltagem (3.2V a 4.2V) para a faixa de porcentagem (0 a 100)
  float porcentagem = ((voltagem - 3.2) / (4.2 - 3.2)) * 100.0;

  // Garante que o valor final esteja sempre entre 0 e 100
  if (porcentagem > 100) {
    porcentagem = 100;
  }
  if (porcentagem < 0) {
    porcentagem = 0;
  }

  return (int)porcentagem; // Retorna o valor como um número inteiro
}

float lerTemperaturaPT100() {
  // 1. Lê o valor bruto do ADC (0-4095)
  int valorADC = analogRead(PT100_PIN);

  // 2. Converte o valor do ADC para Voltagem (assumindo referência de 3.3V)
  float voltagem = (valorADC / 4095.0) * 3.3;

  // 3. Calcula a corrente usando a Lei de Ohm (I = V/R)
  // O resultado será em Amperes, então multiplicamos por 1000 para ter em miliamperes (mA)
  float corrente_mA = (voltagem / R_SHUNT) * 1000.0;

  // 4. Mapeia a faixa de corrente (4mA a 20mA) para a faixa de temperatura
  float temperatura = ((corrente_mA - 4.0) / (20.0 - 4.0)) * (TEMP_MAX - TEMP_MIN) + TEMP_MIN;

  // Garante que o valor não saia da faixa esperada
  if (temperatura < TEMP_MIN) temperatura = TEMP_MIN;
  if (temperatura > TEMP_MAX) temperatura = TEMP_MAX;

  return temperatura;
}