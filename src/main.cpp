// Definições do aplicativo:
// public enum CONFIGTYPE { BASICO, AVANCADO } Basico = 0 / Avançado = 1
// public enum INTERVALTYPE {NONE, HORA, DIA, MES } Hora = 1 / Dia = 2 / Mês = 3

// ==========================================================================
// --- INCLUSÃO DE BIBLIOTECAS (DEPENDÊNCIAS) ---
// ==========================================================================

#include <Arduino.h>       // Framework principal do Arduino para ESP32.
#include <vector>          // Biblioteca C++ para usar listas dinâmicas (vetores).
#include "LoRaMESH.h"      // Driver para o módulo de rádio LoRa em modo Mesh.
#include <BleService.h>    // Módulo personalizado para o serviço Bluetooth Low Energy (BLE).
#include <WiFi.h>          // Gerenciamento de conexão Wi-Fi para sincronização de tempo.
#include <Wire.h>          // Protocolo de comunicação I2C, usado pelo módulo RTC.
#include <SPIFFS.h>        // Gerenciador do sistema de arquivos na memória flash do ESP32.
#include <ArduinoJson.h>   // Para codificar e decodificar dados no formato JSON.
#include "RTClib.h"        // Driver para o módulo de Relógio de Tempo Real (RTC DS3231).
#include <INA226.h>        // Leitura do INA226
#include <MotorEncoder.h>  // <-- Inclui a biblioteca da pasta lib/

// ==========================================================================
// --- CREDENCIAIS DE WI-FI (PARA SINCRONIZAÇÃO NTP) ---
// ==========================================================================
// Estas credenciais são usadas apenas na inicialização para conectar à internet
// e sincronizar o relógio do módulo RTC com um servidor de tempo mundial (NTP).
// Após a sincronização, o Wi-Fi é desligado.
// ==========================================================================
const char* ssid = "Leonardo";      // Nome da rede Wi-Fi (SSID).
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
unsigned long intervaloEnviarGateway = 30000;             // Frequência (em ms) com que o dispositivo tenta enviar logs pendentes.

// ==========================================================================
// --- MAPEAMENTO DE PINOS (HARDWARE) ---
// ==========================================================================

#define LORA_TX_PIN         43  // Conecta ao pino RX do módulo LoRa.
#define LORA_RX_PIN         44  // Conecta ao pino TX do módulo LoRa.
#define SEN_NIVEL_BAIXO     5   // Sensor de nível baixo de lubrificante.
#define SEN_NIVEL_ALTO      6   // Sensor de nível alto de lubrificante.
#define MOTOR_IN1           21  // Pino de controle de direção 1
#define MOTOR_IN2           18  // Pino de controle de direção 2
#define MOTOR_EN            38  // Pino de habilitação (liga/desliga) do motor
#define ENCODER_PIN_A 15        // Pino do enconder A
#define ENCODER_PIN_B 16        // Pino do encoder B
#define BTN_MANUAL          2   // Botão para iniciar um ciclo de lubrificação manual.
//#define BTN_EXTRA           4 // Botão para funções futuras ou testes.
#define I2C_SDA             11  // Pino de dados (SDA) para o RTC DS3231.
#define I2C_SCL             10  // Pino de clock (SCL) para o RTC DS3231.
#define PT100_PIN           1   // Pino ADC para ler a saída do conversor 4-20mA.

// ==========================================================================
// --- ENDEREÇAMENTO DO INA226 (Monitoramento de corrente) ---
// ==========================================================================
INA226 ina226Bateria    (0x40); // Endereço I2C -> Nível da bateria
INA226 ina226Motor      (0x41); // Endereço I2C -> Nível do motor

// ==========================================================================
// --- CONSTANTES DO MOTOR ---
// ==========================================================================

const float CORRENTE_MAXIMA_MOTOR = 2000.0;  // Em mA. Se for maior que isso, o motor está travado
const float SHUNT_MOTOR_OHMS = 0.02;  // Resistencia do resistor shutn
const int VELOCIDADE_MOTOR = 60; // Velocidade PWM (0-255)
const long PULSOS_PARA_UMA_VOLTA = 5650;  // Na teoria é 16 x 369 = 5904. Porém, isso passa de uma volta
const float GRAMAS_POR_VOLTA = 2.21; // Quantas gramas o sistema dosa por 1 volta

// --- Cria o Objeto Motor ---
MotorEncoder Motor(
  ENCODER_PIN_A, 
  ENCODER_PIN_B, 
  MOTOR_EN, 
  MOTOR_IN1, 
  MOTOR_IN2
);

// ==========================================================================
// --- CONSTANTES PARA O SENSOR PT100 4-20mA ---
// ==========================================================================

const float R_SHUNT = 249.0;      // Valor do resistor de precisão (R11) em Ohms. // PROVAVELMENTE EXCLUIR: 
const float TEMP_MIN = 0.0;           // Temperatura correspondente a 4mA (conforme o transmissor).
const float TEMP_MAX = 150.0;         // Temperatura correspondente a 20mA (conforme o transmissor).
#define FATOR_CONVERSAO_PT100 100.0   // (1.0 * 100) (resistor x ganho) (Para o WK IntegrateWise => 8,25 X 25)
#define NUM_AMOSTRAS_ADC 20           // Número de amostras da leitura de temperatura

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
bool isClockValid = false;                    // Flag para garantir que o RTC tem uma hora válida.

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
unsigned long varConfigValorIntervalo = 0; 
int varConfigFrequencia = 0;      // Valor da Frequência (ex: 5)
int varConfigTipoFrequencia = 0;  // Unidade da Frequência (ex: 2=Dia)
unsigned long varConfigIntervaloTrigger_ms = 0; // Frequência em MS (ex: 5 dias em ms)
String varConfigUltimaConexao = "";
bool configExiste = false;

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
unsigned long ultimoAcionamentoBotaoManual = 0;    // Registra o tempo do último acionamento do botão manual.
unsigned long ultimoAcionamentoBotao2 = 0;    // Registra o tempo do último acionamento do botão extra.
const unsigned long intervaloBloqueio = 5000; // Intervalo de tempo (em ms) que um botão fica bloqueado após ser pressionado.
String fonteDoEventoAtual = "Nenhum";         // Diz qual foi o tipo do acionamento MANUAL / AUTOMÁTICO
int VarDosagemManualBasico;                   // Dosagem calculado para o modo básico

// --- Variáveis de Controle do Motor ---
// Enum para definir a direção de rotação do motor de forma clara e segura
bool motorLigado = false;                      // Flag para saber se o motor está em movimento.
unsigned long motorTempoFinal = 0;             // Armazena o "horário" (em millis) em que o motor deve parar.
Sentido motorDirecaoAtual = HORARIO;          // Armazena a direção atual do movimento.

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
void inicializaConfig();
void salvarEstadoCiclo(bool cicloStartado, const String& horarioStartado, const String& ultimaLubrificacao);
void inicializarArquivoTemp();
void displayFileContent(const char* filename);
void registrarEvento(const String& fonte, bool sucesso);
void tentarReenvio();
void marcarEventoComoEnviado();
String formatarTimestamp(const DateTime& dt);
DateTime stringParaDateTime(const String& timestampStr);
float converterParaDias(unsigned long valor, int tipo);
void iniciarMovimentoMotor(float gramas, Sentido direcao);
void pararMotor();
int lerNivelBateria();
float lerTemperaturaPT100();
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
  //pinMode(BTN_EXTRA, INPUT_PULLUP);
  pinMode(SEN_NIVEL_BAIXO, INPUT);
  pinMode(SEN_NIVEL_ALTO, INPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);

  digitalWrite(MOTOR_EN, LOW);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

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

  // Verificação da data/hora do RTC
  DateTime agora = rtc.now();
  if (agora.year() < 2024) { // Checa se o ano é inválido (ex: resetou para 1990)
    Serial.println("\n[ERRO CRÍTICO] A HORA DO RTC É INVÁLIDA!");
    Serial.println("Por favor, sincronize a hora via BLE. Reinicie com acesso à internet.");
    isClockValid = false;
  } else {
    Serial.print("Hora do RTC validada: ");
    Serial.println(formatarTimestamp(agora));
    isClockValid = true;
  }

  // Prepara o sistema SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("[ERRO] Falha ao montar o SPIFFS.");
    while (true);
  }
  Serial.println("[OK] Sistema de arquivos montado.");
  
  // Carrega o arquivo config assim que o esp liga
  inicializaConfig();

  // Carrega o arquivo temp assim que o esp liga
  inicializarArquivoTemp();

  //Preparação do Lora
  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  lora.begin();
  lora.deviceId = lora.localId;
  lora.debug_serial = false;
  Serial.print("[OK] Módulo LoRa iniciado. ID Local: ");
  Serial.println(lora.localId);

  // Inicia os sensores INA226
  ina226Bateria.begin();
  if (!ina226Bateria.isConnected()) {
    Serial.println("[ERRO] Falha ao encontrar o sensor INA226 da BATERIA!");
  } else {
    Serial.println("[OK] Sensor INA226 da Bateria encontrado.");
  }

  ina226Motor.begin();
  if (!ina226Motor.isConnected()) {
    Serial.println("[ERRO] Falha ao encontrar o sensor INA226 do MOTOR!");
  } else {
    Serial.println("[OK] Sensor INA226 do Motor encontrado.");
  }
  ina226Motor.reset();
  ina226Motor.setAverage(128); // Média de 4 amostras
  ina226Motor.setBusVoltageConversionTime(3); 
  ina226Motor.setShuntVoltageConversionTime(3);

  // Passa os parâmetros de calibração para a biblioteca
  Motor.begin(PULSOS_PARA_UMA_VOLTA, GRAMAS_POR_VOLTA);
  
  // Inicia o BLE
  Serial.println("Iniciando ESP32 com BLE...");
  setupBLE();

  Serial.println("\n--- [SLAVE] Inicializado EndPoint com LoRa e BLE ---");
  Serial.println("--- Envie 'c' para ver config.json, 'l' para ver log.jsonl ou 't' oara ver o temp.json ---");

  if (configExiste == false){
    Serial.println("\nARQUIVO DE CONFIGURAÇÃO NÃO CRIADO");
  }
}

// =======================================================================================================================================





































// ==========================================================================
// --- LOOP ---
// ==========================================================================
void loop() {
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

  //----------------------------------------------------------------------------------------------------------------------------------- 
  // Recebendo e processando comandos via Bluetooth
  //-----------------------------------------------------------------------------------------------------------------------------------
  if (newDataFromBLE) {
    newDataFromBLE = false; // Reseta a flag
    Serial.println("\n[BLE] Novo comando recebido via Bluetooth!");
    Serial.print("  > Raw: ");
    Serial.println(bleReceivedValue);

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, bleReceivedValue);

    if (error) {
      Serial.println("[BLE] ERRO: Falha ao interpretar o JSON do comando.");
      updateBLE("ERRO: JSON invalido.");
      return; // Sai da lógica se o JSON estiver mal formatado
    }

    // Extrai o comando do JSON. Se não existir, será uma string vazia.
    String comando = doc["comando"] | "";

    if (comando == "set_config") {
      Serial.println("[BLE] Comando 'set_config' recebido. Salvando nova configuração...");
      
      // Pega apenas o objeto "payload" de dentro do JSON recebido
      JsonObject payload = doc["payload"];
      
      if (payload.isNull()) {
        Serial.println("[BLE] ERRO: Comando 'set_config' recebido sem 'payload'.");
        updateBLE("ERRO: 'payload' ausente.");
      } else {
        // Converte o payload de volta para uma string para usar a função de salvar existente
        String payloadStr;
        serializeJson(payload, payloadStr);
        salvarConfig(payloadStr.c_str());
        updateBLE("OK: Configuracao recebida."); // Envia uma confirmação de sucesso
      }

    } else {
      Serial.print("[BLE] ERRO: Comando desconhecido recebido: '");
      Serial.print(comando);
      Serial.println("'");
      updateBLE("ERRO: Comando desconhecido.");
    }
  }

  //----------------------------------------------------------------------------------------------------------------------------------- 
  // Trava de segurança do RTC
  // Se a flag isClockValid for falsa, a função loop é interrompida aqui e nada abaixo é executado.
  //-----------------------------------------------------------------------------------------------------------------------------------
  if (!isClockValid) {
    return; // Pausa a execução da lógica principal. Precisa reiniciar o sistema para ajustar data/hora.
  }

  // --- Variável de cálculo da dose ---
  float dosagemCalculada = 0.0f;

  // --- Lógica de Cálculo da Dose (antes dos ciclos) ---
  // Define a 'dosagemCalculada' com base no modo
  if (configExiste) {
    if (varConfigTipoConfig == 1) { // MODO BÁSICO (Rateio)
      // Ex: 10 Meses (em dias) / 5 Dias (em dias) = 60 doses
      float duracaoTotalEmDias = converterParaDias(varConfigValorIntervalo, varConfigTipoIntervalo);
      float frequenciaEmDias = converterParaDias(varConfigFrequencia, varConfigTipoFrequencia);

      if (frequenciaEmDias > 0) { // Evita divisão por zero
        float totalDoses = duracaoTotalEmDias / frequenciaEmDias;
        if (totalDoses > 0) { // Evita divisão por zero
          dosagemCalculada = (float)varConfigVolume / totalDoses; // Ex: 500g / 60 doses
        }
      }
      
    } else if (varConfigTipoConfig == 2) { // MODO AVANÇADO (Dose Fixa)
      dosagemCalculada = (float)varConfigVolume; // Ex: 10g
    }
  }

  //-----------------------------------------------------------------------------------------------------------------------------------  
  // CICLO MANUAL (BOTÃO)
  //-----------------------------------------------------------------------------------------------------------------------------------  
  int estadoAtualBtnManual = digitalRead(BTN_MANUAL);
  if (configExiste == true && estadoAtualBtnManual == LOW && estadoAnteriorBtnManual == HIGH && (millis() - ultimoAcionamentoBotaoManual > intervaloBloqueio)){
    Serial.println("Botão manual pressionado!");
    fonteDoEventoAtual = "Manual";

    if (dosagemCalculada > 0) {
      Serial.print("\n[CICLO MANUAL] Iniciando dosagem de ");
      Serial.print(dosagemCalculada, 3); // Imprime com 3 casas decimais
      Serial.println(" g.");
      iniciarMovimentoMotor(dosagemCalculada, HORARIO);
    } else {
      Serial.println("[ERRO] Dosagem calculada é zero. Verifique a configuração.");
    }

    // Salva o horario do ciclo atual...
    ultimoAcionamentoBotaoManual = millis();
    String timestampAtual = formatarTimestamp(rtc.now());
    salvarEstadoCiclo(true,timestampAtual,timestampAtual);
  }
  estadoAnteriorBtnManual = estadoAtualBtnManual;
  
  //----------------------------------------------------------------------------------------------------------------------------------- 
  // CICLO AUTOMÁTICO
  //-----------------------------------------------------------------------------------------------------------------------------------
  if (configExiste == true && varTempCicloStartado) {
    DateTime agora = rtc.now();
    DateTime ultimaLubrificacaoDT = stringParaDateTime(varTempUltimaLubrificacao);
    uint32_t agora_ts = agora.unixtime();
    uint32_t ultima_lub_ts = ultimaLubrificacaoDT.unixtime();

    // ***** ALTERAÇÃO CRÍTICA AQUI *****
    // Usa o 'varConfigIntervaloTrigger_ms' (Frequência) para o gatilho
    uint32_t intervalo_s = varConfigIntervaloTrigger_ms / 1000;

    // Verifica se o tempo de trigger foi atingido (ex: 5 dias)
    if (agora_ts >= (ultima_lub_ts + intervalo_s)) {
      Serial.println("\n[CICLO AUTOMÁTICO] Tempo de lubrificação atingido!");
      fonteDoEventoAtual = "Auto";

      if (dosagemCalculada > 0) {
        Serial.print("\n[CICLO AUTOMÁTICO] Iniciando dosagem de ");
        Serial.print(dosagemCalculada, 3); // Imprime com 3 casas decimais
        Serial.println(" g.");
        iniciarMovimentoMotor(dosagemCalculada, HORARIO);
      } else {
        Serial.println("[ERRO] Dosagem calculada é zero. Verifique a configuração.");
      }

      // Atualiza a variável da última lubrificação com a hora atual
      varTempUltimaLubrificacao = formatarTimestamp(agora);
      salvarEstadoCiclo(true, varTempHorarioStartado, varTempUltimaLubrificacao);
    }
  }
  
  //----------------------------------------------------------------------------------------------------------------------------------- 
  // Lógica dos sensores quando o motor estiver em funcionamento
  //-----------------------------------------------------------------------------------------------------------------------------------
  

if (motorLigado) { // Esta flag é definida como 'true' por iniciarMovimentoMotor
    // --- Verificações de FALHA (Sensores e Corrente) ---
    // 1. Verifica se os sensores de fim de curso foram atingidos

    if (motorDirecaoAtual == HORARIO && digitalRead(SEN_NIVEL_BAIXO) == LOW){
      Serial.println("[MOTOR] FALHA: Sensor de baixo nível atingido!");
      pararMotor(); // Para o motor via biblioteca
      registrarEvento(fonteDoEventoAtual, false); //Registra a lubrificação como falha
      motorLigado = false; // Reseta a flag principal
    }

    if (motorDirecaoAtual == ANTI_HORARIO && digitalRead(SEN_NIVEL_ALTO) == LOW) {
      Serial.println("[MOTOR] AVISO: Sensor de alto nível atingido!");
      pararMotor(); // Para o motor via biblioteca
      motorLigado = false; // Reseta a flag principal
      // (Não registra evento, pois isso é um reabastecimento, não uma dosagem)
    }
    
    // 2. Verificação contínua do valor da corrente (Cálculo Manual CORRIGIDO)
    // A Lei de Ohm é I = V / R
    // Se V está em miliVolts (mV) e R está em Ohms (Ohm), o resultado I é em miliAmperes (mA).
    
    float shuntV_motor_mV = ina226Motor.getShuntVoltage_mV();
    float current_motor_mA = shuntV_motor_mV / SHUNT_MOTOR_OHMS; // (mV / Ohms = mA)

    // Agora comparamos mA com mA (2000.0)
    if (current_motor_mA > CORRENTE_MAXIMA_MOTOR) {
      Serial.print("[MOTOR] FALHA: SOBRECARGA DETECTADA! (");
      Serial.print(current_motor_mA);
      Serial.println(" mA)");
      pararMotor(); // Para o motor via biblioteca
      registrarEvento(fonteDoEventoAtual, false); // Registra o evento como falha
      motorLigado = false; // Reseta a flag principal
    }

    // --- Verificação de SUCESSO (Alvo de Pulsos) ---
    if (motorLigado) // Apenas continue se ainda estiver ligado
    {
      MotorStatus status = Motor.atualizar(); 
      if (status == MOTOR_ALVO_ATINGIDO) {
        Serial.println("[MOTOR] SUCESSO: Alvo de pulsos atingido.");
        registrarEvento(fonteDoEventoAtual, true);
        motorLigado = false;
      }
    }
  }
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
void inicializaConfig() {
  if (SPIFFS.exists(CONFIG_FILE)) {
    configExiste = true;
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
    configExiste = false;
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

  // Carrega os valores básicos
  if (!doc["Uuid"].isNull()) varConfigUuid = doc["Uuid"].as<String>();
  if (!doc["Tag"].isNull()) varConfigTag = doc["Tag"].as<String>();
  if (!doc["Equipamento"].isNull()) varConfigEquipamento = doc["Equipamento"].as<String>();
  if (!doc["Setor"].isNull()) varConfigSetor = doc["Setor"].as<String>();
  if (!doc["Lubrificante"].isNull()) varConfigLubrificante = doc["Lubrificante"].as<String>();
  if (!doc["TipoConfig"].isNull()) varConfigTipoConfig = doc["TipoConfig"].as<int>();
  if (!doc["Volume"].isNull()) varConfigVolume = doc["Volume"].as<int>();
  if (!doc["UltimaConexao"].isNull()) varConfigUltimaConexao = doc["UltimaConexao"].as<String>();
  
  // --- Bloco da DURAÇÃO TOTAL (Intervalo) ---
  // Este bloco calcula a DURAÇÃO TOTAL (ex: 10 meses em ms)
  if (!doc["Intervalo"].isNull()) {
    varConfigValorIntervalo = doc["Intervalo"].as<unsigned long>(); // Salva o "10"
    varConfigTipoIntervalo = doc["TipoIntervalo"].as<int>();       // Salva o "Mês"

    unsigned long multiplicador = 1000; // Padrão é segundos

    switch (varConfigTipoIntervalo) {
      case 1: multiplicador = 3600000UL; break; // Hora
      case 2: multiplicador = 86400000UL; break; // Dia
      case 3: multiplicador = 2592000000UL; break; // Mês (30 dias)
      default: multiplicador = 1000; break;
    }
    // Salva a DURAÇÃO TOTAL em milissegundos
    varConfigIntervalo = varConfigValorIntervalo * multiplicador; 
  }

  // --- NOVO Bloco de FREQUÊNCIA (Trigger) ---
  // Este bloco calcula o GATILHO (ex: 5 dias em ms)
  if (!doc["Frequencia"].isNull()) {
    varConfigFrequencia = doc["Frequencia"].as<unsigned long>();     // Salva o "5"
    varConfigTipoFrequencia = doc["TipoFrequencia"].as<int>(); // Salva o "Dia"

    unsigned long multiplicador = 1000; // Padrão é segundos

    switch (varConfigTipoFrequencia) {
      case 1: multiplicador = 3600000UL; break; // Hora
      case 2: multiplicador = 86400000UL; break; // Dia
      case 3: multiplicador = 2592000000UL; break; // Mês
      default: multiplicador = 1000; break;
    }
    // Salva o INTERVALO DE TRIGGER em milissegundos
    varConfigIntervaloTrigger_ms = varConfigFrequencia * multiplicador;
  }

  // --- Visualização para Debug (Atualizada) ---
  Serial.println("[CONFIG] Variáveis globais carregadas:");
  Serial.print("  > Uuid: "); Serial.println(varConfigUuid);
  Serial.print("  > Tag: "); Serial.println(varConfigTag);
  Serial.print("  > Equipamento: "); Serial.println(varConfigEquipamento);
  Serial.print("  > Setor: "); Serial.println(varConfigSetor);
  Serial.print("  > Lubrificante: "); Serial.println(varConfigLubrificante);
  Serial.print("  > TipoConfig: "); Serial.println(varConfigTipoConfig);
  Serial.print("  > Volume: "); Serial.println(varConfigVolume);
  
  Serial.println(" --- DURAÇÃO TOTAL ---");
  Serial.print("  > Duração (Valor): "); Serial.println(varConfigValorIntervalo);
  Serial.print("  > Duração (Tipo): "); Serial.println(varConfigTipoIntervalo);
  Serial.print("  > Duração (Total ms): "); Serial.print(varConfigIntervalo); Serial.println(" ms");

  Serial.println(" --- FREQUÊNCIA (TRIGGER) ---");
  Serial.print("  > Frequência (Valor): "); Serial.println(varConfigFrequencia);
  Serial.print("  > Frequência (Tipo): "); Serial.println(varConfigTipoFrequencia);
  Serial.print("  > Frequência (Trigger ms): "); Serial.print(varConfigIntervaloTrigger_ms); Serial.println(" ms");


  Serial.print("  > TipoIntervalo: "); Serial.println(varConfigTipoIntervalo);
  Serial.print("  > Intervalo: "); Serial.print(varConfigIntervalo); Serial.println(" ms"); // Mostra a variável correta
  Serial.print("  > UltimaConexao: "); Serial.println(varConfigUltimaConexao);
}

// Regista a lubrificação no log.json
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

  while (lines.size() >= limiteLinhasLog) {
    lines.erase(lines.begin());
  }

  JsonDocument doc;
  doc["Hora"] = formatarTimestamp(rtc.now());   // Data/Hora atual
  doc["Sucesso"] = sucesso;                     // Indica se deu erro conforme valor do sensor INA226
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

  Serial.print("[LOG] Novo evento registrado. Sucesso: ");
  Serial.print(sucesso ? "SIM" : "NÃO");
  Serial.print(". O log agora contém ");
  Serial.print(lines.size());
  Serial.println(" linhas.");
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

// Carrega os valores do arquivo local temporario para as variáveis do programa
void inicializarArquivoTemp() {
  if (SPIFFS.exists(TEMP_FILE)) {
    configExiste = true;
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

/**
 * @brief Converte um valor e tipo (Hora, Dia, Mês) para uma contagem total em DIAS.
 * Usa float para precisão.
 */
float converterParaDias(unsigned long valor, int tipo) {
  // public enum INTERVALTYPE {NONE, HORA, DIA, MES } Hora = 1 / Dia = 2 / Mês = 3
  switch (tipo) {
    case 1: // Hora
      return (float)valor / 24.0f;
    case 2: // Dia
      return (float)valor;
    case 3: // Mês
      // Usamos uma média de 30 dias por mês para este cálculo
      return (float)valor * 30.0f;
    default: // NONE ou Segundos (não deveriam ser usados para rateio)
      return (float)valor / 86400.0f; // Converte segundos para dias
  }
}

int lerNivelBateria() {
  // getBusVoltage_V() retorna a voltagem diretamente em Volts
  float voltagem = ina226Bateria.getBusVoltage();
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

// ==========================================================================
// --- FUNÇÕES DE CONTROLE DO MOTOR ---
// ==========================================================================

/**
 * @brief Inicia o movimento do motor com base nas gramas.
 * Esta função agora chama a biblioteca NÃO-BLOQUEANTE.
 */
void iniciarMovimentoMotor(float gramas, Sentido direcao) {
  // A flag 'motorLigado' será gerenciada pela biblioteca
  if (Motor.estaGirando()) {
    Serial.println("[MOTOR] Comando ignorado, motor ja em movimento.");
    return; 
  }

  // Define a direção global para os sensores saberem
  motorDirecaoAtual = direcao; 
  
  // Converte nossa enum global para a enum da biblioteca
  Sentido sentidoLib = direcao;
  
  Serial.print("[MOTOR] Iniciando dosagem de ");
  Serial.print(gramas);
  Serial.println("g.");

  // Chama a função NÃO-BLOQUEANTE da biblioteca
  Motor.iniciarGiroPorGramas(gramas, sentidoLib, VELOCIDADE_MOTOR);
  
  // A flag 'motorLigado' será usada pela sua lógica de sensores
  motorLigado = true;
}

/**
 * @brief Para o motor (emergência ou fim de curso)
 */
void pararMotor() {
  Serial.println("[MOTOR] Parada de emergencia/sensor acionada.");
  Motor.pararMotor(); // Chama a função da biblioteca
  motorLigado = false;
}