/**
 * =========================================================================================
 * PROJETO: FIRMWARE HÍBRIDO BLINDADO - LUBRISENSE 4.0
 * PLATAFORMA: ESP32-S3 (Framework Arduino / PlatformIO)
 * VERSÃO: 4.7 FINAL (TIMING SEGURO + BUFFER DE FRAGMENTAÇÃO + DOCUMENTAÇÃO)
 * AUTOR: Equipe de Desenvolvimento Smart Factory - FREMTEC - LubriSense 4.0
 * =========================================================================================
 * * DESCRIÇÃO GERAL:
 * Firmware para controle de lubrificador automático inteligente. Opera em modo híbrido,
 * aceitando comandos de duas interfaces de comunicação simultâneas e independentes.
 * * FUNCIONALIDADES:
 * 1. LoRaMESH (Longa Distância): Recebe comandos de acionamento (GPIO) via Gateway e envia telemetria.
 * 2. Bluetooth LE (Local): Permite configuração completa (JSON) e controle manual via App.
 * 3. Controle de Motor: Acionamento de motor DC com encoder para dosagem precisa (gramas).
 * 4. Proteção: Monitoramento de corrente (INA226) para evitar travamento/queima do motor.
 * 5. Persistência: Salva configurações e logs de eventos na memória Flash (SPIFFS).
 * * PINAGEM CRÍTICA (ESP32-S3):
 * [LoRa]   TX: GPIO 43 | RX: GPIO 44 (UART 2)
 * [Motor]  EN: 38 | IN1: 21 | IN2: 18 | Encoder A: 15 | Encoder B: 16
 * [I/O]    LED Status: 7 | Botão Manual: 2 | Nível Baixo: 5 | Nível Alto: 6
 * [I2C]    SDA: 11 | SCL: 10 (Sensores e RTC)
 * [Analog] PT100: 1
 * * =========================================================================================
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
#define GATEWAY_ID 0 
#define CMD_TO_GATEWAY_EVENT   0x20 
const size_t limiteLinhasLog = 10;
const unsigned long intervaloEnviarGateway = 30000; // Intervalo de reenvio de logs (30s)
const unsigned long TIMEOUT_ACK = 15000;            // Tempo limite para ACK do Gateway

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

// ==========================================================================
// 6. IMPLEMENTAÇÃO DO BLUETOOTH (CALLBACKS INTELIGENTES)
// ==========================================================================

/**
 * Gerencia eventos de conexão e desconexão do Bluetooth.
 * Garante reinício rápido do anúncio para reconexão automática.
 */
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println(">> [BLE] CELULAR CONECTADO!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println(">> [BLE] DESCONECTADO. Reiniciando Anuncio...");
      // Reinício imediato para evitar que o dispositivo fique invisível
      pServer->startAdvertising(); 
    }
};

/**
 * Gerencia a recepção de dados do celular.
 * Implementa um BUFFER para remontar pacotes JSON fragmentados.
 * Isso resolve o problema de pacotes grandes sendo cortados pelo Android.
 */
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      
      if (value.length() > 0) {
        String chunk = String(value.c_str());

        // CASO 1: Comando Curto (Controle de LED)
        // Processa imediatamente sem bufferizar
        if (chunk == "1" || chunk == "0") {
             bleBuffer = chunk;
             bleJsonCompleto = true;
             return;
        }

        // CASO 2: JSON Longo (Configuração)
        // O Android pode dividir o JSON em pacotes de 20 a 512 bytes.
        // Acumulamos até encontrar o caractere de fim '}'.
        bleBuffer += chunk;

        String temp = bleBuffer;
        temp.trim();
        if (temp.endsWith("}")) {
            bleJsonCompleto = true; // Sinaliza ao loop que o pacote está pronto
        }
      }
    }
};

/**
 * Inicializa a pilha Bluetooth Low Energy.
 * Configura o nome do dispositivo, serviço e características.
 */
void setupBLE() {
  // Nome visível no Scan do App (Deve coincidir com o filtro do C#)
  BLEDevice::init("LUBRISENSE_Device"); 
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Cria uma característica unificada para Leitura, Escrita e Notificação
  // Adicionado suporte a descritores para permitir notificações Android
  pCharacteristic = pService->createCharacteristic(
                      CHAR_UUID,
                      BLECharacteristic::PROPERTY_READ | 
                      BLECharacteristic::PROPERTY_WRITE | 
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  
  // Configuração de Advertising (Anúncio) para melhor compatibilidade
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); 
  pAdvertising->setMaxPreferred(0x12);
  
  BLEDevice::startAdvertising();
  
  Serial.println(">> [BLE] PRONTO E VISÍVEL (LUBRISENSE_Device)");
}

// ==========================================================================
// 7. IMPLEMENTAÇÃO DO LORA (SNIFFER CIRÚRGICO)
// ==========================================================================
/**
 * Lê o buffer serial byte a byte procurando pelo padrão de comando GPIO (0xC2).
 * Esta abordagem manual resolve o problema de travamento da biblioteca oficial
 * em loops de alta velocidade.
 */
void verificarLoRaCirurgico() {
  if (LoRaSerial.available() > 0) {
    static uint8_t buffer[20];
    static int index = 0;
    
    uint8_t byteLido = LoRaSerial.read();

    // 1. Sincronia: Espera cabeçalho ID 1 (Gateway)
    if (index == 0 && byteLido != 0x01) return; 

    buffer[index++] = byteLido;

    // 2. Tamanho do pacote de comando = 9 bytes
    if (index >= 9) {
      // 3. Assinatura: O 3º byte (índice 2) deve ser o comando 0xC2 (GPIO)
      if (buffer[2] == 0xC2) {
         // 4. O Dado: O nível (Ligar/Desligar) está no 7º byte (índice 6)
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
         
         // Solicita notificação ao App (Processado no Loop Principal)
         precisaNotificarApp = true;
      }
      index = 0; // Reseta buffer
    }
  }
}

// ==========================================================================
// 8. SETUP (INICIALIZAÇÃO DO SISTEMA)
// ==========================================================================
void setup() {
  Serial.begin(115200);
  
  // Trava de Segurança: Segure o botão manual no boot para modo de recuperação
  pinMode(BTN_MANUAL, INPUT_PULLUP);
  if (digitalRead(BTN_MANUAL) == LOW) {
      Serial.println("\n!!! MODO DE SEGURANÇA ATIVADO !!!");
      Serial.println("USB Ativa. Código principal cancelado.");
      while(1) { delay(100); }
  }
  
  delay(100); // Estabilização elétrica

  // Configuração de Pinos
  pinMode(SEN_NIVEL_BAIXO, INPUT);
  pinMode(SEN_NIVEL_ALTO, INPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(LED_PIN_GPIO7, OUTPUT);

  // Estado Inicial Seguro (Tudo Desligado)
  digitalWrite(MOTOR_EN, LOW);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(LED_PIN_GPIO7, LOW);

  // Correção de atenuação do ADC no ESP32-S3 (Para leitura correta do PT100)
  if (PT100_PIN >= 0) analogSetAttenuation(ADC_11db);

  // Inicialização I2C e RTC
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!rtc.begin()) Serial.println("[ERRO] RTC não encontrado");
  
  // Recuperação de Hora via NTP se bateria acabou
  if (rtc.lostPower()) {
    Serial.println("RTC sem hora. Tentando sincronizar via Wi-Fi...");
    if (sincronizarViaNTP()) ultimaSincronizacao = millis();
  }
  
  // Validação de Relógio (Ano base 2024)
  DateTime agora = rtc.now();
  if (agora.year() < 2024) isClockValid = false;
  else isClockValid = true;

  // Inicialização SPIFFS (Memória Flash)
  if (!SPIFFS.begin(true)) Serial.println("[ERRO] SPIFFS falhou");
  else Serial.println("[OK] Sistema de Arquivos montado");
  
  inicializaConfig();
  
  // Configuração Padrão de Fábrica
  if (!configExiste) {
    Serial.println("[INIT] Criando configuração de fábrica...");
    const char* def = "{\"Uuid\":\"novo-uuid\",\"Tag\":\"PONTO-01\",\"Equipamento\":\"MOTOR\",\"TipoConfig\":1,\"Volume\":10,\"Intervalo\":1,\"TipoIntervalo\":3,\"Frequencia\":1,\"TipoFrequencia\":2}";
    salvarConfig(def);
    configExiste = true;
  }

  inicializarArquivoTemp();

  // Inicialização LoRa
  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  lora.begin(false); 
  
  // Força ID Mestre (0) para receber comandos do Gateway
  if(lora.localId != 0) {
      lora.setnetworkId(0);
      lora.setpassword(602);
  }

  // Inicialização Sensores e Motor
  ina226Bateria.begin();
  ina226Motor.begin();
  ina226Motor.setAverage(128); 
  Motor.begin(PULSOS_PARA_UMA_VOLTA, GRAMAS_POR_VOLTA);

  // Inicialização BLE (com atraso de segurança)
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
  
  // --- TAREFA 1: VERIFICAR RÁDIO LORA (Polling Rápido) ---
  verificarLoRaCirurgico();

  // --- TAREFA 2: DIAGNÓSTICO SERIAL (Comandos PC) ---
  if (Serial.available() > 0) {
    char command = Serial.read();
    while(Serial.available() > 0) Serial.read(); // Limpeza buffer
    if (command == 'c') displayFileContent(CONFIG_FILE);
    if (command == 'l') displayFileContent(LOG_FILE);
    if (command == 't') displayFileContent(TEMP_FILE);
  }

  // --- TAREFA 3: PROCESSAR COMANDOS BLUETOOTH (BUFFERIZADO) ---
  // Processa apenas quando o JSON completo estiver no buffer
  if (bleJsonCompleto) {
      bleJsonCompleto = false; // Reseta flag
      String msg = bleBuffer;  // Copia conteúdo
      bleBuffer = "";          // Limpa buffer

      Serial.print("[BLE] Mensagem Completa: ");
      Serial.println(msg);

      if (msg.startsWith("{")) {
          // --- CORREÇÃO CRÍTICA DE TIMING (v4.5) ---
          
          // 1. Pequeno delay para estabilizar o Android (sair do Write)
          delay(100);

          // 2. Envia ACK PRIMEIRO para o celular saber que recebemos
          // Isso evita o Timeout ("Falso Erro") no App
          updateBLE("OK: Config Salva");
          Serial.println("[BLE] Ack enviado. Gravando...");
          
          // 3. DELAY EXTENDIDO (500ms): Garante que o rádio transmita
          // o OK pelo ar antes que a gravação na Flash trave o chip.
          delay(500); 
          
          // 4. Agora sim, salva na Flash (Operação Lenta)
          Serial.println("[BLE] Gravando na Flash...");
          salvarConfig(msg.c_str());
      }
      else if (msg == "1") {
          digitalWrite(LED_PIN_GPIO7, HIGH);
          estadoLedAtual = true;
          updateBLE("1"); 
      }
      else if (msg == "0") {
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

  // --- TAREFA 5: LÓGICA DE NEGÓCIO AUTOMÁTICA (DOSAGEM) ---
  if (isClockValid) {
      // Lógica de Dosagem e Motor (Resumida)
      float doseCalculada = 0.0f;
      if (configExiste) {
        if (varConfigTipoConfig == 2) doseCalculada = (float)varConfigVolume;
        else if (varConfigTipoConfig == 1) { 
             float diasTotal = converterParaDias(varConfigValorIntervalo, varConfigTipoIntervalo);
             float diasFreq = converterParaDias(varConfigFrequencia, varConfigTipoFrequencia);
             if (diasFreq > 0) {
                float numDoses = diasTotal / diasFreq;
                if (numDoses > 0) doseCalculada = (float)varConfigVolume / numDoses;
             }
        } 
      }

      // Acionamento Manual
      int btn = digitalRead(BTN_MANUAL);
      if (configExiste && btn == LOW && estadoAnteriorBtnManual == HIGH && (millis() - ultimoAcionamentoBotaoManual > intervaloBloqueio)){
        Serial.println("[MANUAL] Ciclo Iniciado via Botão");
        fonteDoEventoAtual = "Manual";
        if (doseCalculada > 0) iniciarMovimentoMotor(doseCalculada, HORARIO);
        ultimoAcionamentoBotaoManual = millis();
        salvarEstadoCiclo(true, formatarTimestamp(rtc.now()), formatarTimestamp(rtc.now()));
      }
      estadoAnteriorBtnManual = btn;

      // Acionamento Automático
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

      // Controle do Motor
      if (motorLigado) {
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

  // --- TAREFA 6: MANUTENÇÃO ---
  // Reenvio de Logs LoRa
  if (millis() - ultimoReenvio > intervaloEnviarGateway) {
    ultimoReenvio = millis();
    if (isClockValid && !aguardandoAck) tentarReenvio();
  }

  // Reconexão BLE Automática
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
// 10. IMPLEMENTAÇÃO DAS FUNÇÕES AUXILIARES
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

// Funções auxiliares mantidas
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