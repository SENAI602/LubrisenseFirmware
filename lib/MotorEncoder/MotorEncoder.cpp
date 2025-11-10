/*
 * MotorEncoder.cpp - Implementação (NÃO-BLOQUEANTE) da biblioteca.
 */
 
#include "MotorEncoder.h"

MotorEncoder* MotorEncoder::instance = NULL;

MotorEncoder::MotorEncoder(int pinA, int pinB, int pinPWM, int pinIN1, int pinIN2) {
  _pinA = pinA; _pinB = pinB; _pinPWM = pinPWM; _pinIN1 = pinIN1; _pinIN2 = pinIN2;
  _totalPulsos = 0; _motorGirando = false; _pulsosPorVolta = 0; _gramasPorVolta = 0.0;
  instance = this;
}

void MotorEncoder::begin(long pulsosPorVolta, float gramasPorVolta) {
  _pulsosPorVolta = pulsosPorVolta;
  _gramasPorVolta = gramasPorVolta;
  pinMode(_pinA, INPUT_PULLUP); pinMode(_pinB, INPUT_PULLUP);
  pinMode(_pinPWM, OUTPUT); pinMode(_pinIN1, OUTPUT); pinMode(_pinIN2, OUTPUT);
  pararMotor();
  attachInterrupt(digitalPinToInterrupt(_pinA), isrWrapper, RISING);
}

// NOVA FUNÇÃO: Calcula pulsos e chama a função de iniciar
void MotorEncoder::iniciarGiroPorGramas(float gramas, Sentido direcao, int velocidade) {
  if (_gramasPorVolta == 0 || _pulsosPorVolta == 0 || gramas <= 0) return;
  float voltasNecessarias = gramas / _gramasPorVolta;
  long pulsosNecessarios = (long)round(voltasNecessarias * _pulsosPorVolta);
  if (pulsosNecessarios > 0) {
    iniciarGiroPorPulsos(pulsosNecessarios, direcao, velocidade);
  }
}

// MODIFICADA: Apenas liga o motor e define o alvo. NÃO TEM 'while'.
void MotorEncoder::iniciarGiroPorPulsos(long pulsos, Sentido direcao, int velocidade) {
  if (_motorGirando) return; // Já está em movimento
  
  _motorGirando = true;
  zerarPulsos(); 
  _pulsosAlvo = pulsos;
  _direcaoAtual = direcao;

  if (direcao == HORARIO) {
    digitalWrite(_pinIN1, HIGH);
    digitalWrite(_pinIN2, LOW);
  } else { // ANTI_HORARIO
    digitalWrite(_pinIN1, LOW);
    digitalWrite(_pinIN2, HIGH);
  }
  analogWrite(_pinPWM, velocidade); 
  // A função termina imediatamente, o loop principal não é bloqueado.
}

/**
 * @brief Função principal de atualização. Deve ser chamada no loop().
 * @return O status atual do motor (MOTOR_GIRANDO ou MOTOR_ALVO_ATINGIDO)
 */
MotorStatus MotorEncoder::atualizar() {
  if (!_motorGirando) {
    return MOTOR_PARADO; // Não está fazendo nada
  }

  bool alvoAtingido = false;

  // Verifica se o alvo foi atingido
  if (_direcaoAtual == HORARIO && _totalPulsos >= _pulsosAlvo) {
    alvoAtingido = true;
  } else if (_direcaoAtual == ANTI_HORARIO && _totalPulsos <= -_pulsosAlvo) {
    alvoAtingido = true;
  }

  if (alvoAtingido) {
    pararMotor();
    return MOTOR_ALVO_ATINGIDO; // Sucesso!
  }

  return MOTOR_GIRANDO; // Ainda em movimento
}

// MODIFICADA: 'pararMotor' agora também reseta o estado '_motorGirando'
void MotorEncoder::pararMotor() {
  analogWrite(_pinPWM, 0);
  digitalWrite(_pinIN1, LOW);
  digitalWrite(_pinIN2, LOW);
  _motorGirando = false; 
}

bool MotorEncoder::estaGirando() {
  return _motorGirando;
}

// --- Funções Auxiliares e ISR (Sem alterações) ---
void MotorEncoder::zerarPulsos() {
  noInterrupts();
  _totalPulsos = 0;
  interrupts();
}
long MotorEncoder::getPulsos() { return _totalPulsos; }
void IRAM_ATTR MotorEncoder::isrWrapper() { if (instance) instance->handleEncoder(); }
void IRAM_ATTR MotorEncoder::handleEncoder() {
  if (digitalRead(_pinB) == LOW) _totalPulsos++;
  else _totalPulsos--;
}

