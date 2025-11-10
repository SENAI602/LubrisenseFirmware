/*
 * MotorEncoder.h - Biblioteca (NÃO-BLOQUEANTE) para controle de Motor DC.
 * Refatorada para ser usada em um loop assíncrono.
 */
 
#ifndef MotorEncoder_h
#define MotorEncoder_h

#include <Arduino.h>

enum Sentido { 
  HORARIO, 
  ANTI_HORARIO 
};

// Enum para o status retornado por atualizar()
enum MotorStatus {
  MOTOR_PARADO,
  MOTOR_GIRANDO,
  MOTOR_ALVO_ATINGIDO // Novo status para sucesso
};

class MotorEncoder {
  public:
    MotorEncoder(int pinA, int pinB, int pinPWM, int pinIN1, int pinIN2);
    
    void begin(long pulsosPorVolta, float gramasPorVolta);

    // --- Funções de Comando (NÃO-BLOQUEANTES) ---
    void iniciarGiroPorGramas(float gramas, Sentido direcao, int velocidade);
    void iniciarGiroPorPulsos(long pulsos, Sentido direcao, int velocidade);

    // --- Função de Atualização (Deve ser chamada no loop principal) ---
    MotorStatus atualizar(); // A MÁGICA ACONTECE AQUI

    // Para o motor imediatamente (emergência ou alvo atingido)
    void pararMotor(); 
    
    // --- Funções Auxiliares ---
    void zerarPulsos();
    long getPulsos();
    bool estaGirando(); // Retorna _motorGirando

  private:
    int _pinA, _pinB, _pinPWM, _pinIN1, _pinIN2;
    long _pulsosPorVolta;
    float _gramasPorVolta;
    volatile long _totalPulsos;
    volatile bool _motorGirando;

    // --- Novas variáveis de estado para modo assíncrono ---
    long _pulsosAlvo;
    Sentido _direcaoAtual;
    
    static void IRAM_ATTR isrWrapper();
    void IRAM_ATTR handleEncoder();
    static MotorEncoder* instance;
};

#endif