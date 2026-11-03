#include "Motor.h"

// ============================================================
//  Construcción e inicialización
// ============================================================

Motor::Motor(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, bool inverted)
    : _pwmPin(pwmPin), _in1Pin(in1Pin), _in2Pin(in2Pin), _inverted(inverted)
{
    // Vacío a propósito: aquí NO se toca hardware (ver begin()).
}

void Motor::begin()
{
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);

    // Soporta automáticamente Arduino-ESP32 core 3.x (ledcAttach) y core 2.x (ledcSetup + ledcAttachPin)
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    ledcAttach(_pwmPin, PWM_FREQ_HZ, PWM_RES_BITS);
#else
    static uint8_t nextChannel = 0;
    _ledcChannel = nextChannel++;
    ledcSetup(_ledcChannel, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(_pwmPin, _ledcChannel);
#endif

    stop();   // arrancar siempre en estado seguro
}

// ============================================================
//  Comandos públicos
// ============================================================

void Motor::setDuty(float duty)
{
    // 1. Corregir montaje en espejo
    if (_inverted) duty = -duty;

    // 2. Proteger contra valores fuera de rango (p. ej. error en la cinemática)
    duty = constrain(duty, -1.0f, 1.0f);
    _duty = duty;

    // 3. Primero dirección, luego PWM → evita un pulso en sentido equivocado
    applyDirection(duty);
    applyPwm(fabsf(duty));
}

void Motor::brake()
{
    // Tabla TB6612: IN1=1, IN2=1 → "short brake"
    digitalWrite(_in1Pin, HIGH);
    digitalWrite(_in2Pin, HIGH);
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    ledcWrite(_pwmPin, 0);
#else
    ledcWrite(_ledcChannel, 0);
#endif
    _duty = 0.0f;
}

void Motor::stop()
{
    // Tabla TB6612: IN1=0, IN2=0 → salida en alta impedancia (rueda libre)
    digitalWrite(_in1Pin, LOW);
    digitalWrite(_in2Pin, LOW);
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    ledcWrite(_pwmPin, 0);
#else
    ledcWrite(_ledcChannel, 0);
#endif
    _duty = 0.0f;
}

// ============================================================
//  Helpers privados
// ============================================================

void Motor::applyDirection(float duty)
{
    // Tabla de verdad TB6612FNG (por canal):
    //   IN1  IN2  | Salida
    //    1    0   | CW   (adelante por convención)
    //    0    1   | CCW  (atrás)
    //    0    0   | stop (libre)
    //    1    1   | brake
    //
    // Si TODOS los motores giran al revés de lo esperado, invierte aquí
    // por si acaso los dos comparadores (> por <). Si solo algunos, usa 'inverted'.
    digitalWrite(_in1Pin, duty > 0.0f ? HIGH : LOW);
    digitalWrite(_in2Pin, duty < 0.0f ? HIGH : LOW);
}

void Motor::applyPwm(float magnitude)
{
    // Elevar al deadband: un duty de 0.05 no mueve el motor pero sí
    // lo calienta. Duty = 0 exacto se respeta (motor parado).
    if (magnitude > 0.0f && magnitude < _deadband) magnitude = _deadband;

#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    ledcWrite(_pwmPin, (uint32_t)(magnitude * PWM_MAX));
#else
    ledcWrite(_ledcChannel, (uint32_t)(magnitude * PWM_MAX));
#endif
}
