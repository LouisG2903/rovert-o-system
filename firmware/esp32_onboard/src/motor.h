#pragma once
#include <Arduino.h>

/**
 * Motor
 * -----
 * Controla UN motor DC a través de un canal del driver TB6612FNG.
 *
 * Cada canal del TB6612 usa 3 pines:
 *   PWM  → velocidad  (0..100 % del voltaje VM)
 *   IN1  → dirección  \  ver tabla de verdad en applyDirection()
 *   IN2  → dirección  /
 *
 * El pin STBY del driver NO se maneja aquí: es compartido entre los
 * dos canales y lo gestiona la clase de nivel superior (Drivetrain).
 *
 * Convención de signo: duty > 0 = "adelante" según el rover, no según
 * el motor. Si un motor está montado en espejo, se construye con
 * inverted = true y la clase se encarga de voltearlo.
 */
class Motor {
public:
    /**
     * @param pwmPin   GPIO conectado a PWMA o PWMB
     * @param in1Pin   GPIO conectado a AIN1 o BIN1
     * @param in2Pin   GPIO conectado a AIN2 o BIN2
     * @param inverted true si el motor está montado en espejo
     *                 (se descubre en la prueba: si gira al revés, cámbialo)
     */
    Motor(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, bool inverted = false);

    /// Configura GPIO y canal PWM. Llamar UNA vez dentro de setup().
    /// No hacerlo en el constructor: el hardware aún no está listo.
    void begin();

    /// Fija velocidad y dirección.
    // @param duty  -1.0 (atrás a tope) .. 0 (parado) .. +1.0 (adelante a tope)
    void setDuty(float duty);

    /// Freno activo: cortocircuita el motor → se detiene rápido.
    /// Usar en paradas de seguridad (watchdog, e-stop).
    void brake();

    /// Rueda libre: el motor queda desconectado → se detiene por inercia.
    /// Usar en reposo normal, calienta menos el driver que brake().
    void stop();

    /// Último duty aplicado (ya con el signo de 'inverted' corregido).
    float duty() const { return _duty; }

    /// Zona muerta: duty mínimo que el motor necesita para vencer la fricción.
    /// Cualquier valor entre 0 y este umbral se eleva al umbral.
    /// [AJUSTAR] Prueba en el test serial: baja el duty hasta que el motor
    ///           deje de girar; ese valor + 0.02 es tu deadband.
    void setDeadband(float d) { _deadband = constrain(d, 0.0f, 0.5f); }

private:
    // ---- Pines (fijos después del constructor) ----
    uint8_t _pwmPin, _in1Pin, _in2Pin;
    bool    _inverted;
    uint8_t _ledcChannel = 0;  // usado en Arduino-ESP32 core 2.x

    // ---- Estado ----
    float _duty     = 0.0f;
    float _deadband = 0.15f;   // [AJUSTAR] valor inicial típico para N20 a 12 V

    // ---- Parámetros del PWM ----
    // [AJUSTAR] PWM_FREQ_HZ: >20 kHz evita el silbido audible.
    //           El TB6612 soporta hasta 100 kHz; no bajes de ~1 kHz.
    // [OJO]     PWM_RES_BITS: a 20 kHz el ESP32-S3 admite máximo ~12 bits.
    //           Si subes la frecuencia, baja la resolución o ledcAttach falla.
    static constexpr uint32_t PWM_FREQ_HZ  = 20000;
    static constexpr uint8_t  PWM_RES_BITS = 10;                      // 0..1023
    static constexpr uint32_t PWM_MAX      = (1 << PWM_RES_BITS) - 1;

    // ---- Helpers internos ----
    void applyDirection(float duty);   // escribe IN1/IN2 según el signo
    void applyPwm(float magnitude);    // escribe el PWM aplicando deadband
};
