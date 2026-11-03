#pragma once
#include <Arduino.h>
#include "Motor.h"

/**
 * Drivetrain
 * ----------
 * Agrupa los 4 motores en dos lados (izquierdo / derecho) y ofrece
 * MANIOBRAS de alto nivel en aproximdamante X (cm/s).
 *
 *   forward() / backward()      → ambos lados igual
 *   turnLeft() / turnRight()    → pivote sobre su eje (lados opuestos)
 *   setSpeed(vl, vr)            → primitiva: cualquier combinación
 *
 * La velocidad "de crucero" se fija una vez con setCruiseSpeed() y
 * todas las maniobras la usan salvo que se les pase otra explícitamente.
 *
 * Todas las maniobras son NO bloqueantes: fijan el estado y regresan.
 * Si se indica una duración, update() (llamado en loop) las detiene solo.
 *
 * Esta clase NO sabe nada de ROS ni de sensores: solo mueve ruedas.
 */
class Drivetrain {
public:
    /**
     * @param fl,rl   motores del lado izquierdo (delantero, trasero)
     * @param fr,rr   motores del lado derecho
     * @param stbyPin pin STBY compartido de los dos TB6612
     */
    Drivetrain(Motor& fl, Motor& rl, Motor& fr, Motor& rr, uint8_t stbyPin);

    /// Inicializa STBY y los 4 motores. Llamar en setup().
    void begin();

    // ---------- Configuración física ----------
    /// Velocidad máxima de una rueda a duty = 1.0 (se calcula de RPM y radio).
    /// [Se debe cambiar] Mide la real: duty 1.0 durante 10 s, distancia / 10.
    void setMaxWheelSpeed(float cmps) { _vMax = cmps; }

    /// Velocidad usada por defecto en todas las maniobras.
    /// [Se debe cambiar] Objetivo del proyecto: 5.0 cm/s.
    void setCruiseSpeed(float cmps) { _vCruise = constrain(cmps, 0.0f, _vMax); }
    float cruiseSpeed() const       { return _vCruise; }

    /// Velocidad mínima físicamente alcanzable (deadband × vMax).
    float minSpeed() const;

    // ---------- Maniobras ----------
    /// Si durationMs == 0 → indefinido (hasta otra orden o stop()).
    /// Si speed < 0 → usa la de crucero.
    void forward  (float speedCmps = -1, uint32_t durationMs = 0);
    void backward (float speedCmps = -1, uint32_t durationMs = 0);
    void turnLeft (float speedCmps = -1, uint32_t durationMs = 0);  // pivote CCW
    void turnRight(float speedCmps = -1, uint32_t durationMs = 0);  // pivote CW

    /// Primitiva: velocidad de cada lado en cm/s (signo = dirección).
    /// Las maniobras de arriba se construyen con esto.
    void setSpeed(float leftCmps, float rightCmps, uint32_t durationMs = 0);

    void stop();    // rueda libre
    void brake();   // freno activo (seguridad)

    // ---------- Mantenimiento ----------
    /// Llamar en cada loop(): detiene maniobras temporizadas.
    void update();

    bool isMoving() const { return _moving; }
    float leftSpeed()  const { return _vLeft; }
    float rightSpeed() const { return _vRight; }

private:
    Motor& _fl; Motor& _rl;
    Motor& _fr; Motor& _rr;
    uint8_t _stbyPin;

    // [AJUSTAR] Valores iniciales; se pueden sobreescribir con los setters.
    float _vMax    = 10.2f;   // cm/s  (30 RPM × 2π × 3.25 cm / 60)
    float _vCruise = 1.0f;    // cm/s

    float    _vLeft = 0, _vRight = 0;
    bool     _moving  = false;
    uint32_t _endTime = 0;    // millis() en que termina la maniobra (0 = nunca)

    float resolveSpeed(float requested) const;   // -1 = crucero, y limita a vMax
    float toDuty(float cmps) const;              // cm/s = -1..+1
    void  applySide(Motor& a, Motor& b, float duty);
};
