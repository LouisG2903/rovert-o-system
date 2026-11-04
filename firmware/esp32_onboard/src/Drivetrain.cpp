#include "Drivetrain.h"

// ============================================================
//  Construcción e inicialización
// ============================================================

Drivetrain::Drivetrain(Motor& fl, Motor& rl, Motor& fr, Motor& rr, uint8_t stbyPin)
    : _fl(fl), _rl(rl), _fr(fr), _rr(rr), _stbyPin(stbyPin) {}

void Drivetrain::begin()
{
    pinMode(_stbyPin, OUTPUT);
    digitalWrite(_stbyPin, LOW);    // drivers apagados mientras configuramos

    _fl.begin(); _rl.begin();
    _fr.begin(); _rr.begin();

    digitalWrite(_stbyPin, HIGH);   // habilitar drivers
    stop();
}

float Drivetrain::minSpeed() const
{
    // Aproximación: todos los motores comparten el mismo deadband.
    // Si cambias el deadband en Motor, este valor lo refleja solo.
    return 0.15f * _vMax;   // [AJUSTAR] si cambias _deadband en Motor.h
}

// ============================================================
//  Maniobras
// ============================================================

void Drivetrain::forward(float s, uint32_t ms)
{
    float v = resolveSpeed(s);
    setSpeed(+v, +v, ms);
}

void Drivetrain::backward(float s, uint32_t ms)
{
    float v = resolveSpeed(s);
    setSpeed(-v, -v, ms);
}

void Drivetrain::turnLeft(float s, uint32_t ms)
{
    // Pivote: izquierda atrás, derecha adelante → gira CCW visto desde arriba
    float v = resolveSpeed(s);
    setSpeed(-v, +v, ms);
}

void Drivetrain::turnRight(float s, uint32_t ms)
{
    float v = resolveSpeed(s);
    setSpeed(+v, -v, ms);
}

void Drivetrain::setSpeed(float leftCmps, float rightCmps, uint32_t ms)
{
    // Limitar a lo físicamente posible
    _vLeft  = constrain(leftCmps,  -_vMax, _vMax);
    _vRight = constrain(rightCmps, -_vMax, _vMax);

    applySide(_fl, _rl, toDuty(_vLeft));
    applySide(_fr, _rr, toDuty(_vRight));

    _moving  = (_vLeft != 0.0f || _vRight != 0.0f);
    _endTime = (ms > 0) ? millis() + ms : 0;
}

void Drivetrain::stop()
{
    _fl.stop(); _rl.stop(); _fr.stop(); _rr.stop();
    _vLeft = _vRight = 0.0f;
    _moving = false;
    _endTime = 0;
}

void Drivetrain::brake()
{
    _fl.brake(); _rl.brake(); _fr.brake(); _rr.brake();
    _vLeft = _vRight = 0.0f;
    _moving = false;
    _endTime = 0;
}

// ============================================================
//  Mantenimiento
// ============================================================

void Drivetrain::update()
{
    // Fin de maniobra temporizada. Se usa resta con unsigned para que
    // el desbordamiento de millis() (cada ~49 días) no rompa la lógica.
    if (_endTime != 0 && (int32_t)(millis() - _endTime) >= 0) {
        stop();
    }
}

// ============================================================
//  Helpers privados
// ============================================================

float Drivetrain::resolveSpeed(float requested) const
{
    float v = (requested < 0.0f) ? _vCruise : requested;
    return constrain(v, 0.0f, _vMax);
}

float Drivetrain::toDuty(float cmps) const
{
    // Lazo abierto: duty proporcional a la velocidad deseada.
    // [OJO] Motor::applyPwm aplicará el deadband; por debajo de
    //       minSpeed() la velocidad real será mayor que la pedida.
    return cmps / _vMax;
}

void Drivetrain::applySide(Motor& a, Motor& b, float duty)
{
    a.setDuty(duty);
    b.setDuty(duty);
}