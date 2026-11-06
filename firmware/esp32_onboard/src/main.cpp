#include <Arduino.h>
#include "Motor.h"
#include "Drivetrain.h"

// ---- [AJUSTAR] Pines según tu cableado y placa ----
// Se selecciona automáticamente según el entorno elegido en platformio.ini:
//   - [env:esp32s3]  -> define CONFIG_IDF_TARGET_ESP32S3
//   - [env:esp32dev] -> usa el bloque #else (ESP32 WROOM clásico)
#if CONFIG_IDF_TARGET_ESP32S3
// Pines para ESP32-S3 (GPIO 4..18 están libres)
//          pwm  in1  in2  inverted
Motor fl(   4,   5,   6 );
Motor rl(   7,  15,  16 );
Motor fr(  17,  18,   8,  true );
Motor rr(   9,  10,  11,  true );
constexpr uint8_t PIN_STBY = 12;
#else
// Pines para ESP32 DevKit WROOM (evita GPIO 6..11 conectados a la Flash SPI)
//          pwm  in1  in2  inverted
Motor fl(  13,  14,  27 );
Motor rl(  26,  25,  33 );
Motor fr(  32,  16,  17,  true );
Motor rr(  18,  19,  21,  true );
constexpr uint8_t PIN_STBY = 22;
#endif

Drivetrain drive(fl, rl, fr, rr, PIN_STBY);

void setup()
{
    Serial.begin(115200);
    drive.begin();
    drive.setCruiseSpeed(1.0f);        // [AJUSTAR] objetivo: 1 cm/s

    Serial.println("Comandos: w=adelante s=atras a=izq d=der x=stop b=freno");
    Serial.println("          +/- = crucero ±0.5 cm/s   t = adelante 3 s");
    Serial.printf ("Velocidad minima alcanzable: %.2f cm/s\n", drive.minSpeed());
}

