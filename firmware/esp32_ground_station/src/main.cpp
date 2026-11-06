#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>

// Dirección MAC de ESP32-GS
uint8_t macTerrena[] = {0x58, 0x2A, 0xBD, 0x71, 0xE5, 0xE4};

#define TOTAL_FRAGMENTOS 50      // 50 fragmentos * 250 bytes = 12,500 bytes = 100,000 bits (100 Kbits)
#define TAMANO_PAQUETE_BYTES 250 // Tamaño máximo de payload por paquete en ESP-NOW

// Estructura para transportar fragmentos del sensor LIDAR en formato .CSV
typedef struct __attribute__((packed)) {
    uint8_t tipo_msg;          // 1 = Datos LIDAR CSV
    uint16_t num_fragmento;    // Índice (1 a 50)
    uint16_t total_fragmentos; // 50
    uint32_t t_envio_us;       // Timestamp origen para medición de DELAY
    char csv_payload[241];     // Filas de lecturas en formato .CSV
} PaqueteLidar_t;

// Estructura de respuesta y control recibida desde la Terrena
typedef struct __attribute__((packed)) {
    uint8_t tipo_msg;             // 2 = Confirmación y comandos
    uint8_t orden_cmd;            // Índice de instrucción (1 a 4)
    uint32_t t_origen_us;         // Timestamp devuelto para calcular el RTT
    uint32_t delay_rafaga_ms;     // Tiempo total de transferencia de los 100 Kbits
    uint32_t bits_confirmados;    // Total de bits acumulados por la Terrena
    char confirmacion_csv[64];    // Validación de la muestra CSV
    char instruccion[16];         // "Subir", "Bajar", "Derecha", "Izquierda"
} PaqueteControl_t;

PaqueteLidar_t txLidar;
esp_now_peer_info_t peerInfo;

// Callback de recepción compatible con ESP32 Arduino Core v2.x y v3.x
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
    if (len == sizeof(PaqueteControl_t)) {
        PaqueteControl_t rxCtrl;
        memcpy(&rxCtrl, incomingData, sizeof(PaqueteControl_t));

        uint32_t ahora_us = (uint32_t)esp_timer_get_time();
        float delay_enlace_ms = (ahora_us - rxCtrl.t_origen_us) / 1000.0f;

        if (rxCtrl.orden_cmd == 1) {
            Serial.println("\n========================================================");
            Serial.println("[RX_ROVER] CONFIRMACION RECIBIDA DE ESTACION TERRENA");
            Serial.printf(" -> Bits confirmados : %lu bits (100 Kbits)\n", (unsigned long)rxCtrl.bits_confirmados);
            Serial.printf(" -> Info validada    : %s\n", rxCtrl.confirmacion_csv);
            Serial.printf(" -> DELAY de rafaga  : %lu ms (tiempo de transferencia de 100 Kbits)\n", (unsigned long)rxCtrl.delay_rafaga_ms);
            Serial.printf(" -> DELAY de enlace  : %.2f ms (latencia de ida y vuelta / RTT)\n", delay_enlace_ms);
            Serial.println("--------------------------------------------------------");
            Serial.println("[RX_ROVER] EJECUTANDO SECUENCIA DE INSTRUCCIONES:");
        }

        Serial.printf(" [%u/4] Instruccion recibida: %-10s | DELAY enlace: %.2f ms\n",
                      rxCtrl.orden_cmd, rxCtrl.instruccion, delay_enlace_ms);

        if (rxCtrl.orden_cmd == 4) {
            Serial.println("========================================================\n");
        }
    }
}

void enviarBloque100KbitsLidar() {
    Serial.println("\n--- INICIANDO ENVIO LIDAR CSV (100,000 bits / 12,500 bytes) ---");
    Serial.println("fragmento,punto_id,angulo_deg,distancia_mm,intensidad");

    uint32_t t_inicio_rafaga = (uint32_t)esp_timer_get_time();
    uint16_t punto_id = 1;

    for (uint16_t frag = 1; frag <= TOTAL_FRAGMENTOS; frag++) {
        memset(&txLidar, 0, sizeof(PaqueteLidar_t));
        txLidar.tipo_msg = 1;
        txLidar.num_fragmento = frag;
        txLidar.total_fragmentos = TOTAL_FRAGMENTOS;
        txLidar.t_envio_us = (uint32_t)esp_timer_get_time();

        // Rellenar los 241 bytes del payload con lecturas simuladas en formato .CSV
        int offset = 0;
        while (offset < (int)sizeof(txLidar.csv_payload) - 32) {
            float angulo = (punto_id * 1.44f);
            if (angulo >= 360.0f) angulo -= 360.0f;
            uint16_t distancia_mm = 500 + (esp_random() % 3500); // 500 a 4000 mm
            uint8_t intensidad = 50 + (esp_random() % 200);      // Reflectividad 50 a 250

            int escritos = snprintf(txLidar.csv_payload + offset,
                                    sizeof(txLidar.csv_payload) - offset,
                                    "%u,%.2f,%u,%u\n",
                                    punto_id, angulo, distancia_mm, intensidad);
            if (escritos < 0 || offset + escritos >= (int)sizeof(txLidar.csv_payload)) {
                break;
            }
            offset += escritos;
            punto_id++;
        }

        // Relleno de seguridad con espacios para alcanzar exactamente 250 bytes
        for (int i = offset; i < (int)sizeof(txLidar.csv_payload) - 1; i++) {
            txLidar.csv_payload[i] = ' ';
        }
        txLidar.csv_payload[sizeof(txLidar.csv_payload) - 1] = '\0';

        // Muestra representativa en consola (primer y último fragmento)
        if (frag == 1 || frag == TOTAL_FRAGMENTOS) {
            Serial.print(txLidar.csv_payload);
        }

        esp_now_send(macTerrena, (uint8_t *)&txLidar, sizeof(PaqueteLidar_t));
        delay(4); // Pausa breve para no saturar la cola de transmisión Wi-Fi
    }

    uint32_t t_fin_rafaga = (uint32_t)esp_timer_get_time();
    Serial.printf("--- FIN DE ENVIO: %d paquetes (%d bits) enviados en %.2f ms ---\n",
                  TOTAL_FRAGMENTOS, TOTAL_FRAGMENTOS * TAMANO_PAQUETE_BYTES * 8,
                  (t_fin_rafaga - t_inicio_rafaga) / 1000.0f);
}

// Paquete 4: comando PC -> ESP32-GS (8 bytes, little-endian)
typedef struct __attribute__((packed)) {
    uint8_t header1;    // 0xAA
    uint8_t header2;    // 0x88
    uint8_t cmd_id;     // 1 = ENVIAR, 0 = DETENER
    float distance_m;   // Distancia objetivo en metros
    uint8_t checksum;   // XOR de los bytes 0 al 6
} GsCommandPacket;

// XOR de los primeros len bytes
uint8_t compute_xor(const uint8_t *data, size_t len) {
    uint8_t x = 0;
    while (len--) x ^= *data++;
    return x;
}

// Punto de integración: aquí el equipo de comunicaciones debe llamar esp_now_send()
void enviar_comando_esp_now(const GsCommandPacket& pkt) {
    // esp_now_send(macRover, (const uint8_t *)&pkt, sizeof(pkt));
    Serial.printf("[GS] CMD=%u DIST=%.2f m\n", pkt.cmd_id, pkt.distance_m);
}

// Parser no bloqueante por máquina de estados; llamar en cada loop()
void poll_pc_serial() {
    static uint8_t buf[sizeof(GsCommandPacket)];
    static uint8_t idx = 0;
    while (Serial.available()) {
        uint8_t b = Serial.read();
        if (idx == 0 && b != 0xAA) continue;   // espera header1
        if (idx == 1 && b != 0x88) {           // header2 inválido: resincroniza
            idx = (b == 0xAA) ? 1 : 0;
            continue;
        }
        buf[idx++] = b;
        if (idx == sizeof(buf)) {
            idx = 0;
            if (compute_xor(buf, sizeof(buf) - 1) == buf[sizeof(buf) - 1]) {
                GsCommandPacket pkt;
                memcpy(&pkt, buf, sizeof(pkt));
                enviar_comando_esp_now(pkt);
            }
        }
    }
}

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Desactiva detector de brownout
    Serial.begin(115200);
    delay(1000);

    WiFi.mode(WIFI_STA);
    Serial.println();
    Serial.println("=========================================");
    Serial.print("[RX_ROVER] Iniciado. MAC Local Rover: ");
    Serial.println(WiFi.macAddress());
    Serial.print("[RX_ROVER] Peer Terrena configurado: ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X%s", macTerrena[i], (i < 5) ? ":" : "\n");
    }
    Serial.println("=========================================");

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERROR] No se pudo inicializar ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    memcpy(peerInfo.peer_addr, macTerrena, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ERROR] No se pudo emparejar con la Terrena");
        return;
    }
}

void loop() {
    poll_pc_serial();
    static uint32_t t_ultimo = 0;
    if (millis() - t_ultimo >= 5000) { // Ráfaga cada 5 s sin bloquear
        t_ultimo = millis();
        enviarBloque100KbitsLidar();
    }
}