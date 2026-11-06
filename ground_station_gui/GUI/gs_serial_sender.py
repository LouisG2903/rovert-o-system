import struct
import serial

HEADER1, HEADER2 = 0xAA, 0x88


class GroundStationSerial:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        # Abre el puerto, si no hay ESP32 queda en modo prueba en seco
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0, write_timeout=0.1)
        except (serial.SerialException, OSError):
            self.ser = None
            print(f"[GS] Sin hardware en {port}: modo dry-run")

    def _send(self, cmd_id, distance_m):
        # Paquete 4: <BBBfB>, checksum = XOR de los primeros 7 bytes
        frame = struct.pack("<BBBf", HEADER1, HEADER2, cmd_id, distance_m)
        chk = 0
        for b in frame:
            chk ^= b
        frame += bytes([chk])
        print("[GS] TX:", frame.hex(" "))
        if self.ser:
            try:
                self.ser.write(frame)
            except (serial.SerialException, OSError):
                print("[GS] Error de escritura serial")

    def send_advance(self, distance_m: float):
        # Solo envía si la distancia es positiva
        if distance_m > 0:
            self._send(1, distance_m)

    def send_stop(self):
        self._send(0, 0.0)
