#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is connected to GPIO4
#define ONE_WIRE_BUS 4
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// Define UART pins
#define RXD2 16
#define TXD2 17

DeviceAddress sensor1 = { 0x28, 0xD3, 0x76, 0x48, 0x00, 0x00, 0x00, 0x8E };
DeviceAddress sensor2 = { 0x28, 0x24, 0xF5, 0x47, 0x00, 0x00, 0x00, 0x66 };
DeviceAddress sensor3 = { 0x28, 0x78, 0xEE, 0x48, 0x00, 0x00, 0x00, 0xA3 };

void setup(void){
  // Serial for USB debugging communication
  Serial.begin(9600);

  // Serial2 for Rasperry Pi communication
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("ESP32 UART Bridge Ready");
  sensors.begin();
}

void loop(void){
  
  // Data display for serial monitor debugger
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Request temperatures
  Serial.println("DONE");

  float t1 = sensors.getTempC(sensor1);
  float t2 = sensors.getTempC(sensor2);
  float t3 = sensors.getTempC(sensor3);

  Serial.print("Sensor 1(°C): ");
  Serial.println(t1); 
 
  Serial.print("Sensor 2(°C): ");
  Serial.println(t2); 

  Serial.print("Sensor 3(°C): ");
  Serial.println(t3); 
 
  // Send sensor data to Rasperry Pi on Serial2
  Serial2.printf("%.2f,%.2f,%.2f\n", t1, t2, t3);

  delay(1000);
}

