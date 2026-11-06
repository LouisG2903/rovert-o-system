#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is connected to GPIO15
#define ONE_WIRE_BUS 4
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

DeviceAddress sensor1 = { 0x28, 0xD3, 0x76, 0x48, 0x00, 0x00, 0x00, 0x8E };
DeviceAddress sensor2 = { 0x28, 0x24, 0xF5, 0x47, 0x00, 0x00, 0x00, 0x66 };
DeviceAddress sensor3 = { 0x28, 0x78, 0xEE, 0x48, 0x00, 0x00, 0x00, 0xA3 };

void setup(void){
  Serial.begin(115200);
  sensors.begin();
}

void loop(void){ 
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  
  Serial.print("Sensor 1(°C): ");
  Serial.println(sensors.getTempC(sensor1)); 
 
  Serial.print("Sensor 2(°C): ");
  Serial.println(sensors.getTempC(sensor2)); 

  Serial.print("Sensor 3(°C): ");
  Serial.println(sensors.getTempC(sensor3)); 
 
  delay(2000);
}
