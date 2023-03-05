#ifndef _SENSORS

#define _SENSORS

#define INA260_SYS_ADDR 0x40
#define INA260_PROBE_ADDR 0x41
#define INA260_ORIN_ADDR 0x42
#define INA260_DISP_ADDR 0x44
#define INA260_CAM_ADDR 0x45


#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA260.h>

#include "Config.h"

Adafruit_BME280 _bme; // I2C
Adafruit_INA260 _ina260_sys = Adafruit_INA260();
Adafruit_INA260 _ina260_probe = Adafruit_INA260();
Adafruit_INA260 _ina260_orin = Adafruit_INA260();
Adafruit_INA260 _ina260_disp = Adafruit_INA260();
Adafruit_INA260 _ina260_cam = Adafruit_INA260();

class Sensors {

    private:
        bool sensorsValid;
   
    public:

        float voltage[5];
        float current[5];
        float power[5];
        float temperature;
        float pressure;
        float humidity;

        Sensors() {
            sensorsValid = false;

        }

        bool begin() {

            sensorsValid = true;
            
            if (!_ina260_sys.begin(INA260_SYS_ADDR)) {
                DEBUGPORT.println("Couldn't find INA260 A chip");
                sensorsValid = false;
            }
            else {
                DEBUGPORT.println("System INA260 OK");
            }

            if (!_ina260_probe.begin(INA260_PROBE_ADDR)) {
                DEBUGPORT.println("Couldn't find INA260 B chip");
                sensorsValid = false;
            }
            else {
                DEBUGPORT.println("Strobe INA260 OK");
            }

            if (!_ina260_orin.begin(INA260_ORIN_ADDR)) {
                DEBUGPORT.println("Couldn't find INA260 B chip");
                sensorsValid = false;
            }
            else {
                DEBUGPORT.println("Strobe INA260 OK");
            }

            if (!_ina260_disp.begin(INA260_DISP_ADDR)) {
                DEBUGPORT.println("Couldn't find INA260 B chip");
                sensorsValid = false;
            }
            else {
                DEBUGPORT.println("Strobe INA260 OK");
            }

            if (!_ina260_cam.begin(INA260_CAM_ADDR)) {
                DEBUGPORT.println("Couldn't find INA260 B chip");
                sensorsValid = false;
            }
            else {
                DEBUGPORT.println("Strobe INA260 OK");
            }
            
            
            // default settings
            int status = _bme.begin();  
            // You can also pass in a Wire library object like &Wire2
            // status = bme.begin(0x76, &Wire2)
            if (!status) {
                DEBUGPORT.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
                DEBUGPORT.print("SensorID was: 0x"); Serial.println(_bme.sensorID(),16);
                DEBUGPORT.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
                DEBUGPORT.print("   ID of 0x56-0x58 represents a BMP 280,\n");
                DEBUGPORT.print("        ID of 0x60 represents a BME 280.\n");
                DEBUGPORT.print("        ID of 0x61 represents a BME 680.\n");
                // while (1) delay(10);
                sensorsValid = false;
            }

            return sensorsValid;

        }

        void update() {
            if (!sensorsValid)
                return;
            temperature = _bme.readTemperature();
            pressure = _bme.readPressure();
            humidity = _bme.readHumidity();
            current[0] = _ina260_sys.readCurrent();
            voltage[0] = _ina260_sys.readBusVoltage();
            power[0] = _ina260_sys.readPower();
            current[1] = _ina260_probe.readCurrent();
            voltage[1] = _ina260_probe.readBusVoltage();
            power[1] = _ina260_probe.readPower();
            current[2] = _ina260_orin.readCurrent();
            voltage[2] = _ina260_orin.readBusVoltage();
            power[2] = _ina260_orin.readPower();
            current[3] = _ina260_disp.readCurrent();
            voltage[3] = _ina260_disp.readBusVoltage();
            power[3] = _ina260_disp.readPower();
            current[4] = _ina260_cam.readCurrent();
            voltage[4] = _ina260_cam.readBusVoltage();
            power[4] = _ina260_cam.readPower();
        }

        void printEnv() {
            if (!sensorsValid)
                return;
            temperature = _bme.readTemperature();
            pressure = _bme.readPressure();
            humidity = _bme.readHumidity();
            String output = "$BME280," + String(temperature) + "," + String(pressure) + "," + String(humidity);
            UI1.println(output);
            UI2.println(output);
        }

        void printPower() {
            if (!sensorsValid)
                return;
            current[0] = _ina260_sys.readCurrent();
            voltage[0] = _ina260_sys.readBusVoltage();
            power[0] = _ina260_sys.readPower();

            String output = "$PWR_SYS," + String(current[0]) + "," + String(voltage[0]) + "," + String(power[0]);
            UI1.println(output);
            UI2.println(output);

            current[1] = _ina260_probe.readCurrent();
            voltage[1] = _ina260_probe.readBusVoltage();
            power[1] = _ina260_probe.readPower();

            output = "$PWR_PROBE," + String(current[1]) + "," + String(voltage[1]) + "," + String(power[1]);
            UI1.println(output);
            UI2.println(output);

            current[2] = _ina260_orin.readCurrent();
            voltage[2] = _ina260_orin.readBusVoltage();
            power[2] = _ina260_orin.readPower();

            output = "$PWR_ORIN," + String(current[1]) + "," + String(voltage[1]) + "," + String(power[1]);
            UI1.println(output);
            UI2.println(output);

            current[3] = _ina260_disp.readCurrent();
            voltage[3] = _ina260_disp.readBusVoltage();
            power[3] = _ina260_disp.readPower();

            output = "$PWR_DISP," + String(current[1]) + "," + String(voltage[1]) + "," + String(power[1]);
            UI1.println(output);
            UI2.println(output);

            current[4] = _ina260_cam.readCurrent();
            voltage[4] = _ina260_cam.readBusVoltage();
            power[4] = _ina260_cam.readPower();

            output = "$PWR_CAM," + String(current[1]) + "," + String(voltage[1]) + "," + String(power[1]);
            UI1.println(output);
            UI2.println(output);
            
        }
};

#endif