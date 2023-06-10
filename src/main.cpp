#include "Config.h"
#include "SystemControl.h"
#include "Sensors.h"
#include "SystemTrigger.h"
#include "SystemConfig.h"
#include "Utils.h"

// Global system control variable
SystemControl sys;

int powerButtonCounter = 0;
int powerButtonTimer = 0;

// wrapper for turning system on
void turnOnCamera() {
    sys.turnOnCamera();
}

// wrapper for turning system on
void powerButtonEvent() {
    if (!sys.cameraIsOn()) {
        sys.turnOnCamera();
    }
    else {
        powerButtonCounter++;
        if (powerButtonCounter > 3) {
            sys.sendShutdown();
            powerButtonCounter = 0;
        }
    }
}



void setup() {

    //Turn off strobe and camera power
    pinMode(CAM_POWER, OUTPUT);
    pinMode(DISP_POWER, OUTPUT);
    pinMode(ORIN_POWER, OUTPUT);
    pinMode(PROBE_POWER, OUTPUT);

    digitalWrite(CAM_POWER, LOW);
    digitalWrite(DISP_POWER, LOW);
    digitalWrite(ORIN_POWER, LOW);
    digitalWrite(PROBE_POWER, HIGH);

    // Setup Sd Card Pins
    //pinMode(SDCARD_DETECT, INPUT_PULLUP);

    // Power On Pin
    pinMode(POWER_SWITCH, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(POWER_SWITCH), powerButtonEvent, CHANGE);

    // Start the debug port
    DEBUGPORT.begin(115200);

    delay(4000);

    // Wait until serial port is opened
    //while (!Serial) { delay(10); }
    //while (!Serial0) { delay(10); }

    // Startup all system processes
    sys.begin();

    // Add config parameters for system
    // IMPORTANT: add parameters at t he end of the list, otherwise you'll need to reflash the saved params in EEPROM before reading
    sys.cfg.addParam(LOGINT, "Time in ms between log events", "ms", 0, 100000, 250);
    sys.cfg.addParam(LOCALECHO, "When > 0, echo serial input", "", 0, 1, 1);
    sys.cfg.addParam(CMDTIMEOUT, "time in ms before timeout waiting for user input", "ms", 1000, 100000, 10000);
    sys.cfg.addParam(HWPORT0BAUD, "Serial Port 0 baud rate", "baud", 9600, 115200, 115200);
    sys.cfg.addParam(HWPORT1BAUD, "Serial Port 1 baud rate", "baud", 9600, 115200, 115200);
    sys.cfg.addParam(HWPORT2BAUD, "Serial Port 2 baud rate", "baud", 9600, 115200, 115200);
    sys.cfg.addParam(HWPORT3BAUD, "Serial Port 3 baud rate", "baud", 9600, 115200, 115200);
    sys.cfg.addParam(LOWVOLTAGE, "Voltage in mV where we shut down system", "mV", 10000, 14000, 11500);
    sys.cfg.addParam(STANDBY, "If voltage is low go into standby mode", "", 0, 1, 0);
    sys.cfg.addParam(CHECKHOURLY, "0 = check every minute, 1 = check every hour", "", 0, 1, 0);
    sys.cfg.addParam(STARTUPTIME, "Time in seconds before performing any system checks", "s", 0, 60, 10);
    sys.cfg.addParam(WATCHDOG, "0 = no watchdog, 1 = hardware watchdog timer with 8 sec timeout","", 0, 1, 0);
    sys.cfg.addParam(CAMGUARD,"Time guard between power ON/OFF events in seconds", "s", 10, 120, 30);
    sys.cfg.addParam(TEMPLIMIT, "Temerature in C where controller will shutdown and power off camera","C", 0, 80, 55);
    sys.cfg.addParam(HUMLIMIT, "Humidity in % where controller will shutdown and power off camera","%", 0, 100, 60);
    sys.cfg.addParam(MAXSHUTDOWNTIME, "Max time in seconds we wait before cutting power to camera", "s", 15, 600, 60);
    sys.cfg.addParam(CHECKINTERVAL, "Time in seconds between check for bad operating evironment", "s", 10, 3600, 30);

    // configure watchdog timer if enabled
    sys.configWatchdog();

    // Start the remaining serial ports
    HWPORT0.begin(sys.cfg.getInt(HWPORT0BAUD));
    HWPORT1.begin(sys.cfg.getInt(HWPORT1BAUD));
    HWPORT2.begin(sys.cfg.getInt(HWPORT2BAUD));
    HWPORT3.begin(sys.cfg.getInt(HWPORT3BAUD));

    // Config the SERCOM muxes AFTER starting the ports
    configSerialPins();

    // Load the last config from EEPROM
    sys.readConfig();

    //sys.loadScheduler();
    
}

void loop() {

    sys.update();
    sys.checkInput();
    sys.checkVoltage();
    sys.checkEnv();
    sys.checkCameraPower();

    powerButtonTimer++;

    if (powerButtonTimer > 10) {
        powerButtonTimer = 0;
        powerButtonCounter = 0;
    } 

    int logInt = sys.cfg.getInt(LOGINT);

    delay(logInt);
    Blink(10, 1);

}
