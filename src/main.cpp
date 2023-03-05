#include "Config.h"
#include "SystemControl.h"
#include "Sensors.h"
#include "SystemTrigger.h"
#include "SystemConfig.h"
#include "Utils.h"

// Global system control variable
SystemControl sys;

// High Mag Trigger Callback
void HighMagCallback()
{
    digitalWrite(HIGH_MAG_CAM_TRIG,HIGH);
    delayMicroseconds(sys.trigWidth/2);
    digitalWrite(HIGH_MAG_STROBE_TRIG,HIGH);
    if (sys.highMagStrobeDuration-FLASH_DELAY_OFFSET >= MIN_FLASH_DURATION)
        delayMicroseconds(sys.highMagStrobeDuration-FLASH_DELAY_OFFSET);
    digitalWrite(HIGH_MAG_STROBE_TRIG,LOW);
    delayMicroseconds(sys.trigWidth/2);
    digitalWrite(HIGH_MAG_CAM_TRIG,LOW);
}

// Low Mag Trigger Callback
void LowMagCallback()
{
    digitalWrite(LOW_MAG_CAM_TRIG,HIGH);
    delayMicroseconds(sys.trigWidth/2);
    digitalWrite(LOW_MAG_STROBE_TRIG,HIGH);
    if (sys.lowMagStrobeDuration-FLASH_DELAY_OFFSET >= MIN_FLASH_DURATION)
        delayMicroseconds(sys.lowMagStrobeDuration-FLASH_DELAY_OFFSET);
    digitalWrite(LOW_MAG_STROBE_TRIG,LOW);
    delayMicroseconds(sys.trigWidth/2);
    digitalWrite(LOW_MAG_CAM_TRIG,LOW);
}

// Wrapper for updaing timers and flashes from callback functions
void setTriggers() {
    sys.setTriggers();
}

void setFlashes() {
    sys.configureFlashDurations();
}

void setCTDType() {
    sys.setCTDType();
}

void setPolling() {
    sys.setPolling();
}

// wrapper for turning system on
void turnOnCamera() {
    if (sys.cfg.getInt("PROFILEMODE") == 1) {
        sys.turnOnCamera();
    }
}



void setup() {

    //Turn off strobe and camera power
    pinMode(CAMERA_POWER, OUTPUT);
    pinMode(STROBE_POWER, OUTPUT);
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
    sys.cfg.addParam(POLLFREQ, "Rate of polling instruments", "Hz", 1, 50, 10, false, setPolling);
    sys.cfg.addParam(DEPTHCHECKINTERVAL, "Time in seconds between depth checks for testing ascent/descent", "s", 10, 300, 30);
    sys.cfg.addParam(DEPTHTHRESHOLD, "Depth change threshold to denote ascent or descent", "mm", 500, 10000, 1000);
    sys.cfg.addParam(LOCALECHO, "When > 0, echo serial input", "", 0, 1, 1);
    sys.cfg.addParam(CMDTIMEOUT, "time in ms before timeout waiting for user input", "ms", 1000, 100000, 10000);
    sys.cfg.addParam(HWPORT0BAUD, "Serial Port 0 baud rate", "baud", 9600, 115200, 115200);
    sys.cfg.addParam(HWPORT1BAUD, "Serial Port 1 baud rate", "baud", 9600, 115200, 115200);
    sys.cfg.addParam(HWPORT2BAUD, "Serial Port 2 baud rate", "baud", 9600, 115200, 115200);
    sys.cfg.addParam(HWPORT3BAUD, "Serial Port 3 baud rate", "baud", 9600, 115200, 115200);
    sys.cfg.addParam(STROBEDELAY, "Time between camera trigger and strobe trigger in us", "us", 5, 1000, 50, false, setFlashes);
    sys.cfg.addParam(FRAMERATE, "Camera frame rate in Hz", "Hz", 1, 30, 10, false, setTriggers);
    sys.cfg.addParam(TRIGWIDTH, "Width of the camera trigger pulse in us", "us", 30, 10000, 100, false, setFlashes);
    sys.cfg.addParam(LOWMAGCOLORFLASH, "Width of the low-mag white flash in us", "us", 1, 100000, 10, false, setFlashes);
    sys.cfg.addParam(LOWMAGREDFLASH, "Width of the low-mag far red flash in us", "us", 1, 100000, 10, false, setFlashes);
    sys.cfg.addParam(HIGHMAGCOLORFLASH, "Width of the high-mag white flash in us", "us", 1, 100000, 10, false, setFlashes);
    sys.cfg.addParam(HIGHMAGREDFLASH, "Width of the high-mag far red flash in us", "us", 1, 100000, 10, false, setFlashes);
    sys.cfg.addParam(FLASHTYPE, "0 = white strobes, 1 = far red strobes","", 0, 1, 0, false, setFlashes);
    sys.cfg.addParam(PROFILEMODE,"0 = upcast only, 1 = always on", "", 0, 1, 0);
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
    sys.cfg.addParam(MINDEPTH, "The minimum depth to allow powering on camera and recording", "mm", -2000, 10000, 500);
    sys.cfg.addParam(MAXDEPTH, "The maximum depth to allow powering on camera and recording", "mm", -2000, 1000000, 500000);
    sys.cfg.addParam(ECHORBR,"0 = don't print RBR data, 1 = print RBR data over ui ports", "", 0, 1, 1);
    sys.cfg.addParam(USERBRCLOCK,"0 = Use value of RTC, 1 = Sync RTC with time data from RBR CTD","",0,1,1);
    sys.cfg.addParam(CTDTYPE, "0 = RBR, 1 = SBE39, The type of CTD data to parse","",0,0,1, setCTDType);

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

    sys.loadScheduler();

    // Setup flashes triggers and polling
    setFlashes();
    setTriggers();
    setPolling();
    
}

void loop() {

    sys.update();
    sys.checkInput();
    sys.checkVoltage();
    sys.checkEnv();
    sys.checkEvents();
    sys.checkCameraPower(); 

    int logInt = sys.cfg.getInt(LOGINT);

    delay(logInt);
    Blink(10, 1);

}
