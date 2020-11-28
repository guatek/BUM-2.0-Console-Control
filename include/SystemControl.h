#ifndef _SYSTEMCONTROL

#define _SYSTEMCONTROL

#include <Arduino.h>
#include <RTCZero.h>
#include <RTCLib.h>
#include <WDTZero.h>
#include "Config.h"
#include "DeepSleep.h"
#include "SPIFlash.h"
#include "Sensors.h"
#include "Stats.h"
#include "SystemConfig.h"
#include "SystemTrigger.h"
#include "RBRInstrument.h"
#include "Utils.h"

#define CMD_CHAR '!'
#define PROMPT "DM > "
#define CMD_BUFFER_SIZE 128

#define STROBE_POWER 7
#define CAMERA_POWER 6

// Global Sensors
Sensors _sensors;

// Global RTCZero
RTCZero _zerortc;

//Global RTCLib
RTC_DS3231 _ds3231;

// Global watchdog timer with 8 second hardware timeout
WDTZero _watchdog;

// RBR instrument
RBRInstrument _rbr;

// Static polling function for instruments
bool pollingEnable = false;
void pollInstruments() {
    if (!pollingEnable)
        return;
    _rbr.readData(&RBRPORT);
}

class SystemControl
{
    private:
    float lastDepth;
    bool systemOkay;
    bool ds3231Okay;
    bool cameraOn;
    bool pendingPowerOff;
    bool lowVoltage;
    bool badEnv;
    char cmdBuffer[CMD_BUFFER_SIZE];
    bool rbrData;
    int state;
    unsigned long timestamp;
    unsigned long lastDepthCheck;
    unsigned long startupTimer;
    unsigned long lastPowerOnTime;
    unsigned long lastPowerOffTime;
    unsigned long pendingPowerOffTimer;
    unsigned long envTimer;
    unsigned long voltageTimer;
    MovingAverage<float> avgVoltage;
    MovingAverage<float> avgTemp;
    MovingAverage<float> avgHum;

    int checkCommand(char * input, const char * command, int n) {
        size_t maxLength = n;
        
        // Coerce to shortest string
        if (strlen(input) < maxLength)
            maxLength = strlen(input);
        if (strlen(command) < maxLength)
            maxLength = strlen(command);

        for (unsigned int i=0; i < maxLength; i++) {
            if (tolower(input[i]) < tolower(command[i]))
                return -1;
            if (tolower(input[i]) > tolower(command[i]))
                return 1;
        }

        // string match up to n chars
        return 0;
    }
    
    void readInput(Stream *in) {
      
        if (in != NULL && in->available() > 0) {
            char c = in->read();
            if (c == CMD_CHAR) {

                // Don't echo the command char
                //if (cfg.getInt("LOCALECHO"))
                //    in->write(c);
              
                // Print the prompt
                in->write(PROMPT);


                unsigned long startTimer = millis();
                int index = 0;
                while (startTimer <= millis() && millis() - startTimer < (unsigned int)(cfg.getInt("CMDTIMEOUT"))) {

                    // Break if we have exceed the buffer size
                    if (index >= CMD_BUFFER_SIZE)
                        break;

                    // Wait on user input
                    if (in->available()) {
                        // Read the next char and reset timer          
                        c = in->read();
                        startTimer = millis();
                    }
                    else {
                        continue;
                    }

                    // Exit command loop on repeat command char
                    if (c == CMD_CHAR) {
                        break;
                    }
                    
                    if (c == '\r') {
                        // Command ended try to 
                        if (index <= CMD_BUFFER_SIZE)
                            cmdBuffer[index++] = '\0';
                        else
                            cmdBuffer[CMD_BUFFER_SIZE-1] = '\0';
                        
                        // Parse Command menus
                        char * rest;
                        char * cmd = strtok_r(cmdBuffer,",",&rest);

                        // CFG (configuration commands)
                        if (cmd != NULL && checkCommand(cmd,CFG, 3) == 0) {
                            cfg.parseConfigCommand(rest, in);
                        }

                        // PORTPASS (pass through to other serial ports)
                        if (cmd != NULL && checkCommand(cmd,PORTPASS, 8) == 0) {
                            doPortPass(in, rest);
                        }

                        // SETTIME (set time from string)
                        if (cmd != NULL && checkCommand(cmd,SETTIME, 7) == 0) {
                            setTime(rest, in);
                        }

                        // WRITECONFIG (save the current config to EEPROM)
                        if (cmd != NULL && checkCommand(cmd,WRITECONFIG, 11) == 0) {
                            writeConfig();
                        }

                        // READCONFIG (read the current config to EEPROM)
                        if (cmd != NULL && checkCommand(cmd,READCONFIG, 10) == 0) {
                            readConfig();
                        }

                        if (cmd != NULL && checkCommand(cmd,CAMERAON,8) == 0) {
                            if (confirm(in, "Are you sure you want to power ON camera ? [y/N]: ", cfg.getInt(CMDTIMEOUT)))
                                turnOnCamera();
                        }

                        if (cmd != NULL && checkCommand(cmd,CAMERAOFF,9) == 0) {
                            if (confirm(in, "Are you sure you want to power OFF camera ? [y/N]: ", cfg.getInt(CMDTIMEOUT)))
                                turnOffCamera();
                        }

                        if (cmd != NULL && checkCommand(cmd,SHUTDOWNJETSON,14) == 0) {
                            if (confirm(in, "Are you sure you want to shutdown jetson ? [y/N]: ", cfg.getInt(CMDTIMEOUT)))
                                sendShutdown();
                        }

                        // Reset the buffer and print out the prompt
                        if (c == '\n')
                            in->write('\r');
                        else
                            in->write("\r\n");

                        in->write(PROMPT);

                        index = 0;                       
                        startTimer = millis();
                        continue;
                    }
                    
                    // Handle backspace
                    if (c == '\b') {
                        index -= 1;
                        if (index < 0)
                            index = 0;
                        if (cfg.getInt(LOCALECHO)) {
                           in->write("\b \b");
                        }
                    }
                    else {
                        cmdBuffer[index++] = c;
                        if (cfg.getInt(LOCALECHO))
                            in->write(c);
                    }
                }
            }
        }
    }

    void doPortPass(Stream * in, char * cmd) {
        char * rest;
        char * num = strtok_r(cmd,",",&rest);
        in->print("Passing through to hardware port ");
        in->println(num);
        in->println();
        if (num != NULL) {
            char portNum = *num;
            switch (portNum) {
                case '0':
                    portpass(in, &HWPORT0, cfg.getInt(LOCALECHO) == 1);
                    break;
                case '1':
                    portpass(in, &HWPORT1, cfg.getInt(LOCALECHO) == 1);
                    break;
                case '2':
                    portpass(in, &HWPORT2, cfg.getInt(LOCALECHO) == 1);
                    break;
                case '3':
                    portpass(in, &HWPORT3, cfg.getInt(LOCALECHO) == 1);
                    break;
            }
        }
    }

    void setTime(char * timeString, Stream * ui) {
        if (timeString != NULL) {
            // if we have ds3231 set that first
            DateTime dt(timeString);
            if (ds3231Okay) {
                _ds3231.adjust(dt.unixtime());
            }
            _zerortc.setEpoch(dt.unixtime());
        }
    }
                      
 
    public:

    SystemConfig cfg;
    int trigWidth;
    int lowMagStrobeDuration;
    int highMagStrobeDuration;
    int flashType;
    int frameRate;
  
    SystemControl() {
        systemOkay = false;
        rbrData = false;
        state = 0;
        timestamp = 0;
        ds3231Okay = false;
        pendingPowerOff = false;
        cameraOn = false;
        lowVoltage = false;
        badEnv = false;
    }

    bool begin() {

        //Turn off strobe and camera power
        pinMode(CAMERA_POWER, OUTPUT);
        pinMode(STROBE_POWER, OUTPUT);

        digitalWrite(CAMERA_POWER, LOW);
        digitalWrite(STROBE_POWER, HIGH);

        // Start RTC
        _zerortc.begin();

        // Start DS3231
        ds3231Okay = true;
        if (!_ds3231.begin()) {
            DEBUGPORT.println("Could not init DS3231, time will be lost on power cycle.");
            ds3231Okay = false;
        }
        else {
            // sync rtczero to DS3231
            _zerortc.setEpoch(_ds3231.now().unixtime());
        }

        // set the startup timer
        startupTimer = _zerortc.getEpoch();
        lastPowerOffTime = _zerortc.getEpoch();
        lastPowerOnTime = _zerortc.getEpoch();
        lastDepthCheck = _zerortc.getEpoch();
        voltageTimer = _zerortc.getEpoch();
        envTimer = _zerortc.getEpoch();
        lastDepth = 0.0;

        systemOkay = true;
        if (_flash.initialize()) {
            DEBUGPORT.println("Flash Init OK.");
        }
            
        else {
            DEBUGPORT.print("Init FAIL, expectedDeviceID(0x");
            DEBUGPORT.print(_expectedDeviceID, HEX);
            DEBUGPORT.print(") mismatched the read value: 0x");
            DEBUGPORT.println(_flash.readDeviceId(), HEX);
        }

        // Start sensors
        _sensors.begin();

        return true;

    }

    void configWatchdog() {
        // enable hardware watchdog if requested
        if (cfg.getInt(WATCHDOG) > 0) {
            _watchdog.setup(WDT_HARDCYCLE8S);
        }
    }

    bool turnOnCamera() {
        if (_zerortc.getEpoch() - lastPowerOffTime > (unsigned int)cfg.getInt(CAMGUARD) && !cameraOn) {
            DEBUGPORT.println("Turning ON camera power...");
            cameraOn = true;
            digitalWrite(CAMERA_POWER, HIGH);
            digitalWrite(STROBE_POWER, LOW);
            lastPowerOnTime = _zerortc.getEpoch();
            return true;
        }
        else {
            return false;
        }
    }

    bool turnOffCamera() {
        if (_zerortc.getEpoch() - lastPowerOnTime > (unsigned int)cfg.getInt(CAMGUARD) && cameraOn) {
            DEBUGPORT.println("Turning OFF camera power...");
            cameraOn = false;
            digitalWrite(CAMERA_POWER, LOW);
            digitalWrite(STROBE_POWER, HIGH);
            lastPowerOffTime = _zerortc.getEpoch();
            return true;
        }
        else {
            return false;
        }
    }

    bool update() {

        // Run updates and check for new data
        _sensors.update();

        // Build log string and send to UIs
        char output[256];

        float c = -1.0;
        float t = -1.0;
        float d = -1.0;

        uint32_t unixtime; 

        char timeString[64];
        
        sprintf(timeString,"%s","YYYY-MM-DD hh:mm:ss");

        if (ds3231Okay) {
            DateTime now = _ds3231.now();
            unixtime = now.unixtime();
            now.toString(timeString);
        }
        else {
            unixtime = _zerortc.getEpoch();
            sprintf(timeString, "%04d-%02d-%02d %02d:%02d:%02d", 
                _zerortc.getYear(),
                _zerortc.getMonth(),
                _zerortc.getDay(),
                _zerortc.getHours(),
                _zerortc.getMinutes(),
                _zerortc.getSeconds()
            );
        }

        if (_rbr.haveNewData()) {
            c = _rbr.conductivity();
            t = _rbr.temperature();
            d = _rbr.pressure();
            _rbr.invalidateData();

            // Check state (float up, down ratchet)
            if (unixtime - lastDepthCheck > (unsigned int)cfg.getInt(DEPTHCHECKINTERVAL)) {
                float delta_depth = d - lastDepth;
                if ((state == -1 || state == 0) && delta_depth > ((float)cfg.getInt(DEPTHTHRESHOLD))/1000) {
                    state = 1; // ascent to descent
                }
                if ((state == 1 || state == 0)  && delta_depth < ((float)cfg.getInt(DEPTHTHRESHOLD))/1000) {
                    state = -1; // descent to ascent
                }
                lastDepthCheck = unixtime;
                lastDepth = d;
            }

            // If profile mode is 0 and camera is on, shut it down
            if (state == 1 && cfg.getInt(PROFILEMODE) == 0) {
                if (cameraOn && !pendingPowerOff) {
                    sendShutdown();
                }
            }

            // If profile mode is 0 and camera is off, turn it on
            if (state == -1 && cfg.getInt(PROFILEMODE) == 0) {
                
                if (!cameraOn) {
                    turnOnCamera();
                }
            }
        }


        // The system log string, note this requires enabling printf_float build
        // option work show any output for floating point values
        sprintf(output, "$DM,%s.%03u,%0.3f,%0.3f,%0.2f,%0.2f,%0.2f,%0.2f,%0.3f,%0.3f,%0.3f,%d,%d,%d,%d,%d,%d",

            timeString,
            ((unsigned int) millis()) % 1000,
            _sensors.temperature, // In C
            _sensors.pressure / 1000, // in kPa
            _sensors.humidity, // in %
            _sensors.voltage[0] / 1000, // In Volts
            _sensors.power[0] / 1000, // in W
            _sensors.power[1] / 1000, // in W
            c,
            t,
            d,
            state,
            cameraOn,
            flashType,
            lowMagStrobeDuration,
            highMagStrobeDuration,
            frameRate
            
        );

        // Send output
        printAllPorts(output);

        return true;
    }

    void writeConfig() {
        if (systemOkay) {
            cfg.writeConfig();
        }
    }

    void readConfig() {
        if (systemOkay)
            cfg.readConfig();
    }

    void checkInput() {
        if (DEBUGPORT.available() > 0) {
            readInput(&DEBUGPORT);
        }
        if (UI1.available() > 0) {
            _rbr.disableEcho();
            readInput(&UI1);
        }
        if (UI2.available() > 0) {
            _rbr.disableEcho();
            readInput(&UI2);
        }
        _rbr.enableEcho();
    }

    void printAllPorts(const char output[]) {
        UI1.println(output);
        UI2.println(output);
        DEBUGPORT.println(output);
    }

    void checkCameraPower() {

        // Check for power off flag
        if (pendingPowerOff && ((_sensors.power[0] < 1000) || (_zerortc.getEpoch() - pendingPowerOffTimer > (unsigned int)cfg.getInt(MAXSHUTDOWNTIME)))) {
            turnOffCamera();
            pendingPowerOff = false;
            return;
        }

        if (lowVoltage || badEnv) {
            return;
        }

        // If camera is set to be always on, and there are no issues, power it up now
        if (cfg.getInt(PROFILEMODE) == (unsigned int)1) {
            turnOnCamera();
        }
    }

    void checkEnv() {
        if (_zerortc.getEpoch() - startupTimer <= (unsigned int)cfg.getInt(STARTUPTIME))
            return;


        // Update moving average of temperature
        float latestTemp = avgTemp.update(_sensors.temperature);
        float latestHum = avgHum.update(_sensors.humidity);
        //float latestVoltage = _sensors.voltage[0];

        // Make sure this check happens AFTER updating the average measurement, otherwise
        // the average will not be calculated properly
        if (_zerortc.getEpoch() - envTimer <= (unsigned int)cfg.getInt(CHECKINTERVAL))
            return;

        // Reset check timer
        envTimer = _zerortc.getEpoch();

        if (latestTemp > cfg.getInt(TEMPLIMIT)) {
            char output[64];
            sprintf(output,"Temperature %0.2f C exceeds limit of %0.2f C", latestTemp, (float)cfg.getInt(TEMPLIMIT));
            printAllPorts(output);
            badEnv = true;
            if (cameraOn) {
                printAllPorts("Shuting down camera...");
                sendShutdown();
            }
        }

        if (latestHum > cfg.getInt(HUMLIMIT)) {
            char output[64];
            sprintf(output,"Humidity %0.2f %% exceeds limit of %0.2f %%", latestHum, (float)cfg.getInt(HUMLIMIT));
            printAllPorts(output);
            badEnv = true;
            if (cameraOn) {
                printAllPorts("Shuting down camera...");
                sendShutdown();
            }
        }

        badEnv = false;
        
    }

    void checkVoltage() {

        if (_zerortc.getEpoch() - startupTimer <= (unsigned int)cfg.getInt(STARTUPTIME))
            return;

        // Update moving average of voltage
        float latestVoltage = avgVoltage.update(_sensors.voltage[0]);
        //float latestVoltage = _sensors.voltage[0];

        // Make sure this check happens AFTER updating the average measurement, otherwise
        // the average will not be calculated properly
        if (_zerortc.getEpoch() - voltageTimer <= (unsigned int)cfg.getInt(CHECKINTERVAL))
            return;
        
        // Reset check timer
        voltageTimer = _zerortc.getEpoch();

        if (latestVoltage < 6000.0) {
            // likely on USB power, note voltage is in mV
            return;
        }

        // If battery voltage is too low, notify and sleep
        // If the camera is running at this point, shut it down first
        if (latestVoltage < cfg.getInt(LOWVOLTAGE)) {
            char output[256];
            sprintf(output,"Voltage %f below threshold %d", latestVoltage, cfg.getInt(LOWVOLTAGE));
            printAllPorts(output);
            if (cameraOn) {
                sendShutdown();
            }
            if (cfg.getInt(STANDBY) == 1 && !cameraOn) {

                printAllPorts("Going to sleep...");
                _zerortc.setAlarmTime(0, 0, 0);
                if (cfg.getInt(CHECKHOURLY) == 1) {
                    _zerortc.enableAlarm(RTCZero::MATCH_MMSS);
                }
                else {
                    _zerortc.enableAlarm(RTCZero::MATCH_SS);
                }
                if (cfg.getInt(STANDBY) == 1) {
                    DEBUGPORT.println("Would go into stanby here but currently disabled.");
                    //_zerortc.standbyMode();
                }
            }
        }
    }

    void sendShutdown() {
        if (cameraOn) {
            DEBUGPORT.println("Sending to Jetson: sudo shutdown -h now");
            JETSONPORT.println("sudo shutdown -h now\n");
            pendingPowerOff = true;
            pendingPowerOffTimer = _zerortc.getEpoch();
        }
        else {
            DEBUGPORT.println("Camera not powered on, not sending shutdown command");
        }
    }

    void configureFlashDurations() {
        // Set global delays for ISRs
        trigWidth = cfg.getInt(TRIGWIDTH);
        flashType = cfg.getInt(FLASHTYPE);
        if (flashType == 0) {
            digitalWrite(FLASH_TYPE_PIN,HIGH);
            lowMagStrobeDuration = cfg.getInt(LOWMAGCOLORFLASH);
            highMagStrobeDuration = cfg.getInt(HIGHMAGCOLORFLASH);
        }
        else {
            digitalWrite(FLASH_TYPE_PIN,LOW);
            lowMagStrobeDuration = cfg.getInt(LOWMAGREDFLASH);
            highMagStrobeDuration = cfg.getInt(HIGHMAGREDFLASH);
        }
    }

    void setTriggers() {
        frameRate = cfg.getInt(FRAMERATE); 
        configTriggers(cfg.getInt(FRAMERATE));
    }

    void setPolling() {
        pollingEnable = true;
        configPolling(cfg.getInt(POLLFREQ), pollInstruments);
    }
        
};

#endif
