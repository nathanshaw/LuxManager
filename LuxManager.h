#ifndef __LUX_H__
#define __LUX_H__

#include <Wire.h>
// #include "SparkFun_VEML6030_Ambient_Light_Sensor.h"
#include <SparkFun_VEML6030_Ambient_Light_Sensor.h>
// #include "Adafruit_VEML7700.h"
#include <Adafruit_VEML7700.h>
#include <NeopixelManager.h>
#include <ValueTrackerDouble.h>

#define TCAADDR 0x70
//
#define V6030_ADDR1    0x48
#define V6030_ADDR2    0x10

#ifndef MAX_LUX_SENSORS
#define MAX_LUX_SENSORS 2
#endif

#ifndef TAKE_HIGHEST_LUX
#define TAKE_HIGHEST_LUX 1
#endif

#ifndef TAKE_AVERAGE_LUX
#define TAKE_AVERAGE_LUX 0
#endif

#ifndef LUX_ADJUSTS_BS
#define LUX_ADJUSTS_BS 0
#endif

// THESE DO NOT DENOTE THAT IS HOW THE MANAGER WILL WORK
// IT IS ONLY THE OPTINOS USED IN THE LOGIC
#ifndef LUX_ADJUSTS_MIN_MAX
#define LUX_ADJUSTS_MIN_MAX 1
#endif

#ifndef LUX_ADJUSTS_BS_AND_MIN_MAX
#define LUX_ADJUSTS_BS_AND_MIN_MAX 2
#endif


class LuxManager {
    public:
        LuxManager(long minrt, long maxrt, uint8_t mapping);
        void setBrightnessScalerMinMax(double min, double max);
        void changeMapping(uint8_t mapping);

        void linkNeoGroup(NeoGroup * n);
        void add6030Sensors(float gain, int _int);
        void add7700Sensor(String _name);
        void addSensorTcaIdx(String _name, int tca);

        void setLuxThresholds(float _night, float _low, float _mid, float _high, float _extreme);

        double getLux() {
            return global_lux;
        };
        double getGlobalLux() {return global_lux;};
        void startTCA7700Sensors(byte gain, byte integration);
        void start7700Sensor(byte gain, byte integration);

        // to help multiple lux managers coordinate
        double forceLuxReading();
        void calibrate(long len, bool first_time);

        bool update();
        void resetMinMax();

        double getAvgLux();
        void   resetAvgLux();

        String getName(int i) {return names[i];};

        // brightness and brightness scalers
        double getBrightnessScaler();
        double getBrightnessScalerAvg();
        void resetBrightnessScalerAvg();

        bool getExtremeLux() {return extreme_lux;};
        double getScaledLux() {return global_lux_tracker.getScaled();};

        bool getSensorActive(int i){return sensor_active[i];};

        void print();

        void setPrintBrightnessScaler(bool s) {p_brightness_scaler = s;};
        void setPrintLuxReadings(bool s) {p_lux_readings = s;};
        void setPrintGeneralDebug(bool s) {p_lux = s;};
        bool getActive(){return sensors_active;};

        double lux[MAX_LUX_SENSORS];
        ValueTrackerDouble lux_tracker[MAX_LUX_SENSORS] = {ValueTrackerDouble("lux1", &lux[0], 1.0), 
            ValueTrackerDouble("lux2", &lux[1], 1.0)};

        double global_lux = 400.0;
        ValueTrackerDouble global_lux_tracker = ValueTrackerDouble("global_lux", &global_lux, 1.0);

        double brightness_scaler = 0.0;
        ValueTrackerDouble bs_tracker = ValueTrackerDouble("LuxM_bs", &brightness_scaler, 0.5);

    private:
        ////////////// Linked Neopixels ///////////////
        NeoGroup*  neos[6];
        ////////////// Printing ///////////////
        bool p_brightness_scaler = false;
        bool p_lux_readings = false;
        bool p_lux = false;

        ///////////////////////////////////////////////
        bool sensors_active = false;
        bool sensor_active[MAX_LUX_SENSORS];
        
        Adafruit_VEML7700       sensors_7700[MAX_LUX_SENSORS];

        SparkFun_Ambient_Light  sensors_6030[MAX_LUX_SENSORS] = {
            SparkFun_Ambient_Light(V6030_ADDR1), SparkFun_Ambient_Light(V6030_ADDR2)};

        uint8_t mode = TAKE_HIGHEST_LUX;
        uint8_t num_sensors = 0;
        uint8_t num_6030_sensors = 0;
        uint8_t num_7700_sensors = 0;

        //////////////// Lux Thresholds for Mapping ////////////////
        float night_thresh;
        float low_thresh;
        float mid_thresh;
        float high_thresh;
        float extreme_thresh;

        int tca_addr[MAX_LUX_SENSORS];

        // how long to shutdown depends on the integration time, this is set to 180% of the integration time
        uint16_t shdn_len = 45;

        uint8_t num_neo_groups = 0;
        String names[MAX_LUX_SENSORS];

        double past_readings[MAX_LUX_SENSORS][10];

        void readLux();

        unsigned long min_reading_time = 1000*60*3;
        unsigned long max_reading_time = 1000*60*3;

        elapsedMillis last_reading;
        long polling_rate;

        // for brightness
        double bs_min = 0.125;
        double bs_max = 3.0;

        double calculateBrightnessScaler();

        double read();
        bool extreme_lux;
        uint8_t lux_mapping_schema = LUX_ADJUSTS_BS;
        bool tca_present = false;
};

LuxManager::LuxManager(long minrt, long maxrt, uint8_t mapping){
    min_reading_time = minrt;
    max_reading_time = maxrt;
    lux_mapping_schema = mapping;
    Serial.print("LuxManager created with a min reading time of: ");
    Serial.println(min_reading_time);
    Serial.print("LuxManager created with a max reading time of: ");
    Serial.println(max_reading_time);
    Serial.print("LuxManager created with a mapping schema of  : ");
    if (lux_mapping_schema == LUX_ADJUSTS_BS) {
        Serial.println("LUX_ADJUSTS_BS");
    } else if (lux_mapping_schema == LUX_ADJUSTS_MIN_MAX) {
        Serial.println("LUX_ADJUSTS_MIN_MAX");
    } else {
        Serial.println(" WARNING THIS MAPPING DOES NOT EXIST");

    }
}

void LuxManager::calibrate(long len, bool first_time) {
    printDivide();
    Serial.println("Now starting the LuxManager.calibrate() function");
    for (int i = 0; i < num_neo_groups; i++){
        Serial.print("Turning NeoGroup ");
        Serial.print(neos[i]->getName());
        neos[i]->colorWipe(0,0,0, 0.0);
        Serial.println(" off");
    }
    Serial.println("Turned NeoPixels off");
    printMinorDivide();
    readLux();
    printMinorDivide();
    Serial.println(F("Finished initial reading, now resetting min and max to the current reading"));
    resetMinMax();
    printMinorDivide();
    Serial.println(F("Now reading the lux sensors 10x"));
    printMinorDivide();
    for (int i = 0; i < 10; i++) {
        Serial.print("Reading number: ");
        Serial.println(i);
        readLux();
        delay(len/10);
    }
    print();
    printMinorDivide();
    printMinorDivide();
    Serial.println("LUX_MANAGER calibration is now finished");
    printDivide();
}

void LuxManager::setLuxThresholds(float _night, float _low, float _mid, float _high, float _extreme) {
    night_thresh = _night;
    low_thresh = _low;
    mid_thresh = _mid;
    high_thresh = _high;
    extreme_thresh = _extreme;
}

void LuxManager::setBrightnessScalerMinMax(double min, double max) {
    bs_min = min;
    bs_max = max;
    Serial.print("LuxManager BS min/max updated to: ");
    Serial.print(bs_min);
    Serial.print(" / ");
    Serial.println(bs_max);
}

void LuxManager::changeMapping(uint8_t mapping) {
    if (mapping != lux_mapping_schema){
        lux_mapping_schema = mapping;
    if (lux_mapping_schema == LUX_ADJUSTS_BS) {
        Serial.println("LUX_ADJUSTS_BS");
    } else if (lux_mapping_schema == LUX_ADJUSTS_MIN_MAX) {
        Serial.println("LUX_ADJUSTS_MIN_MAX");
    } else {
        Serial.println(" WARNING THIS MAPPING DOES NOT EXIST");

    }
    }
};


//////////////////////////// lux and stuff /////////////////////////
/*
// for adding a sensor which is not based on tca
void LuxManager::addSensorI2CAddr(String _name, int addr){
names[num_sensors] = _name;
num_sensors] = Adafruit_VEML7700();
sensor_active[num_sensors] = false;// not active until startSensors() is called
num_sensors++;
}
*/
void LuxManager::addSensorTcaIdx(String _name, int tca){
    // will return true if sensor is found and false if it is not
    names[num_sensors] = _name;
    tca_addr[num_sensors] = tca;
    // No need to initalise as that is already done when instanciating the class?
    // sensors_7700[num_sensors] = Adafruit_VEML7700::Adafruit_VEML7700();
    sensor_active[num_sensors] = false;// not active until startSensors() is called
    num_sensors++;
    num_7700_sensors++;
}

void LuxManager::add7700Sensor(String _name) {
    names[num_sensors] = _name;
    // No need to initalise as that is already done when instanciating the class?
    // sensors_7700[num_sensors] = Adafruit_VEML7700::Adafruit_VEML7700();
    sensor_active[num_sensors] = false;// not active until startSensors() is called
    num_sensors++;
    num_7700_sensors++;
}

void LuxManager::add6030Sensors(float gain, int _int) {
    Wire.begin();
    delay(100);
    Serial.println("Adding VEML6030 Lux Sensors");
    names[num_sensors] = "Front";
    // sensors_6030[num_sensors] = SparkFun_Ambient_Light(V6030_ADDR1);
    if (sensors_6030[num_sensors].begin()){
        Serial.println("added first VEML6030 sensor");
        sensors_6030[num_sensors].setGain(gain);
        sensors_6030[num_sensors].setIntegTime(_int);
        Serial.println("configured first VEML6030 sensor");
        sensor_active[num_sensors] = true;
        sensors_active = true;
        num_sensors++;
        num_6030_sensors++;
    }
    else {
        Serial.println("WARNING - TROUBLE STARTING VEML6030 SENSOR");
    }

    names[num_sensors] = "Rear";
    // sensors_6030[num_sensors] = SparkFun_Ambient_Light(V6030_ADDR2);
    if (sensors_6030[num_sensors].begin()){
        Serial.println("added second VEML6030 sensor");
        sensors_6030[num_sensors].setGain(gain);
        sensors_6030[num_sensors].setIntegTime(_int);
        Serial.println("configured second VEML6030 sensor");
        sensor_active[num_sensors] = true;
        sensors_active = true;
        num_sensors++;
        num_6030_sensors++;
    }
    else {
        Serial.println("WARNING - TROUBLE STARTING VEML6030 SENSOR");
    }
    Serial.print(num_sensors);
    Serial.println(" sensors were added correctly");
}

// void LuxManager::findSensor() {
// search for veml7700 sensor
// search for veml7700 sensor via TCA
// search for veml6030 sensor

// determine what side of the PCB the sensor is on (according to neogroups)
// link the lux sensors to the correct neogroups if that is needed/wanted
// }

void LuxManager::linkNeoGroup(NeoGroup * n){
    neos[num_neo_groups] = n;
    num_neo_groups++;
}

double LuxManager::getAvgLux() {
    return global_lux_tracker.getAvg();
}

void LuxManager::resetAvgLux() {
    global_lux_tracker.getAvg(true);
}

void tcaselect(uint8_t i) {
    if (i > 7) return;
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}

void LuxManager::start7700Sensor(byte gain, byte integration) {
    Wire.begin();
    delay(500);
    // TODO - dont assume the 7700 sensor will be the only sensor
    Serial.print("\nattempting to start lux sensor ");
    Serial.println(names[0]);
    if (!sensors_7700[0].begin()) {
        Serial.print("ERROR ---- VEML "); Serial.print(names[0]); Serial.println(" not found");
        for (int i = 0; i < num_neo_groups; i++) {
            neos[i]->colorWipe(255, 0, 0, 1.0);
        }
        unsigned long then = millis();
        while (millis() < then + 5000) {
            Serial.print(".");
            delay(100);
        }
        Serial.println();
    }
    else {
        Serial.print("VEML "); Serial.print(names[0]); Serial.println(" found");
        sensor_active[0] = true;
        sensors_active = true;
        sensors_7700[0].setGain(gain); // talk about gain and integration time in the thesis
        sensors_7700[0].setIntegrationTime(integration);// 800ms was default
    }
}

void LuxManager::startTCA7700Sensors(byte gain, byte integration) {
    Wire.begin();
    delay(500);
    for (int i = 0; i < num_sensors; i++){
        Serial.print("\nattempting to start lux sensor ");
        Serial.println(names[i]);
        if (tca_addr[i] > -1) {
            tcaselect(tca_addr[i]);
        }
        if (!sensors_7700[i].begin()) {
            Serial.print("ERROR ---- VEML "); Serial.print(names[i]); Serial.println(" not found");
            for (int n = 0; n < num_neo_groups; n++) {
                neos[n]->colorWipe(255, 0, 0, 1.0);
            }
            unsigned long then = millis();
            while (millis() < then + 5000) {
                Serial.print(".");
                delay(100);
            }
            Serial.println();
        }
        else {
            Serial.print("VEML "); Serial.print(names[i]); Serial.println(" found");
            sensor_active[i] = true;
            sensors_active = true;
            sensors_7700[i].setGain(gain); // talk about gain and integration time in the thesis
            sensors_7700[i].setIntegrationTime(integration);// 800ms was default
        }
    }
}

void LuxManager::readLux() {
    /* This function handles reading each individual lux sensor and then
     * determining what the global lux is. 
     */
    bool onwards = false;
    for (int i = 0; i < num_sensors; i++) {
        if (sensor_active[i] == true){
            onwards = true;
        }
    }
    if (onwards == false) {
        dprintln(p_lux, "exiting from readLux(), no active sensors");
        return;
    }
    for (int i = 0; i < num_7700_sensors; i++){
        // read each sensor
        if (tca_present == true && tca_addr[i] > -1) {
            Serial.println("Entering into tcaselect loop");
            tcaselect(tca_addr[i]);
            Serial.println("Exiting tcaselect loop");
        }
            double _t = sensors_7700[i].readLux();
            if (_t > 1000000) {
                _t = sensors_7700[i].readLux();
                if (_t  < 1000000) {
                    dprint(p_lux, "lux ");
                    dprint(p_lux, i);
                    dprint(p_lux, " updated from "); 
                    dprint(p_lux, lux[i]);
                    dprint(p_lux, " to ");
                    lux[i] = _t;
                    lux_tracker[i].update();
                    dprintln(p_lux, lux[i]);
                }
            } else {
                dprint(p_lux, "lux ");
                dprint(p_lux, i);
                dprint(p_lux, " updated from "); 
                dprint(p_lux, lux[i]);
                dprint(p_lux, " to ");
                lux[i] = _t;
                lux_tracker[i].update();
                dprintln(p_lux, lux[i]);
            }
        }

        for (int i = 0; i < num_6030_sensors; i++){
            double _t = sensors_6030[i].readLight();
            if (_t > 1000000) {
                _t = sensors_6030[i].readLight();
                if (_t  < 1000000) {
                    dprint(p_lux, "lux ");
                    dprint(p_lux, i);
                    dprint(p_lux, " updated from "); 
                    dprint(p_lux, lux[i], 4);
                    dprint(p_lux, " to ");
                    lux[i] = _t;
                    lux_tracker[i].update();
                    dprintln(p_lux, lux[i], 4);
                }
            } else {
                dprint(p_lux, "lux ");
                dprint(p_lux, i);
                dprint(p_lux, " updated from "); 
                dprint(p_lux, lux[i], 4);
                dprint(p_lux, " to ");
                lux[i] = _t;
                lux_tracker[i].update();
                dprintln(p_lux, lux[i], 4);
            }
        }

        for (int i = 0; i < num_sensors; i++) {
            dprint(p_lux_readings, "Lux for sensor #");
            dprint(p_lux_readings, i);
            dprint(p_lux_readings, ": ");
            dprintln(p_lux_readings, lux[i]);
        }

        if (mode == TAKE_HIGHEST_LUX) {
            double highest = 0.0;
            for (int i = 0; i < num_sensors; i++) {
                if (highest < lux[i]) {
                    highest = lux[i];
                    dprint(p_lux_readings, "highest changed to: ");
                    dprintln(p_lux_readings, highest);
                }
            }
            global_lux = highest;
            global_lux_tracker.update();
            dprint(p_lux_readings, "Taking highest lux: ");
            dprintln(p_lux_readings, global_lux);

        } else if (mode == TAKE_AVERAGE_LUX) {
            double average = 0.0;
            for (int i = 0; i < num_sensors; i++) {
                average = average + lux[i];
            }
            average = average / num_sensors;
            global_lux = average;
            global_lux_tracker.update();
            dprint(p_lux_readings, "Taking average lux: ");
            dprintln(p_lux_readings, global_lux);
        } else {
            Serial.println("ERROR, there is not a valid mode selected for the LuxManager");
        }
        // now trigger an update to the brightness scaler
        brightness_scaler = calculateBrightnessScaler();
        bs_tracker.update();

        // update all linked neogroups with the new brighness scaler
        for (int i = 0; i < num_neo_groups; i++){
            if (lux_mapping_schema == LUX_ADJUSTS_BS || lux_mapping_schema == LUX_ADJUSTS_BS_AND_MIN_MAX) {
                if (neos[i]->getLuxBS() != brightness_scaler) {
                    dprint(p_brightness_scaler, "changing the neopixel brightness scaler from/to :\t");
                    dprint(p_brightness_scaler, neos[i]->getLuxBS());
                    dprint(p_brightness_scaler, "\t");
                    dprintln(p_brightness_scaler, brightness_scaler);

                    neos[i]->setLuxBS(brightness_scaler);
                }
                else {
                    dprintln(p_brightness_scaler, "not updating linked NeoGroup lux_bs as it has not changed based off the new readings");

                }
            } 
            if (lux_mapping_schema == LUX_ADJUSTS_MIN_MAX || lux_mapping_schema == LUX_ADJUSTS_BS_AND_MIN_MAX) {
                    dprintln(p_brightness_scaler, "WARNING LUX_ADJUSTS_MIN_MAX not implemented yet");
                }
        }
        if (p_brightness_scaler == 0) {
            dprint(p_lux_readings, "\tbs: "); 
            dprintln(p_lux_readings, brightness_scaler);
        };
        last_reading = 0;
}

double LuxManager::calculateBrightnessScaler() {
    // todo need to make this function better... linear mapping does not really work, need to map li
    double bs;
    // conduct brightness scaling depending on if the reading is above or below the mid thresh
    // is the unconstrained lux above the extreme_lux_thresh?
    dprintDivide(p_brightness_scaler, "------------------------------------------");
    dprint(p_brightness_scaler, names[0]);

    if (global_lux >= extreme_thresh) {
        bs = 0.0;
        dprintln(p_brightness_scaler, " is above extreme thresh, scaler set to 0.0 due to extreme lux");
        if (extreme_lux == false) {
            extreme_lux = true;
        }
    } 
    else if (global_lux >= high_thresh) {
        bs = bs_max;
        dprintln(p_brightness_scaler, " is between the high and extreme tresh, setting brightness scaler to bs_max");
        if (extreme_lux == true) {
            extreme_lux = false;
        }
    }
    else if (global_lux >= mid_thresh) {
        // bs is scaled to between 1.0 and bs_max
        bs = mapf(global_lux, low_thresh, high_thresh, 1.0, bs_max);
        dprintln(p_brightness_scaler, " is between the mid and high thresh, setting brightness scaler to between 1.0 and bs_max");
        if (extreme_lux == true) {
            extreme_lux = false;
        }
    }
    else if (global_lux >= low_thresh)  {
        // bs is scaled to between bs_min and 1.0
        bs = mapf(global_lux, low_thresh, mid_thresh, bs_min, 1.0);
        dprintln(p_brightness_scaler, " is between low and mid thresh, setting brightness scaler to between bs_min and 1.0");
        if (extreme_lux == true) {
            extreme_lux = false;
        }
    } 
    else if (global_lux >= night_thresh) {
        dprintln(p_brightness_scaler, " is between night and low thresh, setting brightness scaler to bs_min");
        bs = bs_min;
        if (extreme_lux == true) {
            extreme_lux = false;
        }
    }
    else {
        bs = 0.0;
        dprintln(p_brightness_scaler, " is lower than the night_thresh, setting brightness scaler to 0.0");
        if (extreme_lux == false) {
            extreme_lux = true;
        }
    }
    dprint(p_brightness_scaler, "global_lux of ");
    dprint(p_brightness_scaler, global_lux);
    dprint(p_brightness_scaler, " has resulted in a brightness_scaler of ");
    dprintln(p_brightness_scaler, bs);
    dprintDivide(p_brightness_scaler,"----------------------------------------");
    return bs;
}

double LuxManager::getBrightnessScaler() {
    return brightness_scaler;
}

double LuxManager::getBrightnessScalerAvg() {
    return bs_tracker.getAvg();
}

void LuxManager::resetBrightnessScalerAvg() {
    bs_tracker.getAvg(true);
}

void LuxManager::resetMinMax() {
    for (int i = 0; i < num_sensors; i++) {
        lux_tracker[i].resetMinMax();
    }
}

double LuxManager::forceLuxReading() {
    readLux();
    return global_lux;
}


bool LuxManager::update() {
    uint8_t took_reading = 0;
    // for each sensor
    for (int i = 0; i < num_sensors; i++) {
        // and for each linked neopixel group
        // if the LEDs are off 
        uint8_t ctr = 0;
        for (int n = 0; n < num_neo_groups; n++) {
            if (neos[n]->getLedsOn() == false){
                ctr++;
            }
        }
        // if all of the linked neogroups are off
        if (ctr == num_neo_groups){ 
            // dprint(p_lux, "LEDs have been off for ");
            // dprint(p_lux, neos[n]->getOffLen());
            // dprintln(p_lux, "ms");
            // and have been off for more than the shdn_len

            // check to see if they all have shutdown len which is long enough
            // if they have not then nothing happens 
            ctr = 0;
            for (int n = 0; n < num_neo_groups; n++) {
                if (neos[n]->getOffLen() >= shdn_len){
                    ctr++;
                }
            }
            if (ctr == num_neo_groups) {
                // if currently in extreme lux shutdown then poll 20x faster
                // dprint(p_lux, "Neopixels have been off for long enough to take a normal lux reading: ");
                // dprint(p_lux, last_reading);
                // dprint(p_lux, " / ");
                // dprintln(p_lux, min_reading_time);
                if (extreme_lux && last_reading > min_reading_time * 0.05) {
                    dprintln(p_lux,"------------------------------------------");
                    dprint(p_lux, "QUICK UPDATE due to extreme lux reading");
                    readLux();
                    took_reading++;
                    ctr = 0;
                    for (int n = 0; n < num_neo_groups; n++) {
                        if (neos[n]->isInShutdown() == false){
                            ctr++;
                        }
                    }
                }
                else if (last_reading > min_reading_time) {
                    dprintln(p_lux,"------------------------------------------");
                    dprintln(p_lux, "readLux() is being called by luxManager update()");
                    readLux();
                    took_reading++;

                }
            }
        }
        // if the LEDs are on and it has been longer than the max reading time
        else if (last_reading > max_reading_time) { // the LEDs will be off when this logic comes up
            // shdn len has to be longer to ensure the lux sensors get a good reading
            dprintln(p_lux,"------------------------------------------");
            dprint(p_lux, "last lux reading is greater than last reading time of ");
            dprint(p_lux, max_reading_time);
            dprint(p_lux, " and ");
            ctr = 0;
            for (int n = 0; n < num_neo_groups; n++) {
                if (neos[n]->isInShutdown() == false){
                    ctr++;
                }
            }
            dprint(p_lux, ctr);
            dprint(p_lux, " LEDs are not in shutdown");
            if (ctr == num_neo_groups) {
                dprintln(p_lux, "neos not in shutdown, putting them in shutdown now ");
                for (int n = 0; n < num_neo_groups; n++) {
                    dprint(p_lux, n);
                    dprintln(p_lux, " for a forced lux reading\n-----------------------------------------------");
                    neos[n]->shutdown(shdn_len*4);// shutdown the neogroup for more than enough time to conduct a lux reading
                    /// once a lux reading is collected, the shutdown will be reset
                }
            }
            dprintln(p_lux);
        }
    }
    if (took_reading > 0) {
        for (int n = 0; n < num_neo_groups; n++) {
            neos[n]->powerOn();
            dprint(p_lux, "Sending Power On Message to neogroup ");
            dprintln(p_lux, neos[n]->getName());
        }
    }
    return took_reading;
}

void LuxManager::print() {
    Serial.println("----------- Printing LuxManager Values ---------------");
    Serial.println("global lux");
    global_lux_tracker.print();
    for (int i = 0; i < num_sensors; i++){
        Serial.print("\n sensor # ");
        Serial.println(i);
        lux_tracker[i].print();
    }
    Serial.print("brightness_scaler :\t");
    Serial.println(brightness_scaler);
    Serial.println();
}

#endif // __LUX_H__
