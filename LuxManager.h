#ifndef __LUX_H__
#define __LUX_H__

#include "Adafruit_VEML7700.h"
#include <Wire.h>
#include "../Configuration.h"
#include "../NeopixelManager/NeopixelManager.h"
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"

// TODO add code so if the TCA is not available things are cool... also add firmware #define to control this
#define TCAADDR 0x70
//
#define V6030_ADDR1    0x48
#define V6030_ADDR2    0x10

#ifndef MAX_LUX_SENSORS
#define MAX_LUX_SENSORS 2
#endif

#ifndef MAX_NEO_GROUP
#define MAX_NEO_GROUP 4
#endif

#ifndef TAKE_HIGHEST_LUX
#define TAKE_HIGHEST_LUX 0
#endif

#ifndef TAKE_AVERAGE_LUX
#define TAKE_AVERAGE_LUX 1
#endif

#ifndef PRINT_BRIGHTNESS_SCALER_DEBUG
#define PRINT_BRIGHTNESS_SCALER_DEBUG false
#endif

#ifndef PRINT_LUX_READINGS
#define PRINT_LUX_READINGS false
#endif

#ifndef EXTREME_LUX_THRESHOLD
#define EXTREME_LUX_THRESHOLD 4000.0
#endif

#ifndef HIGH_LUX_THRESHOLD
#define HIGH_LUX_THRESHOLD 1200.0
#endif

#ifndef MID_LUX_THRESHOLD
#define MID_LUX_THRESHOLD 250.0
#endif

#ifndef LOW_LUX_THRESHOLD
#define LOW_LUX_THRESHOLD 10.0
#endif

#ifndef BRIGHTNESS_SCALER_MIN
#define BRIGHTNESS_SCALER_MIN 0.125
#endif

#ifndef BRIGHTNESS_SCALER_MAX
#define BRIGHTNESS_SCALER_MAX 2.0
#endif

#ifndef LUX_SHDN_LEN
#define LUX_SHDN_LEN 40
#endif

class LuxManager {
    // initalises its own lux sensors and then handles the readings
<<<<<<< HEAD
    public:
        LuxManager(long minrt, long maxrt);
        void linkNeoGroup(NeoGroup * n);
        void add6030Sensors(float gain, int _int);
        void add7700Sensors();
        void addSensorTcaIdx(String _name, int tca);
=======
  public:
    LuxManager(long minrt, long maxrt);
    void linkNeoGroup(NeoGroup * n);
    void addLuxSensor(int tca, String _name);
    void addLuxSensor(String _name);
>>>>>>> added in a flag for when a project doees not have a TCA

        double getLux() {
            return global_lux;
        };
        void startTCA7700Sensors(byte gain, byte integration);

        // to help multiple lux managers coordinate
        double forceLuxReading();
        void calibrate(long len, bool first_time);

        bool update();
        void resetMinMax();
        double min_reading = 9999.9;
        double max_reading = 0.0;

        double getAvgLux();
        void   resetAvgLux();

        String getName(int i) {return names[i];};

        // brightness and brightness scalers
        double getBrightnessScaler();
        double getBrightnessScalerAvg();
        void resetBrightnessScalerAvg();
        bool getExtremeLux() {return extreme_lux;};

        double brightness_scaler = 0.0;
        double brightness_scaler_avg = 0.0;
        double lux[MAX_LUX_SENSORS];
        double global_lux = 400;
        ///////////////////////////////////
        bool sensor_active[MAX_LUX_SENSORS];

    private:
        Adafruit_VEML7700       sensors_7700[MAX_LUX_SENSORS];
        SparkFun_Ambient_Light  sensors_6030[MAX_LUX_SENSORS] = {
            SparkFun_Ambient_Light(V6030_ADDR1), SparkFun_Ambient_Light(V6030_ADDR2)};

        uint8_t mode = TAKE_HIGHEST_LUX;
        uint8_t num_sensors = 0;
        uint8_t num_6030_sensors = 0;
        uint8_t num_7700_sensors = 0;

        int tca_addr[MAX_LUX_SENSORS];

        NeoGroup *neos[MAX_NEO_GROUP];
        uint8_t num_neo_groups = 0;
        String names[MAX_LUX_SENSORS];

        void updateMinMax();

        double past_readings[MAX_LUX_SENSORS][10];

        void readLux();

        unsigned long min_reading_time = 1000*60*3;
        unsigned long max_reading_time = 1000*60*3;

        elapsedMillis last_reading;
        long polling_rate;

        // for datalogging and such
        double lux_total;
        double lux_readings;

        // for brightness
        double brightness_scaler_total;
        uint32_t num_brightness_scaler_vals;

        double calculateBrightnessScaler();

<<<<<<< HEAD
        double read();
        bool extreme_lux;
=======
    double read();
    bool extreme_lux;
    bool tca_present = false;
>>>>>>> added in a flag for when a project doees not have a TCA
};

LuxManager::LuxManager(long minrt, long maxrt){
    min_reading_time = minrt;
    max_reading_time = maxrt;
}

//////////////////////////// lux and stuff /////////////////////////

<<<<<<< HEAD

void LuxManager::addSensorTcaIdx(String _name, int tca){
    // will return true if sensor is found and false if it is not
    names[num_sensors] = _name;
    tca_addr[num_sensors] = tca;
    sensors_7700[num_sensors] = Adafruit_VEML7700();
    sensor_active[num_sensors] = false;// not active until startSensors() is called
    num_sensors++;
    num_7700_sensors++;
}
/*
// for adding a sensor which is not based on tca
void LuxManager::addSensorI2CAddr(String _name, int addr){
names[num_sensors] = _name;
num_sensors] = Adafruit_VEML7700();
sensor_active[num_sensors] = false;// not active until startSensors() is called
num_sensors++;
}
*/

void LuxManager::add7700Sensors() {
    for (int i = 0; i < 20; i++) {
        Serial.println("WARNING: add7700Sensor() - TODO");
        delay(150);
    }
}

void LuxManager::add6030Sensors(float gain, int _int) {
    Wire.begin();
    Serial.println("Adding VEML6030 Lux Sensors");
    names[num_sensors] = "Front";
    // sensors_6030[num_sensors] = SparkFun_Ambient_Light(V6030_ADDR1);
    if (sensors_6030[num_sensors].begin()){
        Serial.println("added first VEML6030 sensor");
        sensors_6030[num_sensors].setGain(gain);
        sensors_6030[num_sensors].setIntegTime(_int);
        Serial.println("configured first VEML6030 sensor");
        sensor_active[num_sensors] = true;
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

        num_sensors++;
        num_6030_sensors++;
    }
    else {
        Serial.println("WARNING - TROUBLE STARTING VEML6030 SENSOR");
    }
    Serial.print(num_sensors);
    Serial.println(" sensors were added correctly");
=======
void LuxManager::addLuxSensor(int tca, String _name){
  names[num_sensors] = _name;
  tca_addr[num_sensors] = tca;
  tca_present = true;
  sensors[num_sensors] = Adafruit_VEML7700();
  sensor_active[num_sensors] = false;// not active until startSensors() is called
  num_sensors++;
}

void LuxManager::addLuxSensor(String _name){
  names[num_sensors] = _name;
  tca_present = false;
  sensors[num_sensors] = Adafruit_VEML7700();
  sensor_active[num_sensors] = false;// not active until startSensors() is called
  num_sensors++;
>>>>>>> added in a flag for when a project doees not have a TCA
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
    return lux_total / (double) lux_readings;
}

void LuxManager::resetAvgLux() {
    lux_total = 0;
    lux_readings = 0;
    dprintln(P_LUX, "reset lux_total and lux_readings");
}

void tcaselect(uint8_t i) {
    if (i > 7) return;
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}

void LuxManager::startTCA7700Sensors(byte gain, byte integration) {
    Wire.begin();
    delay(500);
    for (int i = 0; i < num_sensors; i++){
<<<<<<< HEAD
        Serial.print("\nattempting to start lux sensor ");
        Serial.println(names[i]);
        if (tca_addr[i] > -1) {
            tcaselect(tca_addr[i]);
        }
        if (!sensors_7700[i].begin()) {
            Serial.print("ERROR ---- VEML "); Serial.print(names[i]); Serial.println(" not found");
            neos[i]->colorWipe(255, 0, 0, 1.0);
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
            sensors_7700[i].setGain(gain); // talk about gain and integration time in the thesis
            sensors_7700[i].setIntegrationTime(integration);// 800ms was default
=======
      Serial.print("attempting to start lux sensor ");
      Serial.println(names[i]);
      if (tca_present == true) {
          if (tca_addr[i] > -1) {
            tcaselect(tca_addr[i]);
          }
      }
      if (!sensors[i].begin()) {
        Serial.print("ERROR ---- VEML "); Serial.print(names[i]); Serial.println(" not found");
        neos[i]->colorWipe(255, 100, 0);
        unsigned long then = millis();
        while (millis() < then + 5000) {
          Serial.print(".");
          delay(100);
>>>>>>> added in a flag for when a project doees not have a TCA
        }
    }
}

void LuxManager::readLux() {
    /* This function handles reading each individual lux sensor and then
     * determining what the global lux is. 
     */

    if (num_7700_sensors > 0) {
        for (int i = 0; i < num_7700_sensors; i++){
            // read each sensor
            if (tca_addr[i] > -1) {
                tcaselect(tca_addr[i]);
            }
            double _t = sensors_7700[i].readLux();
            if (_t > 1000000) {
                _t = sensors_7700[i].readLux();
                if (_t  < 1000000) {
                    lux[i] = _t;
                }
            } else {
                lux[i] = _t;
            }
        }
    }
    if (num_6030_sensors > 0) {
        for (int i = 0; i < num_6030_sensors; i++){
            double _t = sensors_6030[i].readLight();
            if (_t > 1000000) {
                _t = sensors_6030[i].readLight();
                if (_t  < 1000000) {
                    lux[num_7700_sensors + i] = _t;
                }
            } else {
                lux[num_7700_sensors + i] = _t;
            }
        }
    }

    for (int i = 0; i < num_sensors; i++) {
        dprint(P_LUX_READINGS, "Lux for sensor #");
        dprint(P_LUX_READINGS, i);
        dprint(P_LUX_READINGS, ": ");
        dprintln(P_LUX_READINGS, lux[i]);
    }

    if (mode == TAKE_HIGHEST_LUX) {
        double highest = 0.0;
        for (int i = 0; i < num_sensors; i++) {
            if (highest < lux[i]) {
                highest = lux[i];
            }
        }
        global_lux = highest;
        dprint(P_LUX_READINGS, "Taking highest lux: ");
        dprintln(P_LUX_READINGS, global_lux);

    } else if (mode == TAKE_AVERAGE_LUX) {
        double average = 0.0;
        for (int i = 0; i < num_sensors; i++) {
            average = average + lux[i];
        }
        average = average / num_sensors;
        global_lux = average;
        dprint(P_LUX_READINGS, "Taking average lux: ");
        dprintln(P_LUX_READINGS, global_lux);
    }
    lux_total = lux_total + global_lux;
    lux_readings++;
    // now trigger an update to the brightness scaler
    brightness_scaler = calculateBrightnessScaler();
    num_brightness_scaler_vals++;
    brightness_scaler_total += brightness_scaler;
    // update all linked neogroups with the new brighness scaler
    for (int i = 0; i < num_neo_groups; i++){
        neos[i]->setBrightnessScaler(brightness_scaler);
    }
    if (P_BRIGHTNESS_SCALER == 0) {
        dprint(P_LUX_READINGS, "\tbs: "); 
        dprintln(P_LUX_READINGS, brightness_scaler);
    };
    updateMinMax();
    last_reading = 0;
}

double LuxManager::calculateBrightnessScaler() {

    // todo need to make this function better... linear mapping does not really work, need to map li
    double bs;
    // conduct brightness scaling depending on if the reading is above or below the mid thresh
    // is the unconstrained lux above the extreme_lux_thresh?
    dprint(P_BRIGHTNESS_SCALER, names[0]);
    if (global_lux >= EXTREME_LUX_THRESHOLD) {
        bs = 0.0;
        dprintln(P_BRIGHTNESS_SCALER, " Neopixel brightness scaler set to 0.0 due to extreme lux");
        if (extreme_lux == false) {
            extreme_lux = true;
        }
    } 
    else if (global_lux >= HIGH_LUX_THRESHOLD) {
        bs = BRIGHTNESS_SCALER_MAX;
        dprintln(P_BRIGHTNESS_SCALER, " is greater than the MAX_LUX_THRESHOLD, setting brightness scaler to BRIGHTNESS_SCALER_MAX");
        if (extreme_lux == true) {
            extreme_lux = false;
        }
    }
    else if (global_lux >= MID_LUX_THRESHOLD) {
        bs = 1.0;
        // bs = 1.0 + (BRIGHTNESS_SCALER_MAX - 1.0) * ((lux - MID_LUX_THRESHOLD) / (HIGH_LUX_THRESHOLD - MID_LUX_THRESHOLD));
        dprintln(P_BRIGHTNESS_SCALER, " is greater than the MID_LUX_THRESHOLD, setting brightness scaler to 1.0");
        if (extreme_lux == true) {
            extreme_lux = false;
        }
    }
    else if (global_lux >= LOW_LUX_THRESHOLD)  {
        bs = (global_lux - LOW_LUX_THRESHOLD) / (MID_LUX_THRESHOLD - LOW_LUX_THRESHOLD) * (1.0 - BRIGHTNESS_SCALER_MIN);
        bs += BRIGHTNESS_SCALER_MIN;
        dprintln(P_BRIGHTNESS_SCALER, " is greater than the LOW_LUX_THRESHOLD, setting brightness scaler to a value < 1.0");
        if (extreme_lux == true) {
            extreme_lux = false;
        }
    } else {
        bs = BRIGHTNESS_SCALER_MIN;
        dprintln(P_BRIGHTNESS_SCALER, " is lower than the LOW_LUX_THRESHOLD, setting brightness scaler to BRIGHTNESS_SCALER_MIN");
        if (extreme_lux == true) {
            extreme_lux = false;
        }
    }
    dprint(P_BRIGHTNESS_SCALER, "global_lux of ");
    dprint(P_BRIGHTNESS_SCALER, global_lux);
    dprint(P_BRIGHTNESS_SCALER, "has resulted in a brightness_scaler of ");
    dprintln(P_BRIGHTNESS_SCALER, bs);
    return bs;
}

double LuxManager::getBrightnessScaler() {
    return brightness_scaler;
}

double LuxManager::getBrightnessScalerAvg() {
    brightness_scaler_avg = brightness_scaler_total / num_brightness_scaler_vals;
    return brightness_scaler_avg;
}

void LuxManager::resetBrightnessScalerAvg() {
    brightness_scaler_avg = 0;
    num_brightness_scaler_vals = 0;
    brightness_scaler_total = 0;
}

void LuxManager::updateMinMax() {
    for (int i = 0; i < num_sensors; i++) {
        if (lux[i] < min_reading && lux[i] > 0.0) {
            min_reading = lux[i];
        } else if (lux[i] > max_reading && lux[i] < 10000) {
            max_reading = lux[i];
        }
    }
}

void LuxManager::resetMinMax() {
    min_reading = 10000;
    max_reading = 0;
}
// todo move me to the correct place
void LuxManager::calibrate(long len, bool first_time = true) {
    // todo change this function so it takes the average of these readings
    // TODO this is broken now that the manager looks after multiple sensors....
    // 
    Serial.println("WARNING THE LUX CALIBRATION IS CURRENTLY OFFLINE");
    /*
    printMinorDivide();
    Serial.println("Starting Lux Calibration");
    Serial.println("Will print out the highest lux recorded from either sensor 10x and average the reading for the 1st global_lux value");
    double lux_tot = 0.0;
    for (int s = 0; s < num_sensors; s++){
        for (int i = 0; i < 10; i++) {
            delay(len / 10);
            forceLuxReading(); // todo change this to not be hard coded
            if (first_time) {
                Serial.print(global_lux);
                Serial.print("  ");
            }
            lux_tot += global_lux;
            // when we have the first 10 readings
        }
        Serial.print("\nAverage lux readings : ");
        global_lux = lux_tot / 10.0;
        Serial.print(global_lux);
        Serial.println();
        if (first_time) {
            lux_total = 0;
            lux_readings = 0;
        }
    }
    printMinorDivide();
    */
}

double LuxManager::forceLuxReading() {
    readLux();
    return global_lux;
}

bool LuxManager::update() {
    for (int i = 0; i < num_sensors; i++) {
        // if the LEDs are off 
        if (neos[i]->getLedsOn() == false){ 
            dprint(P_LUX, "LEDs have been off for ");
            dprint(P_LUX, neos[i]->getOffLen());
            dprintln(P_LUX, "ms");
            // and have been off for more than the shdn_len
            if (neos[i]->getOffLen() >= LUX_SHDN_LEN) {
                // if currently in extreme lux shutdown then poll 20x faster
                dprint(P_LUX, "linked neos->getLedsOn() is false and getOffLen() > LUX_SHDN_LEN\nLast reading / min_reading_time: ");
                dprint(P_LUX, last_reading);
                dprint(P_LUX, " / ");
                dprintln(P_LUX, min_reading_time);
                if (extreme_lux && last_reading > min_reading_time * 0.05) {
                    dprint(P_LUX, "QUICK UPDATE due to extreme lux reading");
                    readLux();
                    if (neos[i]->getShdnLen() > LUX_SHDN_LEN) {
                        neos[i]->powerOn();
                        dprint(P_LUX, "Sending Power On Message");
                    }
                    return true;
                }
                else if (last_reading > min_reading_time) {
                    dprint(P_LUX, "Normal Lux Reading");
                    readLux();
                    if (neos[i]->getShdnLen() > LUX_SHDN_LEN) {
                        neos[i]->powerOn();
                        dprint(P_LUX, "Sending Power On Message");
                    }
                    return true;
                }
            }
        }
        else if (last_reading > max_reading_time) { // the LEDs will be off when this logic comes up
            // shdn len has to be longer to ensure the lux sensors get a good reading
            dprint(P_LUX, "last lux reading is greater than last reading time of ");
            dprint(P_LUX, max_reading_time);
            dprintln(P_LUX, " and neo LEDs are on");
            if (!neos[i]->isInShutdown()) {
                dprint(P_LUX, "neos not in shutdown, putting them in shutdown now for a forced lux reading");
                neos[i]->shutdown(LUX_SHDN_LEN*2);
            }
            dprintln(P_LUX);
        }
    }
    return false;
}

#endif // __LUX_H__
