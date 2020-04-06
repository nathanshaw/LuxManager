#ifndef __LUX_H__
#define __LUX_H__

#include "Adafruit_VEML7700.h"
#include <Wire.h>
#include "../Configuration.h"
#include "../NeopixelManager/NeopixelManager.h"

// TODO add code so if the TCA is not available things are cool... also add firmware #define to control this
#define TCAADDR 0x70

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

class LuxManager {
    // initalises its own lux sensors and then handles the readings
  public:
    LuxManager(long minrt, long maxrt);
    void linkNeoGroup(NeoGroup * n);
    void addLuxSensor(int tca, String _name);

    double getLux() {
      return global_lux;
    };
    void startSensors(byte gain, byte integration);

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
    double global_lux;

  private:
    Adafruit_VEML7700 sensors[MAX_LUX_SENSORS];
    uint8_t mode = TAKE_HIGHEST_LUX;
    uint8_t num_sensors = 0;
    bool sensor_active[MAX_LUX_SENSORS];

    int tca_addr[MAX_LUX_SENSORS];

    NeoGroup *neos[MAX_NEO_GROUP];
    uint8_t num_neo_groups = 0;
    String names[MAX_LUX_SENSORS];

    void updateMinMax();

    double past_readings[MAX_LUX_SENSORS][10];

    void readLux();

    unsigned long min_reading_time;
    unsigned long max_reading_time;

    elapsedMillis last_reading;
    long polling_rate;

    // for datalogging and such
    double lux_total;
    double lux_readings;

    // for brightness
    double brightness_scaler_total;
    uint32_t num_brightness_scaler_vals;

    double calculateBrightnessScaler();

    double read();
    bool extreme_lux;
};

LuxManager::LuxManager(long minrt, long maxrt){
      min_reading_time = minrt;
      max_reading_time = maxrt;
}

//////////////////////////// lux and stuff /////////////////////////

void LuxManager::addLuxSensor(int tca, String _name){
  names[num_sensors] = _name;
  tca_addr[num_sensors] = tca;
  sensors[num_sensors] = Adafruit_VEML7700();
  sensor_active[num_sensors] = false;// not active until startSensors() is called
  num_sensors++;
}

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
  dprintln(PRINT_LUX_DEBUG, "reset lux_total and lux_readings");
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void LuxManager::startSensors(byte gain, byte integration) {
    Wire.begin();
    for (int i = 0; i < num_sensors; i++){
      Serial.print("attempting to start lux sensor ");
      Serial.println(names[i]);
      if (tca_addr[i] > -1) {
        tcaselect(tca_addr[i]);
      }
      if (!sensors[i].begin()) {
        Serial.print("ERROR ---- VEML "); Serial.print(names[i]); Serial.println(" not found");
        neos[i]->colorWipe(255, 100, 0);
        unsigned long then = millis();
        while (millis() < then + 5000) {
          Serial.print(".");
          delay(100);
        }
      }
      else {
        Serial.print("VEML "); Serial.print(names[i]); Serial.println(" found");
        sensors[i].setGain(gain); // talk about gain and integration time in the thesis
        sensors[i].setIntegrationTime(integration);// 800ms was default
      }
    }
}

void LuxManager::readLux() {
    /* This function handles reading each individual lux sensor and then
     * determining what the global lux is. 
     */
    for (int i = 0; i < num_sensors; i++){
        // read each sensor
      if (tca_addr[i] > -1) {
        tcaselect(tca_addr[i]);
      }
      double _t = sensors[i].readLux();
      if (_t > 1000000) {
          _t = sensors[i].readLux();
          if (_t  < 1000000) {
              lux[i] = _t;
          }
      } else {
          lux[i] = _t;
      }
    }
    if (mode == TAKE_HIGHEST_LUX) {
        double highest = 0.0;
        for (int i = 0; i < num_sensors; i++) {
            if (highest < lux[i]) {
                highest = lux[i];
            }
        }
        global_lux = highest;
    } else if (mode == TAKE_AVERAGE_LUX) {
        double average;
        for (int i = 0; i < num_sensors; i++) {
            average = average + lux[i];
        }
        average = average / num_sensors;
        global_lux = average;
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
      if (PRINT_BRIGHTNESS_SCALER_DEBUG == 0) {
          dprint(PRINT_LUX_READINGS, "\tbs: "); 
          dprintln(PRINT_LUX_READINGS, brightness_scaler);
      };
      updateMinMax();
      last_reading = 0;
}

double LuxManager::calculateBrightnessScaler() {
    // todo need to make this function better... linear mapping does not really work, need to map li
    double bs;
    // conduct brightness scaling depending on if the reading is above or below the mid thresh
    // is the unconstrained lux above the extreme_lux_thresh?
    dprint(PRINT_BRIGHTNESS_SCALER_DEBUG, names[0]);
    if (global_lux >= EXTREME_LUX_THRESHOLD) {
        bs = 0.0;
        dprintln(PRINT_BRIGHTNESS_SCALER_DEBUG, " Neopixel brightness scaler set to 0.0 due to extreme lux");
        if (extreme_lux == false) {
            extreme_lux = true;
        }
    } 
    else if (global_lux >= HIGH_LUX_THRESHOLD) {
        bs = BRIGHTNESS_SCALER_MAX;
        dprintln(PRINT_BRIGHTNESS_SCALER_DEBUG, " is greater than the MAX_LUX_THRESHOLD, setting brightness scaler to BRIGHTNESS_SCALER_MAX");
        if (extreme_lux == true) {
            extreme_lux = false;
        }
    }
    else if (global_lux >= MID_LUX_THRESHOLD) {
        bs = 1.0;
        // bs = 1.0 + (BRIGHTNESS_SCALER_MAX - 1.0) * ((lux - MID_LUX_THRESHOLD) / (HIGH_LUX_THRESHOLD - MID_LUX_THRESHOLD));
        dprintln(PRINT_BRIGHTNESS_SCALER_DEBUG, " is greater than the MID_LUX_THRESHOLD, setting brightness scaler to 1.0");
        if (extreme_lux == true) {
            extreme_lux = false;
        }
    }
    else if (global_lux >= LOW_LUX_THRESHOLD)  {
        bs = (global_lux - LOW_LUX_THRESHOLD) / (MID_LUX_THRESHOLD - LOW_LUX_THRESHOLD) * (1.0 - BRIGHTNESS_SCALER_MIN);
        bs += BRIGHTNESS_SCALER_MIN;
        dprintln(PRINT_BRIGHTNESS_SCALER_DEBUG, " is greater than the LOW_LUX_THRESHOLD, setting brightness scaler to a value < 1.0");
        if (extreme_lux == true) {
            extreme_lux = false;
        }
    } else {
        bs = BRIGHTNESS_SCALER_MIN;
        dprintln(PRINT_BRIGHTNESS_SCALER_DEBUG, " is lower than the LOW_LUX_THRESHOLD, setting brightness scaler to BRIGHTNESS_SCALER_MIN");
        if (extreme_lux == true) {
            extreme_lux = false;
        }
    }
    dprint(PRINT_BRIGHTNESS_SCALER_DEBUG, "global_lux:\t");
    dprint(PRINT_BRIGHTNESS_SCALER_DEBUG, global_lux);
    dprint(PRINT_BRIGHTNESS_SCALER_DEBUG, "\tbrightness_scaler:\t");
    dprintln(PRINT_BRIGHTNESS_SCALER_DEBUG, bs);
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
  printMinorDivide();
  Serial.println("Starting Lux Calibration");
  double lux_tot = 0.0;
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
  Serial.println("------------------------\n");
}

double LuxManager::forceLuxReading() {
  readLux();
  return global_lux;
}

bool LuxManager::update() {
    for (int i = 0; i < num_sensors; i++) {
      if ((neos[i]->getLedsOn() == false && neos[i]->getOnOffLen() >= LUX_SHDN_LEN) || (neos[i]->getShdnLen() > LUX_SHDN_LEN)) {
          // if currently in extreme lux shutdown then poll 20x faster
        if (extreme_lux && last_reading > min_reading_time * 0.05) {
            dprint(PRINT_LUX_DEBUG, "QUICK UPDATE due to extreme lux reading");
            readLux();
            if (neos[i]->getShdnLen() > LUX_SHDN_LEN) {
              neos[i]->powerOn();
            }
            return true;
        }
        else if (last_reading > min_reading_time) {
            readLux();
            if (neos[i]->getShdnLen() > LUX_SHDN_LEN) {
              neos[i]->powerOn();
            }
            return true;
        }
      } else if (last_reading > max_reading_time && neos[i]->getLedsOn() == true) {
        // shdn len has to be longer to ensure the lux sensors get a good reading
        neos[i]->shutdown(LUX_SHDN_LEN*1.25);
      }
    }
  return false;
}

#endif // __LUX_H__
