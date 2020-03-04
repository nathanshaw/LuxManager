#ifndef __LUX_H__
#define __LUX_H__

#include "Adafruit_VEML7700.h"
#include <Wire.h>
#include "../Configuration.h"
#include "../NeopixelManager/NeopixelManager.h"

// TODO add code so if the TCA is not available things are cool... also add firmware #define to control this
#define TCAADDR 0x70

class LuxManager {
    // initalises its own lux sensors and then handles the readings
  public:
    LuxManager(long minrt, long maxrt, int tca, String _name, NeoGroup *n);
    double getLux() {
      return lux;
    };
    void startSensor(byte g, byte r);
    double forceLuxReading();
    void setLuxValue(double);
    void calibrate(long len, bool first_time);
    bool update();

    void resetMinMax();

    double min_reading = 9999.9;
    double max_reading = 0.0;

    double getAvgLux();
    void   resetAvgLux();

    String getName() {return id;};

    double lux;

    // brightness and brightness scalers
    double getBrightnessScaler();
    double getBrightnessScalerAvg();
    void resetBrightnessScalerAvg();
    double brightness_scaler = 0.0;
    double brightness_scaler_avg = 0.0;

  private:
    Adafruit_VEML7700 sensor = Adafruit_VEML7700();
    uint8_t num_sensors = 0;

    int tca_addr;

    NeoGroup *neo;
    String id = "";

    void updateMinMax();

    double past_readings[10];
    void readLux();
    void readLux(Adafruit_VEML7700 *);

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

    double checkForLuxOverValue();

    double read();
};

//////////////////////////// lux and stuff /////////////////////////

LuxManager::LuxManager (long minrt, long maxrt, int tca, String _name, NeoGroup *n) {
  num_sensors = 1;
  tca_addr = tca;
  id = _name;
  neo = n;
  min_reading_time = minrt;
  max_reading_time = maxrt;
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

void LuxManager::startSensor(byte g, byte r) {
  Serial.print("attempting to start lux sensor...");
  Wire.begin();
  if (tca_addr > -1) {
    tcaselect(tca_addr);
  }
  if (!sensor.begin()) {
    Serial.print("ERROR ---- VEML "); Serial.print(id); Serial.println(" not found");
    neo->colorWipe(255, 100, 0);
    unsigned long then = millis();
    while (millis() < then + 5000) {
      Serial.print(".");
      delay(100);
    }
  }
  else {
    Serial.print("VEML "); Serial.print(id); Serial.println(" found");
    sensor.setGain(g);
    sensor.setIntegrationTime(r);// 800ms was default
  }
}

double LuxManager::checkForLuxOverValue() {
  // sometimes the sensor will give an incorrect extremely high reading, this compensates for this...
  if (lux > 100000) {
    dprintln(PRINT_LUX_DEBUG, "lux "); dprintln(PRINT_LUX_DEBUG, id); dprintln(PRINT_LUX_DEBUG, " reading error: ");
    dprintln(PRINT_LUX_DEBUG, (String)lux);
    // take the reading again
    // todo also need to remove the current reading from the logs 
    if (SMOOTH_LUX_READINGS && lux != 0) {
      lux = (lux + sensor.readLux()) * 0.5;
      lux_total += lux;
      lux_readings++;
    } else {
      lux = sensor.readLux();
      lux_total += lux;
      lux_readings++;
    }
  }
  return lux;
}

void LuxManager::setLuxValue(double temp) {
  dprintln(PRINT_LUX_DEBUG, "\nForce lux reading");
  dprint(PRINT_LUX_DEBUG, last_reading); dprint(PRINT_LUX_DEBUG, " readLux("); dprint(PRINT_LUX_DEBUG, id); dprint(PRINT_LUX_DEBUG, ")\t");
  if (temp > 10000) {
    return;
  }
  if (SMOOTH_LUX_READINGS && lux != 0) {
    lux = (lux + temp) * 0.5;
    checkForLuxOverValue();
    lux_total += lux;
    lux_readings++;
  } else {
    lux = temp;
    checkForLuxOverValue();
    lux_total += lux;
    lux_readings++;
  }
  dprint(PRINT_LUX_READINGS, id); dprint(PRINT_LUX_READINGS,":");
  dprint(PRINT_LUX_READINGS, lux); dprintln(PRINT_LUX_READINGS,"");
  // update the brightness scales TODO , this logic does not work if the number of lux sensors is less than the number of groups
  // todo have the brightness scaler mapping
  brightness_scaler = calculateBrightnessScaler();
  num_brightness_scaler_vals++;
  brightness_scaler_total += brightness_scaler;
  brightness_scaler_avg = brightness_scaler_total / num_brightness_scaler_vals;

  neo->setBrightnessScaler(brightness_scaler);
  if (PRINT_BRIGHTNESS_SCALER_DEBUG == 0) {
      dprint(PRINT_LUX_READINGS, "\tbs: "); 
      dprintln(PRINT_LUX_READINGS, brightness_scaler);
  };
  updateMinMax();
  last_reading = 0;
}

void LuxManager::readLux() {
    readLux(&sensor);
}

void LuxManager::readLux(Adafruit_VEML7700 *s) {
  dprint(PRINT_LUX_DEBUG, last_reading); dprint(PRINT_LUX_DEBUG, " readLux("); dprint(PRINT_LUX_DEBUG, id); dprint(PRINT_LUX_DEBUG, ")\t");
  if (tca_addr > -1) {
    tcaselect(tca_addr);
  }
  double temp  = s->readLux();
  if (temp > 10000) {
    return;
  }
  if (SMOOTH_LUX_READINGS && lux != 0) {
    lux = (lux + temp) * 0.5;
    checkForLuxOverValue();
    lux_total += lux;
    lux_readings++;
  } else {
    lux = temp;
    checkForLuxOverValue();
    lux_total += lux;
    lux_readings++;
  }
  dprint(PRINT_LUX_READINGS, id); dprint(PRINT_LUX_READINGS,":");
  dprint(PRINT_LUX_READINGS, lux); dprintln(PRINT_LUX_READINGS,"");
  // dprint(PRINT_LUX_READINGS, "lux_readings "); dprint(PRINT_LUX_READINGS, id);
  // dprint(PRINT_LUX_READINGS, " increased to : "); dprintln(PRINT_LUX_READINGS, lux_readings);
  // update the brightness scales TODO , this logic does not work if the number of lux sensors is less than the number of groups
  // todo have the brightness scaler mapping
  brightness_scaler = calculateBrightnessScaler();
  num_brightness_scaler_vals++;
  brightness_scaler_total += brightness_scaler;
  brightness_scaler_avg = brightness_scaler_total / num_brightness_scaler_vals;
  neo->setBrightnessScaler(brightness_scaler);
  
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
    dprint(PRINT_BRIGHTNESS_SCALER_DEBUG, id);
    if (lux >= EXTREME_LUX_THRESHOLD) {
        bs = 0.0;
        dprintln(PRINT_BRIGHTNESS_SCALER_DEBUG, " Neopixel brightness scaler set to 0.0 due to extreme lux");
        if (neo->getLuxShdn() == false) {
            neo->setExtremeLuxShdn(1);
        }
    } 
    else if (lux >= HIGH_LUX_THRESHOLD) {
        bs = BRIGHTNESS_SCALER_MAX;
        dprintln(PRINT_BRIGHTNESS_SCALER_DEBUG, " is greater than the MAX_LUX_THRESHOLD, setting brightness scaler to BRIGHTNESS_SCALER_MAX");
        if (neo->getLuxShdn() == true) {
            neo->setExtremeLuxShdn(false);
        }
    }
    else if (lux >= MID_LUX_THRESHOLD) {
        bs = 1.0;
        // bs = 1.0 + (BRIGHTNESS_SCALER_MAX - 1.0) * ((lux - MID_LUX_THRESHOLD) / (HIGH_LUX_THRESHOLD - MID_LUX_THRESHOLD));
        dprintln(PRINT_BRIGHTNESS_SCALER_DEBUG, " is greater than the MID_LUX_THRESHOLD, setting brightness scaler to 1.0");
        if (neo->getLuxShdn() == true) {
            neo->setExtremeLuxShdn(false);
        }
    }
    else if (lux >= LOW_LUX_THRESHOLD)  {
        bs = (lux - LOW_LUX_THRESHOLD) / (MID_LUX_THRESHOLD - LOW_LUX_THRESHOLD) * (1.0 - BRIGHTNESS_SCALER_MIN);
        bs += BRIGHTNESS_SCALER_MIN;
        dprintln(PRINT_BRIGHTNESS_SCALER_DEBUG, " is greater than the LOW_LUX_THRESHOLD, setting brightness scaler to a value < 1.0");
        if (neo->getLuxShdn() == true) {
            neo->setExtremeLuxShdn(false);
        }
    } else {
        bs = BRIGHTNESS_SCALER_MIN;
        dprintln(PRINT_BRIGHTNESS_SCALER_DEBUG, " is lower than the LOW_LUX_THRESHOLD, setting brightness scaler to BRIGHTNESS_SCALER_MIN");
        if (neo->getLuxShdn() == true) {
            neo->setExtremeLuxShdn(false);
        }
    }
    dprint(PRINT_BRIGHTNESS_SCALER_DEBUG, "lux:\t");
    dprint(PRINT_BRIGHTNESS_SCALER_DEBUG, lux);
    dprint(PRINT_BRIGHTNESS_SCALER_DEBUG, "\tbrightness_scaler:\t");
    dprintln(PRINT_BRIGHTNESS_SCALER_DEBUG, bs);
    return bs;
}

double LuxManager::getBrightnessScaler() {
    return brightness_scaler;
}

double LuxManager::getBrightnessScalerAvg() {
    return brightness_scaler_avg;
}

void LuxManager::resetBrightnessScalerAvg() {
    brightness_scaler_avg = 0;
    num_brightness_scaler_vals = 0;
    brightness_scaler_total = 0;
}

void LuxManager::updateMinMax() {
  if (lux < min_reading && lux > 0.0) {
    min_reading = lux;
  } else if (lux > max_reading && lux < 10000) {
    max_reading = lux;
  }
}

void LuxManager::resetMinMax() {
  min_reading = 10000;
  max_reading = 0;
}
// todo move me to the correct place
void LuxManager::calibrate(long len, bool first_time = true) {
  // todo change this function so it takes the average of these readings
  printMinorDivide();
  Serial.println("Starting Lux Calibration");
  double lux_tot = 0.0;
  for (int i = 0; i < 10; i++) {
    delay(len / 10);
    forceLuxReading(); // todo change this to not be hard coded
    if (first_time) {
      Serial.print(lux);
      Serial.print("  ");
    }
    lux_tot += lux;
    // when we have the first 10 readings
  }
  Serial.print("\nAverage lux readings : ");
  lux = lux_tot / 10.0;
  Serial.print(lux);
  Serial.println();
  if (first_time) {
    lux_total = 0;
    lux_readings = 0;
  }
  Serial.println("\nLux calibration finished");
  Serial.println("------------------------\n");
}

double LuxManager::forceLuxReading() {
  readLux();
  return lux;
}

bool LuxManager::update() {
  if ((neo->getLedsOn() == false && neo->getOnOffLen() >= LUX_SHDN_LEN) || (neo->getShdnLen() > LUX_SHDN_LEN)) {
      // if currently in extreme lux shutdown then poll 20x faster
    if (neo->getLuxShdn() && last_reading > min_reading_time * 0.05) {
        dprint(PRINT_LUX_DEBUG, "QUICK UPDATE due to extreme lux reading");
        readLux();
        if (neo->getShdnLen() > LUX_SHDN_LEN) {
          neo->powerOn();
        }
        return true;
    }
    else if (last_reading > min_reading_time) {
        readLux();
        if (neo->getShdnLen() > LUX_SHDN_LEN) {
          neo->powerOn();
        }
        return true;
    }
  } else if (last_reading > max_reading_time && neo->getLedsOn() == true) {
    // shdn len has to be longer to ensure the lux sensors get a good reading
    neo->shutdown(LUX_SHDN_LEN*1.25);
  }
  return false;
}

#endif // __LUX_H__
