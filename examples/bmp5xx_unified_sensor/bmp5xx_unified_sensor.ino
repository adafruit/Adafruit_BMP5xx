/*!
 * @file bmp5xx_unified_sensor.ino
 *
 * This is an example for the BMP5xx pressure and temperature sensor using
 * the Adafruit Unified Sensor API. This approach allows for easy integration
 * with other Adafruit sensor libraries and provides standardized sensor events.
 * 
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor "ladyada" Fried for Adafruit Industries.
 * BSD license, all text above must be included in any redistribution
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP5xx.h"

Adafruit_BMP5xx bmp; // Create BMP5xx object

// Get separate sensor objects for temperature and pressure
Adafruit_Sensor *bmp_temp = NULL;
Adafruit_Sensor *bmp_pressure = NULL;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // Wait for Serial Monitor to open
  
  Serial.println(F("Adafruit BMP5xx Unified Sensor API Example"));
  Serial.println();

  // Try to initialize the sensor using I2C with alternative address
  if (!bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
    Serial.println(F("Could not find a valid BMP5xx sensor, check wiring!"));
    while (1) delay(10);
  }

  Serial.println(F("BMP5xx found!"));

  // Get the unified sensor objects
  bmp_temp = bmp.getTemperatureSensor();
  bmp_pressure = bmp.getPressureSensor();

  // Print sensor details using the unified sensor API
  Serial.println(F("=== Temperature Sensor Details ==="));
  sensor_t temp_sensor;
  bmp_temp->getSensor(&temp_sensor);
  Serial.print(F("Sensor Name: ")); Serial.println(temp_sensor.name);
  Serial.print(F("Sensor Type: ")); Serial.println(temp_sensor.type);
  Serial.print(F("Driver Ver:  ")); Serial.println(temp_sensor.version);
  Serial.print(F("Unique ID:   ")); Serial.println(temp_sensor.sensor_id);
  Serial.print(F("Min Value:   ")); Serial.print(temp_sensor.min_value); Serial.println(F(" °C"));
  Serial.print(F("Max Value:   ")); Serial.print(temp_sensor.max_value); Serial.println(F(" °C"));
  Serial.print(F("Resolution:  ")); Serial.print(temp_sensor.resolution); Serial.println(F(" °C"));
  Serial.println();

  Serial.println(F("=== Pressure Sensor Details ==="));
  sensor_t pressure_sensor;
  bmp_pressure->getSensor(&pressure_sensor);
  Serial.print(F("Sensor Name: ")); Serial.println(pressure_sensor.name);
  Serial.print(F("Sensor Type: ")); Serial.println(pressure_sensor.type);
  Serial.print(F("Driver Ver:  ")); Serial.println(pressure_sensor.version);
  Serial.print(F("Unique ID:   ")); Serial.println(pressure_sensor.sensor_id);
  Serial.print(F("Min Value:   ")); Serial.print(pressure_sensor.min_value); Serial.println(F(" hPa"));
  Serial.print(F("Max Value:   ")); Serial.print(pressure_sensor.max_value); Serial.println(F(" hPa"));
  Serial.print(F("Resolution:  ")); Serial.print(pressure_sensor.resolution); Serial.println(F(" hPa"));
  Serial.println();

  // Configure sensor for optimal performance
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);

  Serial.println(F("=== Starting Unified Sensor Readings ==="));
  Serial.println();
}

void loop() {
  // Create sensor event structures
  sensors_event_t temp_event, pressure_event;

  // Get temperature event using unified sensor API
  if (bmp_temp->getEvent(&temp_event)) {
    Serial.print(F("Temperature: "));
    Serial.print(temp_event.temperature);
    Serial.print(F(" °C"));
    
    // Print additional event details
    Serial.print(F(" [Timestamp: "));
    Serial.print(temp_event.timestamp);
    Serial.print(F(" ms, Sensor ID: "));
    Serial.print(temp_event.sensor_id);
    Serial.println(F("]"));
  } else {
    Serial.println(F("Failed to get temperature event"));
  }

  // Get pressure event using unified sensor API
  if (bmp_pressure->getEvent(&pressure_event)) {
    Serial.print(F("Pressure:    "));
    Serial.print(pressure_event.pressure);
    Serial.print(F(" hPa"));
    
    // Print additional event details
    Serial.print(F(" [Timestamp: "));
    Serial.print(pressure_event.timestamp);
    Serial.print(F(" ms, Sensor ID: "));
    Serial.print(pressure_event.sensor_id);
    Serial.println(F("]"));

    // Calculate and display altitude using pressure
    float altitude = 44330.0 * (1.0 - pow(pressure_event.pressure / 1013.25, 0.1903));
    Serial.print(F("Altitude:    "));
    Serial.print(altitude);
    Serial.println(F(" m"));
  } else {
    Serial.println(F("Failed to get pressure event"));
  }

  Serial.println(F("---"));
  delay(2000); // Read every 2 seconds
}