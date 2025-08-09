/*!
 * @file bmp5xx_spi_test.ino
 *
 * This is a test sketch for the BMP5xx pressure and temperature sensor using SPI.
 * It demonstrates basic SPI communication and sensor readings.
 * 
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor "ladyada" Fried for Adafruit Industries.
 * BSD license, all text above must be included in any redistribution
 */

#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP5xx.h"

#define BMP5XX_CS_PIN 10
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP5xx bmp; // Create BMP5xx object

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // Wait for Serial Monitor to open
  
  Serial.println(F("Adafruit BMP5xx SPI Test!"));

  // Try to initialize the sensor using SPI
  // bmp.begin(CS_PIN, &SPI) - CS pin and SPI peripheral
  if (!bmp.begin(BMP5XX_CS_PIN, &SPI)) {
    Serial.println(F("Could not find a valid BMP5xx sensor, check wiring or "
                     "SPI connections!"));
    while (1) delay(10);
  }

  Serial.println(F("BMP5xx found via SPI!"));
  Serial.print(F("Using CS pin: "));
  Serial.println(BMP5XX_CS_PIN);
  Serial.println();

  // Set up basic sensor configuration
  Serial.println(F("=== Configuring Sensor ==="));
  
  Serial.println(F("Setting temperature oversampling to 2X..."));
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);

  Serial.println(F("Setting pressure oversampling to 16X..."));
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);

  Serial.println(F("Setting IIR filter to coefficient 3..."));
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);

  Serial.println(F("Setting output data rate to 50 Hz..."));
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);

  Serial.println(F("Setting power mode to normal..."));
  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);

  Serial.println();
  Serial.println(F("=== Starting Continuous Readings ==="));
  Serial.println();
}

void loop() {
  if (!bmp.performReading()) {
    Serial.println(F("Failed to perform reading"));
    return;
  }
  
  Serial.print(F("Temperature = "));
  Serial.print(bmp.temperature);
  Serial.println(F(" °C"));

  Serial.print(F("Pressure = "));
  Serial.print(bmp.pressure);
  Serial.println(F(" hPa"));

  Serial.print(F("Approx. Altitude = "));
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(F(" m"));

  Serial.println(F("---"));
  
  delay(1000); // Read every second
}