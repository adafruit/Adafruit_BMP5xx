/*!
 * @file bmp5xx_test.ino
 *
 * This is a comprehensive test sketch for the BMP5xx pressure and temperature sensor.
 * It demonstrates all settings with pretty-printed output and continuous mode operation.
 * 
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor "ladyada" Fried for Adafruit Industries.
 * BSD license, all text above must be included in any redistribution
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP5xx.h"

#define SEALEVELPRESSURE_HPA (1013.25)

// For SPI mode, uncomment the next line and comment out the I2C begin() call in setup()
// #define BMP5XX_CS_PIN 10

Adafruit_BMP5xx bmp; // Create BMP5xx object
bmp5xx_powermode_t desiredMode = BMP5XX_POWERMODE_NORMAL; // Cache desired power mode

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // Wait for Serial Monitor to open
  
  Serial.println(F("Adafruit BMP5xx Comprehensive Test!"));

  // Try to initialize the sensor
  // For I2C mode (default):
  if (!bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
  // For SPI mode (uncomment the line below and comment out the I2C line above):
  // if (!bmp.begin(BMP5XX_CS_PIN, &SPI)) {
    Serial.println(F("Could not find a valid BMP5xx sensor, check wiring!"));
    while (1) delay(10);
  }

  Serial.println(F("BMP5xx found!"));
  Serial.println();

  // Demonstrate all setter functions with range documentation
  Serial.println(F("=== Setting Up Sensor Configuration ==="));
  
  /* Temperature Oversampling Settings:
   * BMP5XX_OVERSAMPLING_1X   - 1x oversampling (fastest, least accurate)
   * BMP5XX_OVERSAMPLING_2X   - 2x oversampling  
   * BMP5XX_OVERSAMPLING_4X   - 4x oversampling
   * BMP5XX_OVERSAMPLING_8X   - 8x oversampling
   * BMP5XX_OVERSAMPLING_16X  - 16x oversampling
   * BMP5XX_OVERSAMPLING_32X  - 32x oversampling
   * BMP5XX_OVERSAMPLING_64X  - 64x oversampling
   * BMP5XX_OVERSAMPLING_128X - 128x oversampling (slowest, most accurate)
   */
  Serial.println(F("Setting temperature oversampling to 2X..."));
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);

  /* Pressure Oversampling Settings (same options as temperature):
   * Higher oversampling = better accuracy but slower readings
   * Recommended: 16X for good balance of speed/accuracy
   */
  Serial.println(F("Setting pressure oversampling to 16X..."));
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);

  /* IIR Filter Coefficient Settings:
   * BMP5XX_IIR_FILTER_BYPASS   - No filtering (fastest response)
   * BMP5XX_IIR_FILTER_COEFF_1  - Light filtering
   * BMP5XX_IIR_FILTER_COEFF_3  - Medium filtering
   * BMP5XX_IIR_FILTER_COEFF_7  - More filtering
   * BMP5XX_IIR_FILTER_COEFF_15 - Heavy filtering
   * BMP5XX_IIR_FILTER_COEFF_31 - Very heavy filtering
   * BMP5XX_IIR_FILTER_COEFF_63 - Maximum filtering
   * BMP5XX_IIR_FILTER_COEFF_127- Maximum filtering (slowest response)
   */
  Serial.println(F("Setting IIR filter to coefficient 3..."));
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);

  /* Output Data Rate Settings (Hz):
   * BMP5XX_ODR_240_HZ, BMP5XX_ODR_218_5_HZ, BMP5XX_ODR_199_1_HZ
   * BMP5XX_ODR_179_2_HZ, BMP5XX_ODR_160_HZ, BMP5XX_ODR_149_3_HZ
   * BMP5XX_ODR_140_HZ, BMP5XX_ODR_129_8_HZ, BMP5XX_ODR_120_HZ
   * BMP5XX_ODR_110_1_HZ, BMP5XX_ODR_100_2_HZ, BMP5XX_ODR_89_6_HZ
   * BMP5XX_ODR_80_HZ, BMP5XX_ODR_70_HZ, BMP5XX_ODR_60_HZ, BMP5XX_ODR_50_HZ
   * BMP5XX_ODR_45_HZ, BMP5XX_ODR_40_HZ, BMP5XX_ODR_35_HZ, BMP5XX_ODR_30_HZ
   * BMP5XX_ODR_25_HZ, BMP5XX_ODR_20_HZ, BMP5XX_ODR_15_HZ, BMP5XX_ODR_10_HZ
   * BMP5XX_ODR_05_HZ, BMP5XX_ODR_04_HZ, BMP5XX_ODR_03_HZ, BMP5XX_ODR_02_HZ
   * BMP5XX_ODR_01_HZ, BMP5XX_ODR_0_5_HZ, BMP5XX_ODR_0_250_HZ, BMP5XX_ODR_0_125_HZ
   */
  Serial.println(F("Setting output data rate to 50 Hz..."));
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);

  /* Power Mode Settings:
   * BMP5XX_POWERMODE_STANDBY     - Standby mode (no measurements)
   * BMP5XX_POWERMODE_NORMAL      - Normal mode (periodic measurements)
   * BMP5XX_POWERMODE_FORCED      - Forced mode (single measurement then standby)
   * BMP5XX_POWERMODE_CONTINUOUS  - Continuous mode (fastest measurements)
   * BMP5XX_POWERMODE_DEEP_STANDBY - Deep standby (lowest power)
   */
  Serial.println(F("Setting power mode to normal..."));
  desiredMode = BMP5XX_POWERMODE_NORMAL;
  bmp.setPowerMode(desiredMode);

  /* Enable/Disable Pressure Measurement:
   * true  - Enable pressure measurement (default)
   * false - Disable pressure measurement (temperature only)
   */
  Serial.println(F("Enabling pressure measurement..."));
  bmp.enablePressure(true);

  /* Interrupt Configuration:
   * BMP5XX_INTERRUPT_PULSED / BMP5XX_INTERRUPT_LATCHED - Interrupt mode
   * BMP5XX_INTERRUPT_ACTIVE_LOW / BMP5XX_INTERRUPT_ACTIVE_HIGH - Interrupt polarity  
   * BMP5XX_INTERRUPT_PUSH_PULL / BMP5XX_INTERRUPT_OPEN_DRAIN - Interrupt drive
   * BMP5XX_INTERRUPT_DATA_READY, BMP5XX_INTERRUPT_FIFO_FULL, etc. - Interrupt sources (can combine with |)
   */
  Serial.println(F("Configuring interrupt pin with data ready source..."));
  bmp.configureInterrupt(BMP5XX_INTERRUPT_LATCHED, BMP5XX_INTERRUPT_ACTIVE_HIGH, BMP5XX_INTERRUPT_PUSH_PULL, BMP5XX_INTERRUPT_DATA_READY, true);

  Serial.println();
  Serial.println(F("=== Current Sensor Configuration ==="));
  
  // Pretty print temperature oversampling inline
  Serial.print(F("Temperature Oversampling: "));
  switch(bmp.getTemperatureOversampling()) {
    case BMP5XX_OVERSAMPLING_1X:   Serial.println(F("1X")); break;
    case BMP5XX_OVERSAMPLING_2X:   Serial.println(F("2X")); break;
    case BMP5XX_OVERSAMPLING_4X:   Serial.println(F("4X")); break;
    case BMP5XX_OVERSAMPLING_8X:   Serial.println(F("8X")); break;
    case BMP5XX_OVERSAMPLING_16X:  Serial.println(F("16X")); break;
    case BMP5XX_OVERSAMPLING_32X:  Serial.println(F("32X")); break;
    case BMP5XX_OVERSAMPLING_64X:  Serial.println(F("64X")); break;
    case BMP5XX_OVERSAMPLING_128X: Serial.println(F("128X")); break;
    default: Serial.println(F("Unknown")); break;
  }
  
  // Pretty print pressure oversampling inline
  Serial.print(F("Pressure Oversampling: "));
  switch(bmp.getPressureOversampling()) {
    case BMP5XX_OVERSAMPLING_1X:   Serial.println(F("1X")); break;
    case BMP5XX_OVERSAMPLING_2X:   Serial.println(F("2X")); break;
    case BMP5XX_OVERSAMPLING_4X:   Serial.println(F("4X")); break;
    case BMP5XX_OVERSAMPLING_8X:   Serial.println(F("8X")); break;
    case BMP5XX_OVERSAMPLING_16X:  Serial.println(F("16X")); break;
    case BMP5XX_OVERSAMPLING_32X:  Serial.println(F("32X")); break;
    case BMP5XX_OVERSAMPLING_64X:  Serial.println(F("64X")); break;
    case BMP5XX_OVERSAMPLING_128X: Serial.println(F("128X")); break;
    default: Serial.println(F("Unknown")); break;
  }
  
  // Pretty print IIR filter coefficient inline
  Serial.print(F("IIR Filter Coefficient: "));
  switch(bmp.getIIRFilterCoeff()) {
    case BMP5XX_IIR_FILTER_BYPASS:   Serial.println(F("Bypass (No filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_1:  Serial.println(F("1 (Light filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_3:  Serial.println(F("3 (Medium filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_7:  Serial.println(F("7 (More filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_15: Serial.println(F("15 (Heavy filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_31: Serial.println(F("31 (Very heavy filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_63: Serial.println(F("63 (Maximum filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_127:Serial.println(F("127 (Maximum filtering)")); break;
    default: Serial.println(F("Unknown")); break;
  }
  
  // Pretty print output data rate inline
  Serial.print(F("Output Data Rate: "));
  switch(bmp.getOutputDataRate()) {
    case BMP5XX_ODR_240_HZ:   Serial.println(F("240 Hz")); break;
    case BMP5XX_ODR_218_5_HZ: Serial.println(F("218.5 Hz")); break;
    case BMP5XX_ODR_199_1_HZ: Serial.println(F("199.1 Hz")); break;
    case BMP5XX_ODR_179_2_HZ: Serial.println(F("179.2 Hz")); break;
    case BMP5XX_ODR_160_HZ:   Serial.println(F("160 Hz")); break;
    case BMP5XX_ODR_149_3_HZ: Serial.println(F("149.3 Hz")); break;
    case BMP5XX_ODR_140_HZ:   Serial.println(F("140 Hz")); break;
    case BMP5XX_ODR_129_8_HZ: Serial.println(F("129.8 Hz")); break;
    case BMP5XX_ODR_120_HZ:   Serial.println(F("120 Hz")); break;
    case BMP5XX_ODR_110_1_HZ: Serial.println(F("110.1 Hz")); break;
    case BMP5XX_ODR_100_2_HZ: Serial.println(F("100.2 Hz")); break;
    case BMP5XX_ODR_89_6_HZ:  Serial.println(F("89.6 Hz")); break;
    case BMP5XX_ODR_80_HZ:    Serial.println(F("80 Hz")); break;
    case BMP5XX_ODR_70_HZ:    Serial.println(F("70 Hz")); break;
    case BMP5XX_ODR_60_HZ:    Serial.println(F("60 Hz")); break;
    case BMP5XX_ODR_50_HZ:    Serial.println(F("50 Hz")); break;
    case BMP5XX_ODR_45_HZ:    Serial.println(F("45 Hz")); break;
    case BMP5XX_ODR_40_HZ:    Serial.println(F("40 Hz")); break;
    case BMP5XX_ODR_35_HZ:    Serial.println(F("35 Hz")); break;
    case BMP5XX_ODR_30_HZ:    Serial.println(F("30 Hz")); break;
    case BMP5XX_ODR_25_HZ:    Serial.println(F("25 Hz")); break;
    case BMP5XX_ODR_20_HZ:    Serial.println(F("20 Hz")); break;
    case BMP5XX_ODR_15_HZ:    Serial.println(F("15 Hz")); break;
    case BMP5XX_ODR_10_HZ:    Serial.println(F("10 Hz")); break;
    case BMP5XX_ODR_05_HZ:    Serial.println(F("5 Hz")); break;
    case BMP5XX_ODR_04_HZ:    Serial.println(F("4 Hz")); break;
    case BMP5XX_ODR_03_HZ:    Serial.println(F("3 Hz")); break;
    case BMP5XX_ODR_02_HZ:    Serial.println(F("2 Hz")); break;
    case BMP5XX_ODR_01_HZ:    Serial.println(F("1 Hz")); break;
    case BMP5XX_ODR_0_5_HZ:   Serial.println(F("0.5 Hz")); break;
    case BMP5XX_ODR_0_250_HZ: Serial.println(F("0.25 Hz")); break;
    case BMP5XX_ODR_0_125_HZ: Serial.println(F("0.125 Hz")); break;
    default: Serial.println(F("Unknown")); break;
  }
  
  // Pretty print power mode inline
  Serial.print(F("Power Mode: "));
  switch(bmp.getPowerMode()) {
    case BMP5XX_POWERMODE_STANDBY:     Serial.println(F("Standby")); break;
    case BMP5XX_POWERMODE_NORMAL:      Serial.println(F("Normal")); break;
    case BMP5XX_POWERMODE_FORCED:      Serial.println(F("Forced")); break;
    case BMP5XX_POWERMODE_CONTINUOUS:  Serial.println(F("Continuous")); break;
    case BMP5XX_POWERMODE_DEEP_STANDBY:Serial.println(F("Deep Standby")); break;
    default: Serial.println(F("Unknown")); break;
  }
  
  Serial.println();
}

void loop() {
  // Check if new data is ready before reading
  if (!bmp.dataReady()) {
    return;
  }

  // Data is ready, perform reading
  if (!bmp.performReading()) {
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
  
  delay(10); // Short delay since we're checking dataReady()
}