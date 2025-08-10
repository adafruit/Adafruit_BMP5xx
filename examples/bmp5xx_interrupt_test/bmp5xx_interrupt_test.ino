/*!
 * @file bmp5xx_interrupt_test.ino
 *
 * This is a simple test sketch for the BMP5xx interrupt functionality.
 * It demonstrates data ready interrupts and shows how to monitor the interrupt pin.
 * Connect the BMP5xx INT pin to the specified Arduino interrupt pin.
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

#define BMP5XX_IRQ_PIN 2  // Interrupt pin (change this if using a different pin)

Adafruit_BMP5xx bmp; // Create BMP5xx object

// Interrupt flag - set by interrupt handler
volatile bool dataReady = false;

// Interrupt service routine
void bmp5xxInterruptHandler() {
  dataReady = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // Wait for Serial Monitor to open
  
  Serial.println(F("Adafruit BMP5xx Interrupt Test"));
  Serial.print(F("Using interrupt pin: "));
  Serial.println(BMP5XX_IRQ_PIN);
  Serial.println();

  // Configure interrupt pin as input
  pinMode(BMP5XX_IRQ_PIN, INPUT);
  
  // Try to initialize the sensor using I2C
  if (!bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
    Serial.println(F("Could not find a valid BMP5xx sensor, check wiring!"));
    while (1) delay(10);
  }

  Serial.println(F("BMP5xx found!"));

  // Configure sensor settings for interrupt testing
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_1X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_1X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_BYPASS);
  bmp.setOutputDataRate(BMP5XX_ODR_01_HZ);  // 1 Hz for clear interrupt timing
  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);

  // Configure interrupt: pulsed, active high, push-pull, data ready source
  Serial.println(F("Configuring data ready interrupt..."));
  if (!bmp.configureInterrupt(BMP5XX_INTERRUPT_PULSED, 
                              BMP5XX_INTERRUPT_ACTIVE_HIGH, 
                              BMP5XX_INTERRUPT_PUSH_PULL, 
                              BMP5XX_INTERRUPT_DATA_READY, 
                              true)) {
    Serial.println(F("Failed to configure interrupt!"));
    while (1) delay(10);
  }

  // Attach interrupt handler
  attachInterrupt(digitalPinToInterrupt(BMP5XX_IRQ_PIN), bmp5xxInterruptHandler, RISING);
  
  Serial.println(F("Interrupt configured successfully!"));
  Serial.println(F("Waiting for interrupts..."));
  Serial.println();

}

void loop() {
  // Check if interrupt occurred
  if (dataReady) {
    // Clear the interrupt flag
    dataReady = false;
    
    // Check if data is actually ready using library function
    if (bmp.dataReady()) {
      // Read the sensor data
      if (bmp.performReading()) {
        Serial.print(F("  Temperature: "));
        Serial.print(bmp.temperature);
        Serial.println(F(" °C"));
        
        Serial.print(F("  Pressure: "));
        Serial.print(bmp.pressure);
        Serial.println(F(" hPa"));
        
        Serial.print(F("  Pin state after read: "));
        Serial.println(digitalRead(BMP5XX_IRQ_PIN) ? F("HIGH") : F("LOW"));
      } else {
        Serial.println(F("  Failed to read sensor data"));
      }
    } else {
      Serial.println(F("Data not ready (false interrupt?)"));
    }
    
    Serial.println(F("---"));
  }
  
  // Small delay to prevent overwhelming the serial output
  delay(10);
}