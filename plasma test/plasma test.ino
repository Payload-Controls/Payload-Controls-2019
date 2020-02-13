#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 adc1(0x48);

void setup() {
  Serial.begin(9600);
  adc1.begin();
  adc1.setGain(GAIN_SIXTEEN);
  Serial.println("Sensors Initialized");

}

void loop() {
  int16_t d = adc1.readADC_Differential_0_1();
  Serial.println(d);
  delay(100);

}
