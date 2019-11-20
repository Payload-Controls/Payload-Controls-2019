#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

#define CHIP_SELECT 4
#define WRITE true //CHANGE BEFORE FLIGHT

int fileCount = 0;
unsigned long fileCountTimer = 0;
unsigned long frequency = 30000; //new file every 30 seconds (30,000ms)
String inputData = "";

//ADC data
int16_t S1HF, S2HF;

Adafruit_ADS1115 adc1(0x4B);

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
#ifndef ESP8266
  while (!Serial);     // For running on ESP boards. Will pause until serial console opens
#endif
  Serial.begin(115200);
  Serial.print("Initializing SD card...");  // see if the card is present and can be initialized:
  Serial1.begin(57600);
  if (!SD.begin(CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    while (1 && WRITE);
  }
  Serial.println("card initialized.");
  initSensors();
}

void loop() {
  if (Serial1.available()) {
    int inChar = Serial1.read();
    if (inChar != '\n') {
      inputData += (char)inChar;
    }else{
      Serial.print(inputData);
      inputData = "";
    }
  }
  readSensors();
  //printADCs();
  String dataString = formatData();
  if (WRITE) {
    if (millis() - fileCountTimer >= frequency) {
      fileCountTimer = millis();
      fileCount++;
    }
    File dataFile = SD.open(fileName(), FILE_WRITE);
    if (dataFile) { // if the file is available, write to it:
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      Serial.println(dataString);
    } else {
      Serial.println("error opening " + fileName());
    }
  }
  //Serial.println(dataString);
}

String fileName() {
  return "log" + String(fileCount) + ".csv";
}

String formatData() {
  String ADCs = String(S1HF) + "," + String(S2HF) + ",";
  return ADCs;// + inputString;
}

void initSensors() {
  adc1.begin();
  adc1.setGain(GAIN_SIXTEEN);
  Serial.println("Sensors Initialized");
}

void readSensors() {
  //Read ADCs
  S1HF = adc1.readADC_Differential_0_1();

  S2HF = adc1.readADC_Differential_2_3();
}

void printADCs() {
  Serial.print(F("Sensor 1 Heat Flux:"));
  Serial.println(S1HF);
  Serial.print("Sensor 2 Heat Flux:");
  Serial.println(S2HF);
}
