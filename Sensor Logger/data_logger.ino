#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_MPRLS.h>
#include <Adafruit_BME280.h>

#define CHIP_SELECT 4
#define WRITE true //CHANGE BEFORE FLIGHT
#define TCAADDR 0x70

int fileCount = 0;
unsigned long fileCountTimer = 0;
unsigned long frequency = 30000; //new file every 30 seconds (30,000ms)

int16_t S1HF, S2HF, COIL;

float PRESSURE1, PRESSURE2, TEMP;

Adafruit_ADS1115 adcHF(0x4B);
Adafruit_ADS1115 adcRogowski(0x48);

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr1 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_MPRLS mpr2 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

Adafruit_BME280 bme;

// to choose which mpr sensor to pull from
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(115200);
  Serial.print("Initializing SD card...");  // see if the card is present and can be initialized:
  if (!SD.begin(CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    while (1 && WRITE);
  }
  writeToFile("Time (ms),HF Sensor 1,HF Sensor 2,Rogowski coil,Pressure 1 (hPa),Pressure 2 (hPa),Temp (C)");
  Serial.println("card initialized.");
  initSensors();
}

void loop() {
  readSensors();
  String dataString = formatData();
  writeToFile(dataString);
}

void writeToFile(String dataString) {
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
}

String fileName() {
  return "log" + String(fileCount) + ".csv";
}

String formatData() {
  String data = String(millis()) + "," + String(S1HF) + "," + 
                String(S2HF) + "," + String(COIL) + "," + 
                String(PRESSURE1) + "," + String(PRESSURE2) + "," +
                String(TEMP);
  return data;
}

void initSensors() {
  adcHF.begin();
  adcHF.setGain(GAIN_SIXTEEN);

  adcRogowski.begin();
  adcRogowski.setGain(GAIN_SIXTEEN);

  tcaselect(0);
  mpr1.begin();
  tcaselect(1);
  mpr2.begin();

  bme.begin();
  
  Serial.println("Sensors Initialized");
}

void readSensors() {
  //Read HF sensors
  S1HF = adcHF.readADC_Differential_0_1();
  S2HF = adcHF.readADC_Differential_2_3();

  // Read Rogowski coil
  COIL = adcRogowski.readADC_Differential_0_1();

  // Read Pressure Sensors
  tcaselect(0);
  PRESSURE1 = mpr1.readPressure();
  tcaselect(1);
  PRESSURE2 = mpr2.readPressure();

  // Read temp
  TEMP = bme.readTemperature();
}
