#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)


// Create LSM9DS0 board instance.
Adafruit_LSM9DS0     lsm(1000);  // Use I2C, ID #1000
Adafruit_BME280 bme;

// Headers of modules
int safeToFire(float height[20], float yaw[20], float pitch[20], int timeSinceLaunch);
float getAverage (float arr[20]); 
float getStD(float arr[20], float average);
void configureLSM9DS0(void);


// Create simple AHRS algorithm using the LSM9DS0 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());


// Function to configure the sensors on the LSM9DS0 board.
// You don't need to change anything here, but have the option to select different
// range and gain values.
void configureLSM9DS0(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}


void setup() {
  Serial.begin(115200);
  Serial.println(F("Adafruit LSM9DS0 9 DOF Board AHRS Example")); Serial.println("");
  
  // Initialize the LSM9DS0 board.
  if(!lsm.begin()) {
    // There was a problem detecting the LSM9DS0 ... check your connections
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  
  unsigned status;
  status = bme.begin();  
  
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    while (1);
  }
    
  // Setup the sensor gain and integration time.
  configureLSM9DS0();
}


// initializes arrays of size 20 to all be 0
float height[20] = { }; 
float pitch[20] = { };
float yaw[20] = { };

int count = 0;
int safe = 0;

signed long timeLaunched = -1;
signed long timeSinceLaunch = -1;


void loop() {

  int launchedInput = 0; // get input from someone

  sensors_vec_t orientation;

  // Use the simple AHRS function to get the current orientation.
  ahrs.getOrientation(&orientation);
  
  // set to input from sensors
  float heightIn = bme.readAltitude(SEALEVELPRESSURE_HPA); // inputHeight;
  float yawIn = orientation.roll;
  float pitchIn = orientation.pitch; //inputPitch;

  if (launchedInput) {
    if (timeLaunched < 0) {
      timeLaunched = millis();
    }
    timeSinceLaunch = millis() - timeLaunched;
  }

  // Fills the whole array with the first value, then the whole array (minus the index zero) with the second value, and so on until it is full;
  // did this to greatly reduce redundancy and make the average a functional for the first 19 clocks cycles
  if (count < 20) {
    for (int i = 0; i < 20; i++) {
      if (height[i] == 0) {
        height[i] = heightIn;
      }
      if (yaw[i] == 0) {
        yaw[i] = yawIn;
      }
      if (pitch[i] == 0) {
        pitch[i] = pitchIn;
      }
      
    }
  } else {
      int place = count % 20;  // loops through 0-19, changing the value in that index of each array
      height[place] = heightIn;
      yaw[place] = yawIn;
      pitch[place] = pitchIn;
   }

   count++; // increments the counter to loop through the array

   safe = safeToFire(height, yaw, pitch, timeSinceLaunch);
   digitalWrite(LED_BUILTIN, safe);
   
}

// takes 20 data points, takes the average of them, and then asks if based on the average it is safe to fire the plasma
int safeToFire(float height[20], float yaw[20], float pitch[20], int timeSinceLaunch) {
  int isSafe = 0;
  
  float heightAvg = getAverage(height);
  float yawAvg = getAverage(yaw);
  float pitchAvg = getAverage(pitch);
  
  float heightStD = getStD(height, heightAvg); 
  float yawStD = getStD(yaw, yawAvg); 
  float pitchStD = getStD(pitch, pitchAvg); 

  if (timeSinceLaunch > 10000) {   // 10000 ms = 10 seconds
    if (heightStD < 150 && yawStD < 10 && pitchStD < 10) {                                    // adjust when we look at actual data to reflect a data point that seems unreasonable
      if (heightAvg > 11 && yawAvg > -30 && yawAvg < 30 && pitchAvg > -30 && pitchAvg < 30) { // made up conditions, determine actual
        isSafe = 1;
      }
    }
  }
  
 Serial.print(heightStD); ; Serial.print("   "); Serial.print(yawStD); Serial.print("   "); Serial.print(pitchStD); Serial.print("   "); 
 Serial.print(heightAvg); Serial.print("   "); Serial.print(yawAvg); Serial.print("   "); Serial.print(pitchAvg); Serial.print("   ");  
 Serial.print(isSafe); Serial.println("   ");
  
  return isSafe;
}

// takes and returns the average of every index in the array
float getAverage (float arr[20]) {
  float sum = 0.0;
  for (int i = 0; i < 20; i++) {
    sum += arr[i];
  }
  float average = sum / 20;
  return average;
}

// calculates the standard deviation given an array and its average
float getStD(float arr[20], float average) {
  float StD = 0;
  for (int i = 0; i < 20; i++) {
    StD += sq(arr[i] - average);
  }
  return sqrt(StD / 20);
}

//float withinCone (yawAvg, pitchAvg) {
//  
//
//  
//}
