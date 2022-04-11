#include "VL53L1X_ULD.h"

/* Defines -------------------------------------------------------*/
#define VL53L1X_GPIO1 24
#define VL53L1X_XSHUT1 25

#define VL53L1X_GPIO2 26
#define VL53L1X_XSHUT2 27

#define VL53L1X_GPIO3 28
#define VL53L1X_XSHUT3 29

/* Variables -----------------------------------------------------*/
VL53L1X_ULD sensor2;
VL53L1X_ULD sensor3;
bool dataReady2;
bool dataReady3;

void setup() {
  Serial.begin(115200); // Start the serial port
  Wire.begin(); // Initialize the I2C controller

  // Sensor n°2 pin init
  pinMode(VL53L1X_GPIO2, INPUT);
  pinMode(VL53L1X_XSHUT2, OUTPUT);
  digitalWrite(VL53L1X_XSHUT2, LOW);
  
  // Sensor n°3 pin init
  pinMode(VL53L1X_GPIO3, INPUT);
  pinMode(VL53L1X_XSHUT3, OUTPUT);
  digitalWrite(VL53L1X_XSHUT3, LOW);

  delay(500);
  digitalWrite(VL53L1X_XSHUT2, HIGH);
 
  // Initialize the sensor
  VL53L1_Error status = sensor2.Begin();
  if (status != VL53L1_ERROR_NONE) {
    // If the sensor could not be initialized print out the error code. -7 is timeout
    Serial.println("Could not initialize the sensor, error code: " + String(status));
    while (1) {}
  }
  Serial.println("Sensor initialized");

  // Read out the interrupt polarity
  EInterruptPolarity polarity2; // 0 = ActiveLow, 1 = ActiveHigh
  sensor2.GetInterruptPolarity(&polarity2);
  Serial.println("Interrupt polarity: " + String((uint8_t)polarity2));

  attachInterrupt(digitalPinToInterrupt(VL53L1X_GPIO2), ReadData2, RISING);

  sensor2.SetI2CAddress(0x60);

  delay(500);
  digitalWrite(VL53L1X_XSHUT3, HIGH);
 
  // Initialize the sensor
  status = sensor3.Begin();
  if (status != VL53L1_ERROR_NONE) {
    // If the sensor could not be initialized print out the error code. -7 is timeout
    Serial.println("Could not initialize the sensor, error code: " + String(status));
    while (1) {}
  }
  Serial.println("Sensor initialized");

  // Read out the interrupt polarity
  EInterruptPolarity polarity3; // 0 = ActiveLow, 1 = ActiveHigh
  sensor3.GetInterruptPolarity(&polarity3);
  Serial.println("Interrupt polarity: " + String((uint8_t)polarity3));

  attachInterrupt(digitalPinToInterrupt(VL53L1X_GPIO3), ReadData3, RISING);

  // Start ranging
  sensor2.StartRanging();
  sensor3.StartRanging();
}

void loop() {
  if (dataReady2) {
    // Get the results
    uint16_t distance2;
    sensor2.GetDistanceInMm(&distance2);

    // After reading the results reset the interrupt to be able to take another measurement
    sensor2.ClearInterrupt();
    dataReady2 = false;

    Serial.println("Distance2 in mm: " + String(distance2));
    delay(500);
  }

  if (dataReady3) {
    // Get the results
    uint16_t distance3;
    sensor3.GetDistanceInMm(&distance3);

    // After reading the results reset the interrupt to be able to take another measurement
    sensor3.ClearInterrupt();
    dataReady3 = false;

    Serial.println("Distance3 in mm: " + String(distance3));
    delay(500);
  }
}

// This function runs everytime the sensor has data available
void ReadData2() {
  dataReady2 = true;
}

void ReadData3() {
  dataReady3 = true;
}
