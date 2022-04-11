#include "VL53L1X_ULD.h"

/* Defines -------------------------------------------------------*/
#define VL53L1X_GPIO1 24
#define VL53L1X_XSHUT1 25

/* Variables -----------------------------------------------------*/
VL53L1X_ULD sensor;
bool dataReady;

void setup() {
  Serial.begin(115200); // Start the serial port
  Wire.begin(); // Initialize the I2C controller

  // XSHUT pin initialization
  pinMode(VL53L1X_XSHUT1, OUTPUT);
  digitalWrite(VL53L1X_XSHUT1, HIGH);

  // Set the pin to INPUT as it will be used as an interrupt
  // Technically not nedded as all pins will be defined as inputs on reset
  pinMode(VL53L1X_GPIO1, INPUT);

  // Initialize the sensor
  VL53L1_Error status = sensor.Begin();
  if (status != VL53L1_ERROR_NONE) {
    // If the sensor could not be initialized print out the error code. -7 is timeout
    Serial.println("Could not initialize the sensor, error code: " + String(status));
    while (1) {}
  }
  Serial.println("Sensor initialized");

  // It is also possible to change the interrupt polarity of the sensor.
  // By default it is ActiveHIGH which means the pin will go HIGH when a measurement is done
  
  // If you want to reverse the polarity to ActiveLOW uncomment the following line and change the attachInterrupt from RISING to FALLING
  //sensor.SetInterruptPolarity(ActiveLOW); // Possible values are ActiveHIGH and ActiveLOW
  
  // Read out the interrupt polarity
  EInterruptPolarity polarity; // 0 = ActiveLow, 1 = ActiveHigh
  sensor.GetInterruptPolarity(&polarity);
  Serial.println("Interrupt polarity: " + String((uint8_t)polarity));

  attachInterrupt(digitalPinToInterrupt(VL53L1X_GPIO1), ReadData, RISING);

  // Start ranging
  sensor.StartRanging();
}

void loop() {
  if (dataReady) {
    // Get the results
    uint16_t distance;
    sensor.GetDistanceInMm(&distance);

    // After reading the results reset the interrupt to be able to take another measurement
    sensor.ClearInterrupt();
    dataReady = false;

    Serial.println("Distance in mm: " + String(distance));
  }
}

// This function runs everytime the sensor has data available
void ReadData() {
  dataReady = true;
}
