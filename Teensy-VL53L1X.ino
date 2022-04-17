/*
 * @file    Teensy-VL53L1X
 * @author  Arnaud C.
 * 
 */

#include <stdint.h>
#include "VL53L1X_ULD.h" // VL53L1X library created by Ruben Neurink-Sluiman

/* Defines -------------------------------------------------------*/
#define VL53L1X_MAX_NB  16
#define VL53L1X_DEFAULT_ADDRESS (0x29 << 1)

#define NB_SENSORS 3  // user defined

/* Typedef -------------------------------------------------------*/

/**
 * vl53l1x configuration
 */
typedef struct vl53l1x{
  VL53L1X_ULD sensor;
  uint16_t i2c_address;
  uint8_t gpio;
  uint8_t xshut;
  uint16_t distance;

} tof_t;

/**
 * interrupt routine references
 */
typedef struct vl53l1x_routines{
  void (*routine0) ();
  void (*routine1) ();
  void (*routine2) ();
  void (*routine3) ();
  void (*routine4) ();
  void (*routine5) ();
  void (*routine6) ();
  void (*routine7) ();
  void (*routine8) ();
  void (*routine9) ();
  void (*routine10) ();
  void (*routine11) ();
  void (*routine12) ();
  void (*routine13) ();
  void (*routine14) ();
  void (*routine15) ();
  
} tof_routines_t;

/* Prototypes ----------------------------------------------------*/
void tof_addSensorConfig(uint8_t index, uint8_t gpio, uint8_t xshut);
int tof_init(tof_t *tof, uint8_t nb_sensor);

void _readData0();
void _readData1();
void _readData2();

/* Variables -----------------------------------------------------*/
tof_t tof[VL53L1X_MAX_NB];
bool dataReady[VL53L1X_MAX_NB];

/* Constantes ----------------------------------------------------*/
const tof_routines_t tof_routines = {
  _readData0,
  _readData1,
  _readData2,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL
};

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Sensor configurations
  tof_addSensorConfig(0, 26, 27);
  tof_addSensorConfig(1, 28, 29);
  tof_addSensorConfig(2, 24, 25);

  // Sensors initialization
  tof_init(tof, NB_SENSORS);
}

void loop() 
{
  for(int i=0; i<NB_SENSORS; i++)
  {
    if (dataReady[i]) 
    {
      // Get the results
      tof[i].sensor.GetDistanceInMm(&tof[i].distance);
  
      // Clear interrupt
      tof[i].sensor.ClearInterrupt();
      dataReady[i] = false;
    }
  }

  for(int i=0; i<NB_SENSORS; i++)
  {
     Serial.print("Dist[" + String(i) + "]: " + String(tof[i].distance) + "   ");
  }
  Serial.println("");
  delay(100);
}


/* Functions -----------------------------------------------------*/

/**
 * tof_addSensorConfig
 * @param       index
 * @param gpio  pin number
 * @param xshut pin number
 */
void tof_addSensorConfig(uint8_t index, uint8_t gpio, uint8_t xshut){
  tof[index].gpio = gpio;
  tof[index].xshut = xshut;
}

/**
 * tof_init
 * @param tof       vl53l1x configuration array
 * @param nb_sensor 
 * @return status
 */
int tof_init(tof_t *tof, uint8_t nb_sensor)
{
  VL53L1_Error status;
  
  // Init pins
  for(int i=0; i<nb_sensor; i++){
    pinMode(tof[i].gpio, INPUT);
    pinMode(tof[i].xshut, OUTPUT);
    digitalWrite(tof[i].xshut, LOW);
  }

  delay(500);

  // Init sensors
  for(int i=0; i<nb_sensor; i++)
  {
    // Xshut to high state
    digitalWrite(tof[i].xshut, HIGH);

    // Sensor begin
    status = tof[i].sensor.Begin();
    if (status != VL53L1_ERROR_NONE) {
      Serial.println("Could not initialize the sensor, error code: " + String(status));
      while (1) {}
    }
    Serial.println("Sensor initialized");

    // If we use interruptions
    switch(i)
    {
      case 0: 
        attachInterrupt(digitalPinToInterrupt(tof[i].gpio), tof_routines.routine0, RISING);
        break;
      case 1: 
        attachInterrupt(digitalPinToInterrupt(tof[i].gpio), tof_routines.routine1, RISING);
        break;
      case 2: 
        attachInterrupt(digitalPinToInterrupt(tof[i].gpio), tof_routines.routine2, RISING);
        break;
      case 3: 
        attachInterrupt(digitalPinToInterrupt(tof[i].gpio), tof_routines.routine3, RISING);
        break;
      case 4: 
        attachInterrupt(digitalPinToInterrupt(tof[i].gpio), tof_routines.routine4, RISING);
        break;
      case 5: 
        attachInterrupt(digitalPinToInterrupt(tof[i].gpio), tof_routines.routine5, RISING);
        break;    
      case 6: 
        attachInterrupt(digitalPinToInterrupt(tof[i].gpio), tof_routines.routine6, RISING);
        break;    
      case 7: 
        attachInterrupt(digitalPinToInterrupt(tof[i].gpio), tof_routines.routine7, RISING);
        break;    
      case 8: 
        attachInterrupt(digitalPinToInterrupt(tof[i].gpio), tof_routines.routine8, RISING);
        break;    
      case 9: 
        attachInterrupt(digitalPinToInterrupt(tof[i].gpio), tof_routines.routine9, RISING);
        break;   
      case 10: 
        attachInterrupt(digitalPinToInterrupt(tof[i].gpio), tof_routines.routine10, RISING);
        break; 
      case 11: 
        attachInterrupt(digitalPinToInterrupt(tof[i].gpio), tof_routines.routine11, RISING);
        break; 
      case 12: 
        attachInterrupt(digitalPinToInterrupt(tof[i].gpio), tof_routines.routine12, RISING);
        break; 
      case 13: 
        attachInterrupt(digitalPinToInterrupt(tof[i].gpio), tof_routines.routine13, RISING);
        break; 
      case 14: 
        attachInterrupt(digitalPinToInterrupt(tof[i].gpio), tof_routines.routine14, RISING);
        break; 
      case 15: 
        attachInterrupt(digitalPinToInterrupt(tof[i].gpio), tof_routines.routine15, RISING);
        break;                         
    }


    // New i2c address
    tof[i].sensor.SetI2CAddress(0x60 + i * 2);
    tof[i].i2c_address = 0x60 + i * 2;

    delay(250);
  }

  // Start sensors
  for(int i=0; i<nb_sensor; i++)
  {
    tof[i].sensor.StartRanging();
  }

  return status;
}


void _readData0(){
  dataReady[0] = true;
}

void _readData1() {
  dataReady[1] = true;
}

void _readData2() {
  dataReady[2] = true;
}
