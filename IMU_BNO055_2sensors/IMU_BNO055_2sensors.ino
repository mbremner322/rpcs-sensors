#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno2 = Adafruit_BNO055(56, 0x29);


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(Adafruit_BNO055 b)
{
  sensor_t sensor;
  b.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
    /* Initialise the sensor */
  if(!bno1.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   
  delay(1000);

  /* Use external crystal for better accuracy */
  bno1.setExtCrystalUse(true);
    /* Display some basic information on this sensor */
  displaySensorDetails(bno1);

    
  /* Initialise the sensor */
  if(!bno2.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   
  delay(1000);
    /* Use external crystal for better accuracy */
  bno2.setExtCrystalUse(true);

  /* Display some basic information on this sensor */
  displaySensorDetails(bno2);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  bno1.getEvent(&event);
  Serial.println();Serial.println("sensor=1");
  /* x,y,z coordinates in angles*/
  Serial.print(F("Orientation: x="));
  Serial.print((float)event.orientation.x);
  Serial.print(F(" y="));
  Serial.print((float)event.orientation.y);
  Serial.print(F(" z="));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));


  /* Get a new sensor event */
  bno2.getEvent(&event);
  Serial.println();Serial.println("sensor=2");
  /* The processing sketch expects data as roll, pitch, heading */
  Serial.print(F("Orientation: x="));
  Serial.print((float)event.orientation.x);
  Serial.print(F(" y="));
  Serial.print((float)event.orientation.y);
  Serial.print(F(" z="));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
