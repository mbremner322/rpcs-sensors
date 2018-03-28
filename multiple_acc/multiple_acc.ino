#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define NUM_AGS 4
MPU6050 accelgyro1, accelgyro2, accelgyro3, accelgyro4;
MPU6050 ags[4] = {accelgyro1, accelgyro2, accelgyro3, accelgyro4};
int pins[4] = {5, 6, 10, 11};
int i;

typedef struct{
    int16_t ax, ay, az, gx, gy, gz;
} acc_gyr_t;

typedef struct{
    int16_t ax, ay, az, gx, gy, gz;
} calibration_t;

calibration_t calibrations[NUM_AGS];
acc_gyr_t ag_data[NUM_AGS];


#define OUTPUT_READABLE_ACCELGYRO
#define COMPUTE_CALIBRATION 0


void initialize_calibration_offsets(){
  #if COMPUTE_CALIBRATION == 1
      
  #elif COMPUTE_CALIBRATION == 0
      // sensor 1
      calibrations[0].ax = -848;
      calibrations[0].ay = -35;
      calibrations[0].az = -1906;

      // sensor 2
      calibrations[1].ax = -847;
      calibrations[1].ay = -35;
      calibrations[1].az = -1866;

      // sensor 3
      calibrations[2].ax = -847;
      calibrations[2].ay = -33;
      calibrations[2].az = -1872;

      // sensor 4
      calibrations[3].ax = -848;
      calibrations[3].ay = -34;
      calibrations[3].az = -1866;
  #endif
}


void set_calibration_offsets(int i){
    ags[i].setXAccelOffset(calibrations[i].ax);
    ags[i].setYAccelOffset(calibrations[i].ay);
    ags[i].setZAccelOffset(calibrations[i].az);
}


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(9600);
    while (!Serial);

    initialize_calibration_offsets();

    // initialize device
    Serial.println("Initializing I2C devices...");
    for (i = 0; i < NUM_AGS; i++){
        digitalWrite(pins[i], LOW);

        char buf[100];
        sprintf(buf, "Initializing sensor %d connected to pin %d", i, pins[i]);
        Serial.println(buf);
        ags[i].initialize();
        Serial.println(ags[i].testConnection() ? " MPU6050 connection successful" : " MPU6050 connection failed");

        set_calibration_offsets(i);
        
        digitalWrite(pins[i], HIGH);
    }
    delay(500); 
}


void loop() {
    int16_t x,y,z, test;
    for (i = 0; i < NUM_AGS; i++){
        digitalWrite(pins[i], LOW);
        ags[i].getMotion6(&x, &y, &z, &test, &test, &test); 
        digitalWrite(pins[i], HIGH);

        Serial.print("Sensor");
        Serial.println(i);
        Serial.print("x=");Serial.print(x);Serial.print("\t");
        Serial.print("y=");Serial.print(y);Serial.print("\t");
        Serial.print("z=");Serial.print(z);Serial.print("\t");
        Serial.println();
    }
    

    delay(250);
}
