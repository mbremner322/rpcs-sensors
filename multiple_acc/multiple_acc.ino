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

acc_gyr_t ag_data[NUM_AGS];


#define OUTPUT_READABLE_ACCELGYRO

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(9600);
    while (!Serial);

    // initialize device
    Serial.println("Initializing I2C devices...");
    for (i = 0; i < NUM_AGS; i++){
        digitalWrite(pins[i], LOW);
    
        ags[i].initialize();
        Serial.println("range");
        Serial.println(ags[i].getFullScaleAccelRange());
        Serial.print("pin=");
        Serial.print(pins[i]);
        Serial.println(ags[i].testConnection() ? " MPU6050 connection successful" : " MPU6050 connection failed");

         // results from mpu_zero
        if (i == 0){  
            ags[0].setXAccelOffset(-848);
            ags[0].setYAccelOffset(-35);
            ags[0].setZAccelOffset(-1906);
        }
        if (i == 1){
             ags[1].setXAccelOffset(-847);
             ags[1].setYAccelOffset(-35);
             ags[1].setZAccelOffset(1866);

        }
        if (i == 2){    
            ags[2].setXAccelOffset(-847);
            ags[2].setYAccelOffset(-33);
            ags[2].setZAccelOffset(1872);
        }
        if (i == 3){    
            ags[3].setXAccelOffset(-848);
            ags[3].setYAccelOffset(-34);
            ags[3].setZAccelOffset(1866);
        }

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
