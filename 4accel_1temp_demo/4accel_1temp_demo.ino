#include "I2Cdev.h"
#include "MPU6050.h"
#include "Adafruit_HTU21DF.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/**################# MACROS ####################**/
#define COMPUTE_CALIBRATION 0
#define PRINT_DATA_TO_CONSOLE 1
#define NUM_AGS 4
#define INACTIVE_PIN_NO -1


/**################ TYPES ######################**/
typedef struct{
    int16_t ax, ay, az, gx, gy, gz;
} ag_data_t;

typedef struct{
    int16_t ax, ay, az, gx, gy, gz;
} calibration_t;


/**############### GLOBALS #####################**/
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

MPU6050 accelgyro1, accelgyro2, accelgyro3, accelgyro4;
MPU6050 ags[NUM_AGS] = {accelgyro1, accelgyro2, accelgyro3, accelgyro4};
int pins[NUM_AGS] = {10, 11, 5, 6};


void initialize_calibration_offsets(calibration_t *calibrations){
  #if COMPUTE_CALIBRATION == 1
      
  
  #elif COMPUTE_CALIBRATION == 0
      // precomputed values from running mcu_zero
      // sensor 1
      calibrations[0].ax = -848;
      calibrations[0].ay = -35;
      calibrations[0].az = -1906;
      calibrations[0].gx = -64;
      calibrations[0].gy = -1;
      calibrations[0].gz = -160;

      // sensor 2
      calibrations[1].ax = -847;
      calibrations[1].ay = -35;
      calibrations[1].az = -1866;
      calibrations[1].gx = -65;
      calibrations[1].gy = -1;
      calibrations[1].gz = -161;

      // sensor 3
      calibrations[2].ax = -847;
      calibrations[2].ay = -33;
      calibrations[2].az = -1872;
      calibrations[2].gx = -66;
      calibrations[2].gy = -1;
      calibrations[2].gz = -161;

      // sensor 4
      calibrations[3].ax = -848;
      calibrations[3].ay = -34;
      calibrations[3].az = -1866;
      calibrations[3].gx = -54;
      calibrations[3].gy = -2;
      calibrations[3].gz = -161;
  #endif
}


void set_calibration_offsets(int i, calibration_t c){
    ags[i].setXAccelOffset(c.ax);
    ags[i].setYAccelOffset(c.ay);
    ags[i].setZAccelOffset(c.az);
    
    ags[i].setXGyroOffset(c.gx);
    ags[i].setYGyroOffset(c.gy);
    ags[i].setZGyroOffset(c.gz);
}



void setup() {
    int i;
    bool connection;
    char buf[200];
    calibration_t calibrations[NUM_AGS];
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(9600);
    while (!Serial);

    initialize_calibration_offsets(calibrations);

    // set pins to OUTPUT mode and set them to default HIGH
    for (i = 0; i < NUM_AGS; i++){
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], HIGH);
    }

    // initialize device
    Serial.println("Initializing I2C devices...");
    for (i = 0; i < NUM_AGS; i++){
        // select this accelerometer to have slave address 0x68
        digitalWrite(pins[i], LOW);

        ags[i].initialize();
        connection = ags[i].testConnection();
        sprintf(buf, "Sensor %d connected to pin %d %s", i, pins[i], 
          connection ? 
            " MPU6050 connection successful" : " MPU6050 connection failed");
        Serial.print(buf);

        // mark that this accelerometer is not connected
        if (!connection){
           pins[i] = INACTIVE_PIN_NO;
        }
       
        set_calibration_offsets(i, calibrations[i]);

        // deselect this accelerometer to have slave address 0x68 (back to 0x69)
        digitalWrite(pins[i], HIGH);
    }
}


void loop() {
    int i;
    float temp, hum;
    ag_data_t ag_data[NUM_AGS];
    char buf[500];
    
    for (i = 0; i < NUM_AGS; i++){
        // ignore inactive accelerometer
        if (pins[i] == INACTIVE_PIN_NO) continue;

        // select this accelerometer to have slave address 0x68
        digitalWrite(pins[i], LOW);
        ags[i].getMotion6(&ag_data[i].ax, &ag_data[i].ay, &ag_data[i].az,
                                &ag_data[i].gx, &ag_data[i].gy, &ag_data[i].gz); 
        // deselect this accelerometer to have slave address 0x68 (back to 0x69)
        digitalWrite(pins[i], HIGH);

        // PRINT
        sprintf(buf, "ACCELEROMETER %d : x=%d, y=%d, z=%d\n", i, ag_data[i].ax, ag_data[i].ay, ag_data[i].az);
        Serial.println(buf);
        
    }

    temp = htu.readTemperature();
    hum = htu.readHumidity();

    sprintf(buf, "TEMPERATURE : temp=%f, hum=%f\n", temp, hum);
    Serial.println(buf);

    // send over bluetooth
    
    delay(250);
}
