/** HELPFUL LINKS FOR THIS TO WORK
 *  
 *  TEENSY 3.5 BOARD SETUP:
 *     https://learn.adafruit.com/add-boards-arduino-v164/setup
 *     https://learn.adafruit.com/adafruit-feather-m0-basic-proto/using-with-arduino-ide
 *  LIBRARIES NEEDED:
 *     https://github.com/adafruit/Adafruit_BluefruitLE_nRF51
 *     https://github.com/adafruit/Adafruit_BNO055
 *     https://github.com/bblanchon/ArduinoJson
 */
 
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_HTU21DF.h"
#include "BluefruitConfig.h"
#include <Adafruit_BNO055.h>
#include <ArduinoJson.h>

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*================DEBUG FLAGS================*/
#define DEBUG_MODE 1 //allows for more print statements if set to 1


/*==============GLOBALS===================*/
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno2 = Adafruit_BNO055(56, 0x29);
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

const int BAUDRATE = 115200;
const int BLE_BUFFER_SIZE = 20;
const int PRESSURE_SIZE = 105;
int pressure_map_arr[PRESSURE_SIZE];
char pressure_char_arr[PRESSURE_SIZE];
int pressure_offsets[PRESSURE_SIZE];
const float UPPER_IMU_THRESH = 90.0;
const float LOWER_IMU_THRESH = -90.0;


/** PRESSURE MAP GLOBALS AND DESCRIPTION 
 * 7x15 pressure matrix test
 * 
 * This test verifies that multiplexers can be used to read a single pressure map point
 * 
 * Two multiplexers (MAX4617CPE) are connected to the feather M0 board. 
 *  1) Mut ROW: Pin X is connected to analog input with a 10k pull up, 
 *              Pins A, B, C are connected to digital inputs, 
 *              Pins X0-X7 are connected to the rows of the sensor
 *  2) Mut COL1: Pin X is connected to gnd,
 *               Pins A, B, C are connected to digital inputs
 *               Pins X0-X7 are connected to the first 8 columns of the sensor
 *  3) Mut COL2: Pin X is connected to gnd,
 *               Pins A, B, C are connected to digital inputs
 *               Pins X0-X7 are connected to the last 7 columns of the sensor
 */
const int MUX_COL_A_PIN = 17;
const int MUX_COL_B_PIN = 22;
const int MUX_COL_C_PIN = 23;

const int MUX_ROW1_A_PIN = 33;
const int MUX_ROW1_B_PIN = 34;
const int MUX_ROW1_C_PIN = 35;
const int MUX_ROW2_A_PIN = 37;
const int MUX_ROW2_B_PIN = 38;
const int MUX_ROW2_C_PIN = 39;

const int MUX_ROW1_EN_PIN = 36;
const int MUX_ROW2_EN_PIN = 16;

const int ANA_PIN = 14; // The pin that the first analog mutex is connected to

// The number of rows in the matrix
const int NUM_ROWS = 15;
// The number of columns in the matrix
const int NUM_COLS = 7;

// These macros compute whether the A, B, or C outputs to the lower order muxes
// should be high or low based on the desired mux selection
#define GET_A_OUTPUT(x)  ((x) & 0x1)
#define GET_B_OUTPUT(x)  (((x) >> 1) & 0x1)
#define GET_C_OUTPUT(x)  (((x) >> 2) & 0x1)
// The enable pin is active low, and the chip should be enabled if there is a zero
// in the 2^3 spot
#define GET_EN1_OUTPUT(x) (!!((x) & 0x8))
// The enable pin is active low, and the chip should be enabled if there is a one
// in the 2^3 spot
#define GET_EN2_OUTPUT(x) (!((x) & 0x8))
/*==============GLOBALS===================*/

void setup(void)
{
  Serial.begin(BAUDRATE);
  
  Serial.println(F("Sensor Team IMU->BLE"));
  Serial.println(F("------------------------------------------------"));
  
  imu_init();
  pressure_map_init();
  for (int i = 0; i < PRESSURE_SIZE; i++){
    pressure_offsets[i] = 1000;
  }
  // at this point nothing should be touching the pressure map
  get_pressure_array(pressure_offsets, false);
  delay(500);
  get_pressure_array(pressure_offsets, false);
  delay(500);
  get_pressure_array(pressure_offsets, false);
  ble_init();
}


void loop(void)
{
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  char json_buf[1000];
  int buf_len;
  unsigned long tick, tock;

  tick = millis();
  
  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(100);
  }

  /* signal the beginning of this packet and flush the pipeline */
  ble.print("*");
  ble.flush();

  /* get pressure points and map to a string */
  get_pressure_array(pressure_map_arr, true);
  int_array_to_string(pressure_map_arr, pressure_char_arr);

  /* construct json object */
  root["a"] = round(get_imu_angle());
  root["t"] = round((9 * htu.readTemperature() / 5) + 32);
  root["h"] = round(htu.readHumidity());
  root["p"] = pressure_char_arr;

  /* stringify json */
  root.printTo(json_buf);
  buf_len = strlen(json_buf);
  
#if DEBUG_MODE
  Serial.println(json_buf);
#endif

  /* break up string into chunks of BLE_BUFFER_SIZE and send individually */
  for (int i = 0; i < buf_len; i += BLE_BUFFER_SIZE){
      char temp_buf[BLE_BUFFER_SIZE + 1];
      
      memcpy(temp_buf, json_buf+i, BLE_BUFFER_SIZE);
      temp_buf[BLE_BUFFER_SIZE] = '\0';
      /* Send input data to host via Bluefruit */
      ble.print(temp_buf);
      delay(200);
  }

 #if DEBUG_MODE
   tock = millis();
   Serial.print(tock - tick);
   Serial.println(" milliseconds");
   Serial.println();
 #endif
}


/** =========================== HELPER FUNCTIONS ============================ **/

void pressure_map_init(){
  // Initialize all pins
  pinMode(ANA_PIN, INPUT);
  
  pinMode(MUX_COL_A_PIN, OUTPUT);
  pinMode(MUX_COL_B_PIN, OUTPUT);
  pinMode(MUX_COL_C_PIN, OUTPUT);
  
  pinMode(MUX_ROW1_A_PIN, OUTPUT);
  pinMode(MUX_ROW1_B_PIN, OUTPUT);
  pinMode(MUX_ROW1_C_PIN, OUTPUT);
  pinMode(MUX_ROW2_A_PIN, OUTPUT);
  pinMode(MUX_ROW2_B_PIN, OUTPUT);
  pinMode(MUX_ROW2_C_PIN, OUTPUT);

  pinMode(MUX_ROW1_EN_PIN, OUTPUT);
  pinMode(MUX_ROW2_EN_PIN, OUTPUT);
  
  // Make sure pin A inputs to both muxes are low so the sensor starts disconnected
  digitalWrite(MUX_COL_A_PIN, LOW);
  digitalWrite(MUX_COL_B_PIN, LOW);
  digitalWrite(MUX_COL_C_PIN, LOW);
  
  digitalWrite(MUX_ROW1_A_PIN, LOW);
  digitalWrite(MUX_ROW1_B_PIN, LOW);
  digitalWrite(MUX_ROW1_C_PIN, LOW);
  digitalWrite(MUX_ROW2_A_PIN, LOW);
  digitalWrite(MUX_ROW2_B_PIN, LOW);
  digitalWrite(MUX_ROW2_C_PIN, LOW);

  digitalWrite(MUX_ROW1_EN_PIN, LOW);
  digitalWrite(MUX_ROW2_EN_PIN, LOW);
}

void ble_init(){
  Serial.println(F("******************************"));
  
  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
        Serial.println("Factory Reset Failed");
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
  
  Serial.println(F("******************************"));
}

void imu_init(){
  /* Initialise the sensor */
  if(!bno1.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, failed to connect bno1 ... Check your wiring or I2C ADDR!");
  }

  /* Initialise the sensor */
  if(!bno2.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, failed to connect bno2 ... Check your wiring or I2C ADDR!");
  }

  delay(1000);
  
  bno1.setExtCrystalUse(true);
  bno2.setExtCrystalUse(true);

  displaySensorDetails(bno1);
  displaySensorDetails(bno2);
}

/** arithmetic to shift the z-orientation value to [-90, 90] **/
float get_imu_angle(){
  sensors_event_t event1, event2;
  float average, shifted;
  
  bno1.getEvent(&event1);
  bno2.getEvent(&event2);

  average = (event1.orientation.z + event2.orientation.z) /2;
  if (average < -90.0){
    shifted = -average - 270;
  }
  else{
    shifted = 90.0 - average;
  }

  return constrain(shifted, LOWER_IMU_THRESH, UPPER_IMU_THRESH);
}

/* generate 105 points between 0 and 50 using the 8x8 map */
void get_pressure_array(int *A, bool scale){
    int reading, index;
    
    for (int c = 0; c < NUM_COLS; c++) {
      digitalWrite(MUX_COL_A_PIN, GET_A_OUTPUT(c));
      digitalWrite(MUX_COL_B_PIN, GET_B_OUTPUT(c));
      digitalWrite(MUX_COL_C_PIN, GET_C_OUTPUT(c));
        
      for (int r = 0; r < NUM_ROWS; r++) {
        digitalWrite(MUX_ROW1_A_PIN, GET_A_OUTPUT(r));
        digitalWrite(MUX_ROW1_B_PIN, GET_B_OUTPUT(r));
        digitalWrite(MUX_ROW1_C_PIN, GET_C_OUTPUT(r));
        digitalWrite(MUX_ROW2_A_PIN, GET_A_OUTPUT(r));
        digitalWrite(MUX_ROW2_B_PIN, GET_B_OUTPUT(r));
        digitalWrite(MUX_ROW2_C_PIN, GET_C_OUTPUT(r));

        digitalWrite(MUX_ROW1_EN_PIN, GET_EN1_OUTPUT(r));
        digitalWrite(MUX_ROW2_EN_PIN, GET_EN2_OUTPUT(r));

        index = c * NUM_ROWS + r;

        reading = analogRead(ANA_PIN);
        if (scale){
          reading = reading - pressure_offsets[index];
          Serial.print("O = ");Serial.print(reading);Serial.print(" R =");
          reading = map(reading, -40, 10, 50, 0);
          reading = constrain(reading, 0, 50);
          A[index] = reading;
        }else{
          A[index] = min(A[index], reading);
        }
#if DEBUG_MODE
        Serial.print(A[index]);
        Serial.print(",");
#endif
      }
#if DEBUG_MODE
      Serial.println();
#endif
    }
    Serial.println();
}

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/* str must be PRESSURE_SIZE bytes */
void int_array_to_string(int *int_arr, char *char_arr){
  for (int i = 0; i < PRESSURE_SIZE; i++){
    char_arr[i] = (char)(int_arr[i] + 65);
  }
}

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
