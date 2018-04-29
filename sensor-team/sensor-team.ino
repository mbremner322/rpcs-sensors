#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_HTU21DF.h"
#include "BluefruitConfig.h"
#include <Adafruit_BNO055.h>
#include <ArduinoJson.h>

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif


/*===============HELPER FUNCTION DECLARATIONS=====================*/
void error(const __FlashStringHelper*err);
void displaySensorDetails(Adafruit_BNO055 b);
void connect_to_ble();
void initialize_bnos();
float get_imu_angle();
void int_array_to_string(int &int_arr, char &str);
void pressure_map_init();
/*===============HELPER FUNCTION DECLARATIONS=====================*/


/*==========================GLOBALS========================================*/
/**Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);**/
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno2 = Adafruit_BNO055(56, 0x29);
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

const int BAUDRATE = 115200;
const int BLE_BUFFER_SIZE = 22;
const int PRESSURE_SIZE = 105;
int pressure_map_arr[105];
char pressure_char_arr[105];
const float UPPER_IMU_THRESH = 90.0;
const float LOWER_IMU_THRESH = -90.0;


/** PRESSURE MAP GLOBALS AND DESCRIPTION **/
/**
 * 8x8 pressure matrix test
 * 
 * This test verifies that multiplexers can be used to read a single pressure map point
 * 
 * Two multiplexers (MAX4617CPE) are connected to the feather M0 board. 
 *  1) Mut ROW: Pin X is connected to analog input with a 10k pull up, 
 *              pin A is connected to a digital input, 
 *              pins X0-X7 are connected to the rows of the sensor
 *  2) Mut COL: Pin X is connected to gnd,
 *              Pin A is connected to a digital input
 *              Pins X0-X7 are connected to the columns of the sensor
 */
const int MUX_COL_A_PIN = 4;
const int MUX_COL_B_PIN = 3;
const int MUX_COL_C_PIN = 2;

const int MUX_ROW_A_PIN = 7;
const int MUX_ROW_B_PIN = 6;
const int MUX_ROW_C_PIN = 5;
const int ANA_PIN = 14; 

// dimensions for the physical pressure map
const int NUM_PHYS_ROWS = 8;
const int NUM_PHYS_COLS = 8;

// dimensions for the matrix this program generates
const int NUM_VIRT_ROWS = 15;
const int NUM_VIRT_COLS = 7;

// These macros compute whether the A, B, or C outputs to the muxes should be high or low
// based on the desired mux selection
#define GET_A_OUTPUT(x) ((x) & 0x1)
#define GET_B_OUTPUT(x) (((x) & 0x2) >> 1)
#define GET_C_OUTPUT(x) (((x) & 0x4) >> 2)

/*==========================GLOBALS========================================*/

void setup(void)
{
  Serial.begin(BAUDRATE);
  
  Serial.println(F("Sensor Team IMU->BLE"));
  Serial.println(F("------------------------------------------------"));
  
  initialize_bnos();
  pressure_map_init();
  connect_to_ble();
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
  get_pressure_array(pressure_map_arr);
  int_array_to_string(pressure_map_arr, pressure_char_arr);

  /* construct json object */
  root["a"] = get_imu_angle();
  root["t"] = 0.0;//htu.readTemperature();
  root["h"] = 0.0;//htu.readHumidity();
  root["p"] = pressure_char_arr;

  /* stringify json */
  root.printTo(json_buf);
  buf_len = strlen(json_buf);
  //Serial.println(json_buf);

  //Serial.println(buf_len);

  /* break up string into chunks of BLE_BUFFER_SIZE and send individually */
  for (int i = 0; i < buf_len; i += BLE_BUFFER_SIZE){
      char temp_buf[BLE_BUFFER_SIZE + 1];
      
      memcpy(temp_buf, json_buf+i, BLE_BUFFER_SIZE);
      temp_buf[BLE_BUFFER_SIZE] = '\0';
      //Serial.println(temp_buf);
      /* Send input data to host via Bluefruit */
      ble.print(temp_buf);
      ble.flush();
      delay(210);
  }

   tock = millis();
   Serial.print("This iteration took ");
   Serial.print(tock - tick);
   Serial.println(" milliseconds");
   Serial.println();
}


/** =========================== HELPER FUNCTIONS ============================ **/
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void pressure_map_init(){
    // Initialize all pins
  pinMode(ANA_PIN, INPUT);
  
  pinMode(MUX_COL_A_PIN, OUTPUT);
  pinMode(MUX_COL_B_PIN, OUTPUT);
  pinMode(MUX_COL_C_PIN, OUTPUT);
  
  pinMode(MUX_ROW_A_PIN, OUTPUT);
  pinMode(MUX_ROW_B_PIN, OUTPUT);
  pinMode(MUX_ROW_C_PIN, OUTPUT);

  // Make sure pin A inputs to both muxes are low so the sensor starts disconnected
  digitalWrite(MUX_COL_A_PIN, LOW);
  digitalWrite(MUX_COL_B_PIN, LOW);
  digitalWrite(MUX_COL_C_PIN, LOW);
  
  digitalWrite(MUX_ROW_A_PIN, LOW);
  digitalWrite(MUX_ROW_B_PIN, LOW);
  digitalWrite(MUX_ROW_C_PIN, LOW);
}

void connect_to_ble(){
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
      //error(F("Couldn't factory reset"));
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

void initialize_bnos(){
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

/** arithmetic to shift the z-orientation value to [-90, 90] **/
float get_imu_angle(){
  sensors_event_t event1, event2;
  float average, shifted;
  
  bno1.getEvent(&event1);
  bno2.getEvent(&event2);

  average = ((float)event1.orientation.z + (float)event2.orientation.z) /2;
  if (average < -90.0){
    shifted = -average - 270;
  }
  else{
    shifted = 90.0 - average;
  }

  /** apply thresholds **/
  if (shifted > UPPER_IMU_THRESH){
    shifted = UPPER_IMU_THRESH;
  }
  if (shifted < LOWER_IMU_THRESH){
    shifted = LOWER_IMU_THRESH;
  }
  return shifted;
}

/* str must be PRESSURE_SIZE bytes */
void int_array_to_string(int *int_arr, char *char_arr){
  for (int i = 0; i < PRESSURE_SIZE; i++){
    char_arr[i] = (char)(int_arr[i] + 65);
  }
}

/* generate 105 points between 0 and 50 using the 8x8 map */
void get_pressure_array(int *A){
    int reading;
    
    for (int c = 0; c < NUM_VIRT_COLS; c++) {
    digitalWrite(MUX_COL_A_PIN, GET_A_OUTPUT(c % NUM_PHYS_COLS));
    digitalWrite(MUX_COL_B_PIN, GET_B_OUTPUT(c % NUM_PHYS_COLS));
    digitalWrite(MUX_COL_C_PIN, GET_C_OUTPUT(c % NUM_PHYS_COLS));
    
    for (int r = 0; r < NUM_VIRT_ROWS; r++) {
      digitalWrite(MUX_ROW_A_PIN, GET_A_OUTPUT(r % NUM_PHYS_ROWS));
      digitalWrite(MUX_ROW_B_PIN, GET_B_OUTPUT(r % NUM_PHYS_ROWS));
      digitalWrite(MUX_ROW_C_PIN, GET_C_OUTPUT(r % NUM_PHYS_ROWS));

      reading = analogRead(ANA_PIN);
      reading = map(reading, 250, 500, 50, 0);
      reading = constrain(reading, 0, 50);
      
      *A = reading;
      A++;
    }
  }
}





