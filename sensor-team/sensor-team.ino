#include "Adafruit_BluefruitLE_SPI.h"
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
/*===============HELPER FUNCTION DECLARATIONS=====================*/


/*==========================GLOBALS========================================*/
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno2 = Adafruit_BNO055(56, 0x29);
int packet_id = 0;
const int ble_buffer_size = 15;
const int pressure_size = 105;
int pressure_map_arr[105];
char pressure_char_arr[105];
/*==========================GLOBALS========================================*/


void setup(void)
{
  while(!Serial);
  Serial.begin(9600);
  
  Serial.println(F("Sensor Team IMU->BLE"));
  Serial.println(F("------------------------------------------------"));
  
  connect_to_ble();

  initialize_bnos();
}



void loop(void)
{
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  //JsonArray& pressure_array = root.createNestedArray("pressure_array");
  char json_buf[1000];
  int buf_len;
  unsigned long tick, tock;

  tick = millis();
  
  
  /* Wait for connection */
  while (! ble.isConnected()) {
    Serial.println("waiting for connection...");
  }

  /* signal the beginning of this packet and flush the pipeline */
  ble.print("*");
  ble.flush();

  /* get pressure points and map to a string */
  get_pressure_array(pressure_map_arr);
  int_array_to_string(pressure_map_arr, pressure_char_arr);

  /* construct json object */
  root["id"] = packet_id++;
  root["imu_angle"] = get_imu_angle();
  root["temp"] = 0;
  root["hum"] = 0;
  root["pressure_array"] = pressure_char_arr;

  /* stringify json */
  root.printTo(json_buf);
  buf_len = strlen(json_buf);
  Serial.println(json_buf);

  /* break up string into chunks of ble_buffer_size and send individually */
  for (int i = 0; i < buf_len; i += ble_buffer_size){
      char temp_buf[250];
      
      memcpy(temp_buf, json_buf+i, ble_buffer_size);
      temp_buf[ble_buffer_size] = '\0';
      
      Serial.println(temp_buf);
       
      /* Send input data to host via Bluefruit */
      ble.print(temp_buf);
      ble.flush();
      delay(1000);
  }

   tock = millis();
   Serial.print("This iteration took ");
   Serial.print(tock - tick);
   Serial.println(" milliseconds");
}


/** =========================== HELPER FUNCTIONS ============================ **/
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
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
}

float get_imu_angle(){
  sensors_event_t event1, event2;
  bno1.getEvent(&event1);
  bno2.getEvent(&event2);
  return ((float)event1.orientation.z + (float)event2.orientation.z) / 2.0;
}

// str must be pressure_size bytes 
void int_array_to_string(int *int_arr, char *char_arr){
  for (int i = 0; i < pressure_size; i++){
    char_arr[i] = (char)int_arr[i];
  }
}

void get_pressure_array(int *A){
  for (int i = 0; i < pressure_size; i++){
    pressure_map_arr[i] = i % 100 + 48;
  }
}


