#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"
#include <Adafruit_BNO055.h>
#include <ArduinoJson.h>

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

/*==========================GLOBALS========================================*/
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BNO055 bno = Adafruit_BNO055();
int packet_id = 0;
const int ble_buffer_size = 15;
/*==========================GLOBALS========================================*/

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
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

void setup(void)
{
  while(!Serial);

  Serial.begin(9600);
  Serial.println(F("Sensor Team IMU->BLE"));
  Serial.println(F("------------------------------------------------"));
  
  connect_to_ble();
  
    /* Initialise the sensor */
  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}


void insert_imu_data(JsonObject& imu1, JsonObject& imu2){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu1["imu_id"] = 1;
  imu1["x"] = 0.0;
  imu1["y"] = 0.0;
  imu1["z"] = 0.0;

  imu2["imu_id"] = 2;
  imu2["x"] = 0.0;
  imu2["y"] = 0.0;
  imu2["z"] = 0.0;
}


void loop(void)
{
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& imu1 = jsonBuffer.createObject();
  JsonObject& imu2 = jsonBuffer.createObject();
  JsonArray& imu_array = root.createNestedArray("imu");
  char json_buf[500];
  int buf_len;
  

  /* Wait for connection */
  while (! ble.isConnected()) {
  }

  /* signal the beginning of this packet and flush the pipeline */
  ble.print("*");
  ble.flush();

  /* construct json object */
  root["id"] = packet_id++;
  insert_imu_data(imu1, imu2);
  imu_array.add(imu1);
  imu_array.add(imu2);
  root["temp"] = 0;
  root["hum"] = 0;

  /* stringify json */
  root.printTo(json_buf);
  buf_len = strlen(json_buf);

  for (int i = 0; i < buf_len; i += ble_buffer_size){
      char temp_buf[250];
      
      memcpy(temp_buf, json_buf+i, ble_buffer_size);
      temp_buf[ble_buffer_size] = '\0';
      
      Serial.println(temp_buf);
       
      /* Send input data to host via Bluefruit */
      ble.print(temp_buf);
      ble.flush();
  }

  delay(1000);
}
