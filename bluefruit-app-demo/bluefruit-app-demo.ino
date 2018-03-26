
#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_HTU21DF.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "BluefruitConfig.h"

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

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

Adafruit_HTU21DF htu = Adafruit_HTU21DF();
MPU6050 accelgyro;

void setup() {
  while (!Serial){
      delay(1);
  }
  
  Serial.begin(9600);
  Serial.println(F("Temperature and Accelerometer to Bluefruit App demo"));

  Serial.println(F("Initializing temperature sensor"));
  if (!htu.begin()) {
    Serial.println("Couldn't find sensor!");
    while (1);
  }

  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


  Serial.print(F("Initialising the Bluefruit LE module: "));
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }
  
}


void loop() {
    
    int c;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    while( !ble.available() );
    /**
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["temperature"] = htu.readTemperature();
    root["humidity"] = htu.readHumidity();
    JsonArray& acc = root.createNestedArray("accelerometer");
    acc.add(ax);acc.add(ay);acc.add(az);
    JsonArray& gyr = root.createNestedArray("gyroscope");
    gyr.add(gx);gyr.add(gy);gyr.add(gz);

    String output;
    root.printTo(output);
    
    ble.print(F("AT+BLEUARTTX="));
    ble.print(output);
    **/
    char buffer[200];
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sprintf(buffer, "Temp=%f, Hum=%f, accX=%hi, accY=%hi, accZ=%hi \n", htu.readTemperature(), htu.readHumidity(), ax, ay, az);
    // print temp
    Serial.println(buffer);
    ble.print("AT+BLEUARTTX=");
    ble.print(buffer);

    // check response stastus
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send?"));
    }

    delay(1000);
}
