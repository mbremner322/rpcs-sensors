# rpcs-sensors

Spring 2018 Rapid Prototyping of Computer Systems Sensor team code. 

The code used in the final demo can be found in sensor-team folder. This can be loaded onto a Teensy 3.5 microcontroller connected to the circuit described in the teensy_circuit.jpg and pressure_map_circuit.jpg files to run the final demo. 

Code to verify the pressure matrix can be found in the Pressure Map folder. The Arduino sketches (.ino) can be loaded onto a microcontroller hooked up to a pressure matrix (circuit described in the sketches). The data from the pressure matrixes can then be viewed using on of the processing sketches (.pde) run on any computer. 

# Arduino IDE setup
    1. Add https://adafruit.github.io/arduino-board-index/package_adafruit_index.json to Boards Manager URLs option in preferences
    1. Install Arduino SAMD Boards (32-bits ARM Cortex-M0+) by Arduino in Boards Manager
    1. Install Adafruit SAMD Boards by Adafruit in Boards Manager

     https://learn.adafruit.com/add-boards-arduino-v164/setup
     https://learn.adafruit.com/adafruit-feather-m0-basic-proto/using-with-arduino-ide

# Teensy 3.5 setup
    1. Download Teensyduino
       https://www.pjrc.com/teensy/td_download.html
    1. In Arduino IDE, choose Board "Teensy 3.5"

# Libraries needed
    * Bluetooth LE - Adafruit Bluefruit LE 
      https://github.com/adafruit/Adafruit_BluefruitLE_nRF51
    * Accelerometer - Adafruit ADXL345
      https://github.com/adafruit/Adafruit_ADXL345
    * IMU - Adafruit BNO055
      https://github.com/adafruit/Adafruit_BNO055
     
