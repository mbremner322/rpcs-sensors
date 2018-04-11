// Example by Tom Igoe

import processing.serial.*;

int NUM_POINTS = 64;
int dynamic_range_low = -17;
int dynamic_range_high = 110;

boolean calibrating = false;
int calibration_count= 0;
int calibration_iterations = 10;

float[] avg_array = new float[NUM_POINTS];

Serial myPort;  // The serial port

void setup() {
  size(640, 640);
  // List all the available serial ports:
  printArray(Serial.list());
  // Open the port you are using at the rate you want:
  myPort = new Serial(this, Serial.list()[1], 9600);
  colorMode(RGB, 255);
}

void draw() {
  while (myPort.available() > 0) {
    int lf = 10;
    // Expand array size to the number of bytes you expect:
    byte[] inBuffer = new byte[500];
    myPort.readBytesUntil(lf, inBuffer);
    if (inBuffer != null) {
      String data = new String(inBuffer);
      data = trim(data);
      int[] data_array = int(split(data, ','));
      if (data_array.length == NUM_POINTS) {
        if (calibrating) {
          textSize(50);
          fill(255, 0, 0);
          text("Calibrating", 10, 70);
          for(int i = 0; i < NUM_POINTS; i++) {
            avg_array[i] += data_array[i];
          }
          
          calibration_count++;
          if (calibration_count >= calibration_iterations) {
            calibrating = false;
            for(int i = 0; i < NUM_POINTS; i++) {
              avg_array[i] = avg_array[i] / calibration_count; 
            }
          }
        } else {
          int r;
          int c;
          float intensity;
          for(int i = 0; i < NUM_POINTS; i++) {
            c = 7 - i/8;
            r = i%8;
            intensity = map((float)(data_array[i]) - avg_array[i], 
                            dynamic_range_low, dynamic_range_high, 0, 255);
            fill(intensity);
            rect(c*80, r*80, 80, 80); 
          }
        }
      }
    }
  }
}

void keyPressed() {

  calibrating = true;
  /*
  print("calibrating");
  for (int i = 0; i < NUM_POINTS; i++) {
    avg_array[i] = 0;
  }
  
  int count = 0;
  while (count < calibration_count) {
    while (myPort.available() > 0) {
      int lf = 10;
      // Expand array size to the number of bytes you expect:
      byte[] inBuffer = new byte[500];
      myPort.readBytesUntil(lf, inBuffer);
      if (inBuffer != null) {
        String data = new String(inBuffer);
        data = trim(data);
        int[] data_array = int(split(data, ','));
        if (data_array.length == NUM_POINTS) {
          for(int i = 0; i < NUM_POINTS; i++) {
            avg_array[i] += data_array[i];
          }
          count++;
          if (count >= calibration_count) {
            break;
          }
        }
      }
    }
  }
  
  for(int i = 0; i < NUM_POINTS; i++) {
    avg_array[i] = avg_array[i] / calibration_count; 
  }
  
  print("end_calibration");
  */
}
