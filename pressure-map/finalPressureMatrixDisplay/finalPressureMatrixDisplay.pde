// Example by Tom Igoe

import processing.serial.*;

int NUM_POINTS = 105;

Serial myPort;  // The serial port

void setup() {
  size(1200, 560);
  // List all the available serial ports:
  printArray(Serial.list());
  // Open the port you are using at the rate you want:
  myPort = new Serial(this, Serial.list()[3], 9600);
}

void draw() {
  while (myPort.available() > 0) {
    int lf = 10;
    // Expand array size to the number of bytes you expect:
    byte[] inBuffer = new byte[1000];
    myPort.readBytesUntil(lf, inBuffer);
    if (inBuffer != null) {
      String data = new String(inBuffer);
      //println(data);
      data = trim(data);
      int[] data_array = int(split(data, ','));
      //println(data_array);
      if (data_array.length == NUM_POINTS) {
        int r;
        int c;
        float intensity;
        for(int i = 0; i < NUM_POINTS; i++) {
          c = i%15;
          r = i/15;
          intensity = map((float)data_array[i], 300, 600, 0, 255);
          
          fill(intensity);
          rect(c*80, r*80, 80, 80); 
        }
      }
    }
  }
}
