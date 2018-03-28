// Example by Tom Igoe

import processing.serial.*;

int NUM_POINTS = 64;

Serial myPort;  // The serial port

void setup() {
  size(640, 640);
  // List all the available serial ports:
  printArray(Serial.list());
  // Open the port you are using at the rate you want:
  myPort = new Serial(this, Serial.list()[1], 9600);
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
        int r;
        int c;
        float intensity;
        for(int i = 0; i < NUM_POINTS; i++) {
          r = i/8;
          c = i%8;
          intensity = map((float)data_array[i], 0, 1023, 0, 255);
          
          fill(intensity);
          rect(c*100, r*100, 100, 100); 
        }
      }
    }
  }
}
