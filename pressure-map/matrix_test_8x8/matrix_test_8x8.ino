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

// These macros compute whether the A, B, or C outputs to the muxes should be high or low
// based on the desired mux selection
#define GET_A_OUTPUT(x) ((x) & 0x1)
#define GET_B_OUTPUT(x) (((x) & 0x2) >> 1)
#define GET_C_OUTPUT(x) (((x) & 0x4) >> 2)

/** 
//UNCOMMENT if feather m0 
const int MUX_COL_A_PIN = 10;
const int MUX_COL_B_PIN = 11;
const int MUX_COL_C_PIN = 12;

const int MUX_ROW_A_PIN = 5;
const int MUX_ROW_B_PIN = 6;
const int MUX_ROW_C_PIN = 9;
const int ANA_PIN = A0; // The pin that the first analog mutex is connected to
**/


//UNCOMMENT if teensy 3.5 
const int MUX_COL_A_PIN = 4;
const int MUX_COL_B_PIN = 3;
const int MUX_COL_C_PIN = 2;

const int MUX_ROW_A_PIN = 7;
const int MUX_ROW_B_PIN = 6;
const int MUX_ROW_C_PIN = 5;
const int ANA_PIN = 14; // The pin that the first analog mutex is connected to



// The number of milliseconds to wait at the end of the loop
const int WAIT_TIME = 100;

// The baud rate of the serial communication
const int baud_rate = 9600;

// The number of rows in the matrix
const int NUM_ROWS = 8;
// The number of columns in the matrix
const int NUM_COLS = 8;


void setup() {
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

  Serial.begin(baud_rate);

}

void loop() {

  for (int c = 0; c < NUM_COLS; c++) {
    digitalWrite(MUX_COL_A_PIN, GET_A_OUTPUT(c));
    digitalWrite(MUX_COL_B_PIN, GET_B_OUTPUT(c));
    digitalWrite(MUX_COL_C_PIN, GET_C_OUTPUT(c));
    
    for (int r = 0; r < NUM_ROWS; r++) {
      digitalWrite(MUX_ROW_A_PIN, GET_A_OUTPUT(r));
      digitalWrite(MUX_ROW_B_PIN, GET_B_OUTPUT(r));
      digitalWrite(MUX_ROW_C_PIN, GET_C_OUTPUT(r));

      int reading = analogRead(ANA_PIN);
      Serial.print(reading);

      if (c != (NUM_COLS - 1) || r != (NUM_ROWS - 1)) {
        Serial.print(",");
      }
    }
  }

  // Signal the end of the transmission with a newline character
  Serial.println();

  delay(WAIT_TIME);
}
