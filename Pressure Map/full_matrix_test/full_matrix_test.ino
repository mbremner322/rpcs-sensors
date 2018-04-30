/**
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

// These macros compute whether the A, B, or C outputs to the lower order muxes
// should be high or low based on the desired mux selection
#define GET_A_OUTPUT(x)  ((x) & 0x1)
#define GET_B_OUTPUT(x)  (((x) >> 1) & 0x1)
#define GET_C_OUTPUT(x)  (((x) >> 2) & 0x1)
// The enable pin is active low, and the chip should be enabled if there is a zero
// in the 2^3 spot
#define GET_EN1_OUTPUT(x) ((x) & 0x8)
// The enable pin is active low, and the chip should be enabled if there is a one
// in the 2^3 spot
#define GET_EN2_OUTPUT(x) (!((x) & 0x8))




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


// The number of milliseconds to wait at the end of the loop
const int WAIT_TIME = 100;

// The baud rate of the serial communication
const int baud_rate = 9600;

// The number of rows in the matrix
const int NUM_ROWS = 15;
// The number of columns in the matrix
const int NUM_COLS = 7;


void setup() {
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


  Serial.begin(baud_rate);

}

void loop() {

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
