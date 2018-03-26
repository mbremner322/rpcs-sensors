/**
 * Single Point multiplexer test
 * 
 * This test verifies that multiplexers can be used to read a single pressure map point
 * 
 * Two multiplexers (MAX4617CPE) are connected to the feather M0 board. 
 *  1) Mut ANA: Pin X is connected to analog input with a 10k pull up, 
 *              pin A is connected to a digital input, 
 *              pin X1 is connected to one side of the single point sensor
 *  2) Mut GND: Pin X is connected to gnd,
 *              Pin A is connected to a digital input
 *              Pin X1 is connected to one side of the single point sensor
 */

const int MUT_GND_A_PIN = 6;
const int MUT_ANA_A_PIN = 5;

const int ANA_PIN = A0; // The pin that the first analog mutex is connected to


void setup() {
  // Initialize all pins
  pinMode(ANA_PIN, INPUT);
  pinMode(MUT_GND_A_PIN, OUTPUT);
  pinMode(MUT_ANA_A_PIN, OUTPUT);

  // Make sure pin A inputs to both muxes are low so the sensor starts disconnected
  digitalWrite(MUT_GND_A_PIN, LOW);
  digitalWrite(MUT_ANA_A_PIN, LOW);

  Serial.begin(9600);

}

void loop() {
  int disconnected_read = analogRead(ANA_PIN);

  // now connect sensor with muxes
  digitalWrite(MUT_GND_A_PIN, HIGH);
  digitalWrite(MUT_ANA_A_PIN, HIGH);
  delay(100);
  int connected_read = analogRead(ANA_PIN);

  digitalWrite(MUT_GND_A_PIN, LOW);
  digitalWrite(MUT_ANA_A_PIN, LOW);
  
  Serial.println("disconnected: " + String(disconnected_read) +
                 "\t connected: " + String(connected_read));

  delay(100);

}
