#include <EEPROM.h>
//motorpins
#define right_motor_forward 3   //PWM PIN
#define right_motor_backward 5  //PWM PIN
#define left_motor_forward 6    //PWM PIN
#define left_motor_backward 9   //PWM PIN
//speed control pins (PWM)
//#define right_motor_speed 3  //EnA (L298) //TB6612 --- PWMA
//#define left_motor_speed 9   //EnB (L298) //TB6612 --- PWMB
//MUX PIN
#define S0 2
#define S1 4
#define S2 7
#define S3 8
#define SIG_PIN A7
//button
#define button1 10
#define button2 11
#define button3 12
//led
#define led 13

//Sensor Variables
#define sensorNumber 12
int sensorADC[sensorNumber];
int sensorDigital[sensorNumber];
int bitWeight[sensorNumber] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048 };
int WeightValue[sensorNumber] = { 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120 };
int theshold = 500;
int sumOnSensor;
int sensorWight;
int bitSensor;
int Max_ADC[sensorNumber];
int Min_ADC[sensorNumber];
int Reference_ADC[sensorNumber];
//PID Variables
float line_position;
float error;
float center_position = 45;
float derivative, previous_error;
int base_speed = 150;
int kp = 8;
int kd = 100;
//inverse parameter
bool inverseON = 0;  //0 = normal line, 1 = inverse line


//************************************SETUP***************************************
void setup() {
  Serial.begin(9600);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(led, OUTPUT);

  pinMode(left_motor_forward, OUTPUT);
  pinMode(left_motor_backward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);

  LoadCalibration();  //load calibration value from eeprom whenever the arduino is start or reset
}


//************************************LOOP***************************************
void loop() {
  if (digitalRead(button1) == 0) {  //when button 1 is pressed
    calibrateSensor();
  }
  if (digitalRead(button2) == 0) {  //when button 2 is pressed
    PID_Controller(base_speed, kp, kd);
  }
}

void PID_Controller(int base_speed, int p, int d) {

  while (1) {
a:
    read_black_line();

    if (sumOnSensor > 0) line_position = sensorWight / sumOnSensor;
    error = center_position - line_position;
    derivative = error - previous_error;
    int right_motor_correction = base_speed + (error * p + derivative * d);
    int left_motor_correction = base_speed - (error * p + derivative * d);
    previous_error = error;
    //Drive Motor According to PID Value
    motor(left_motor_correction, right_motor_correction);
    //
    if (bitSensor == 207 || bitSensor == 231 || bitSensor == 239 || bitSensor == 231 || bitSensor == 247 || bitSensor == 243) {
      inverseON = !inverseON;
      digitalWrite(led, inverseON);
      Bit_Sensor_Show();
      goto a;
    }
  }
}

void calibrateSensor() {
  // Initialize calibration limits
  for (int i = 0; i < sensorNumber; i++) {
    Max_ADC[i] = 0;     // Set the maximum ADC value for each sensor to 0 (lowest possible)
    Min_ADC[i] = 1024;  // Set the minimum ADC value for each sensor to 1024 (highest possible, since ADC range is 0–1023)
  }
  // Define rotation directions for calibration sweeps
  int rotationDirections[][2] = {
    { -1, 1 },  // Rotate robot slightly to the left
    { 1, -1 },  // Rotate robot slightly to the right
    { 1, -1 },  // Repeat rotation to the right for better calibration coverage
    { -1, 1 }   // Rotate left again to cover the full sensor range
  };

  // Perform calibration sweeps (4 passes)
  for (int step = 0; step < 4; step++) {
    // Rotate the robot slowly in the defined direction for this step
    motor(base_speed * 0.4 * rotationDirections[step][0],
          base_speed * 0.4 * rotationDirections[step][1]);

    // Sweep through several readings while rotating
    for (int sweep = 0; sweep < 500; sweep++) {
      // Read each sensor one by one
      for (int i = 0; i < sensorNumber; i++) {
        selectChannel(i);                    // Select which sensor channel to read (if using multiplexer)
        sensorADC[i] = analogRead(SIG_PIN);  // Read the analog value from the sensor

        // Update the maximum and minimum ADC values recorded for this sensor
        Max_ADC[i] = max(Max_ADC[i], sensorADC[i]);
        Min_ADC[i] = min(Min_ADC[i], sensorADC[i]);
      }
    }
    motor(0, 0);  // Stop the motors after each rotation step
    delay(200);   // Small pause before changing rotation direction
  }

  // Store calibrated Reference, Max, and Min values in EEPROM
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = (Max_ADC[i] + Min_ADC[i]) / 2.0;  // Calculate mid-point (reference) value

    EEPROM.write(i, Reference_ADC[i] / 4);           // Store reference value (divided by 4 because EEPROM stores 0–255)
    delay(5);                                        // Small delay for EEPROM write stability
    EEPROM.write(i + sensorNumber, Max_ADC[i] / 4);  // Store maximum calibrated value
    delay(5);
    EEPROM.write(i + (sensorNumber * 2), Min_ADC[i] / 4);  // Store minimum calibrated value
    delay(5);
  }
}

// Function to load calibration data from EEPROM
void LoadCalibration() {
  // Print header labels for serial monitor (sensor indices)
  Serial.print("S0: ");
  Serial.print(", S1: ");
  Serial.print(", S2: ");
  Serial.print(", S3: ");
  Serial.print(", S4: ");
  Serial.print(", S5: ");
  Serial.print(", S6: ");
  Serial.print(", S7: ");
  Serial.println();

  // Load the stored calibration data for each sensor
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = EEPROM.read(i) * 4;                 // Read and restore reference value
    Max_ADC[i] = EEPROM.read(i + sensorNumber) * 4;        // Read and restore maximum value
    Min_ADC[i] = EEPROM.read(i + (sensorNumber * 2)) * 4;  // Read and restore minimum value

    Serial.print(String(Reference_ADC[i]) + ",  ");  // Print loaded reference value
  }
  Serial.println();  // Move to new line after printing all sensor values
}
void motor(int LPWM, int RPWM) {

  LPWM = constrain(LPWM, -255, 255);
  RPWM = constrain(RPWM, -255, 255);

  // ----- LEFT MOTOR -----
  if (LPWM > 0) {
    // Forward
    analogWrite(left_motor_forward, LPWM);
    analogWrite(left_motor_backward, 0);
  } else if (LPWM < 0) {
    // Backward
    analogWrite(left_motor_forward, 0);
    analogWrite(left_motor_backward, -LPWM);
  }

  // ----- RIGHT MOTOR -----
  if (RPWM > 0) {
    // Forward
    analogWrite(right_motor_forward, RPWM);
    analogWrite(right_motor_backward, 0);
  } else if (RPWM < 0) {
    // Backward
    analogWrite(right_motor_forward, 0);
    analogWrite(right_motor_backward, -RPWM);
  }
}
void read_black_line() {
  sumOnSensor = 0;
  sensorWight = 0;
  bitSensor = 0;

  for (int i = 0; i < sensorNumber; i++) {
    selectChannel(i);
    delayMicroseconds(20);
    sensorADC[i] = analogRead(SIG_PIN);
    //map analog value x
    //analog to digital
    if (sensorADC[i] > Reference_ADC[i]) {
      if (inverseON == 0) {    //black line - normal mode.
        sensorDigital[i] = 1;  //black line =1
      } else {
        sensorDigital[i] = 0;  //black line =0 (black line -inverse mode)
      }
    } else {
      if (inverseON == 0) {    //white line - normal mode
        sensorDigital[i] = 0;  //white line =0 - normal mode
      } else {
        sensorDigital[i] = 1;  //white line - inverse mode.
      }
    }

    sumOnSensor += sensorDigital[i];
    sensorWight += sensorDigital[i] * WeightValue[i];
    bitSensor += sensorDigital[i] * bitWeight[(sensorNumber - 1) - i];
  }
}

void selectChannel(int channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
}

void analog_value_test() {
  while (1) {
    read_black_line();
    for (int i = 0; i < sensorNumber; i++) {
      Serial.print(String(sensorADC[i]) + "  ");
    }
    Serial.println();
  }
}

void Bit_Sensor_Show() {
  while (1) {
    read_black_line();  //take sensor reading

    Serial.print("Bit Sensor in Decimal: " + String(bitSensor));  //Decimal Value Show
    Serial.print(" | Bit Sensor in Binary: ");

    for (int i = (sensorNumber - 1); i >= 0; i--) {  //display bitsensor data in binary
      Serial.print(String(bitRead(bitSensor, i)));
    }
    Serial.println();
  }
}
