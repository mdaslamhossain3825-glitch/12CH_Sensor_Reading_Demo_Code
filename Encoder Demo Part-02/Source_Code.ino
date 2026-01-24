//ENCODER PIN DEFINITIONS
#define encoder_left_A 5    //(green wire)
#define encoder_left_B 18   //(yellow wire)
#define encoder_right_A 19  //(yellow wire)
#define encoder_right_B 23  //(green wire)
// MOTOR pins for BTS7960 (PWM controlled)
#define left_motor_forward 16     // LPWM
#define left_motor_backward 17    // RPWM
#define right_motor_forward 2   // LPWM
#define right_motor_backward 4  // RPWM
// Buttons
#define button1 15
//LEDC CONFIG
#define LEDC_FREQ 20000
#define LEDC_RES 8
// LEDC channels
#define CH_LF 0
#define CH_LB 1
#define CH_RF 2
#define CH_RB 3
//set target pulses
uint16_t EncoderSum;
uint16_t target_pulse = 200; //adjust according to your requirement

//ENCODER COUNTERS
volatile long left_pulses = 0;
volatile long right_pulses = 0;
//LAST QUADRATURE STATES
volatile uint8_t lastStateL = 0;
volatile uint8_t lastStateR = 0;
//TASK HANDLE
TaskHandle_t EncoderTaskHandle;

//pulse decode and Count
inline int8_t decodeQuadrature(uint8_t last, uint8_t current) {
  if ((last == 0b00 && current == 0b01) || (last == 0b01 && current == 0b11) || (last == 0b11 && current == 0b10) || (last == 0b10 && current == 0b00)) {
    return +1;
  }
  if ((last == 0b00 && current == 0b10) || (last == 0b10 && current == 0b11) || (last == 0b11 && current == 0b01) || (last == 0b01 && current == 0b00)) {
    return -1;
  }
  return 0;
}

//ENCODER TASK ON CORE 0
void EncoderTask(void *parameter) {
  // Initialize last states
  lastStateL = (digitalRead(encoder_left_A) << 1) | digitalRead(encoder_left_B);
  lastStateR = (digitalRead(encoder_right_A) << 1) | digitalRead(encoder_right_B);

  while (true) {
    // -------- LEFT ENCODER --------
    uint8_t currentStateL = (digitalRead(encoder_left_A) << 1) | digitalRead(encoder_left_B);

    int8_t dirL = decodeQuadrature(lastStateL, currentStateL);
    left_pulses += dirL;
    lastStateL = currentStateL;

    // -------- RIGHT ENCODER --------
    uint8_t currentStateR = (digitalRead(encoder_right_A) << 1) | digitalRead(encoder_right_B);

    int8_t dirR = decodeQuadrature(lastStateR, currentStateR);
    right_pulses += dirR;
    lastStateR = currentStateR;

    //delay
    vTaskDelay(1);
  }
}
//MOTOR CONTROL
void motor(int LPWM, int RPWM) {
  LPWM = constrain(LPWM, -255, 255);
  RPWM = constrain(RPWM, -255, 255);

  // LEFT MOTOR
  if (LPWM > 0) {
    ledcWrite(CH_LF, LPWM);
    ledcWrite(CH_LB, 0);
  } else if (LPWM < 0) {
    ledcWrite(CH_LF, 0);
    ledcWrite(CH_LB, -LPWM);
  } else {
    ledcWrite(CH_LF, 0);
    ledcWrite(CH_LB, 0);
  }

  // RIGHT MOTOR
  if (RPWM > 0) {
    ledcWrite(CH_RF, RPWM);
    ledcWrite(CH_RB, 0);
  } else if (RPWM < 0) {
    ledcWrite(CH_RF, 0);
    ledcWrite(CH_RB, -RPWM);
  } else {
    ledcWrite(CH_RF, 0);
    ledcWrite(CH_RB, 0);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(encoder_left_A, INPUT_PULLUP);
  pinMode(encoder_left_B, INPUT_PULLUP);
  pinMode(encoder_right_A, INPUT_PULLUP);
  pinMode(encoder_right_B, INPUT_PULLUP);
  pinMode(button1, INPUT_PULLUP);
  pinMode(left_motor_forward, OUTPUT);
  pinMode(left_motor_backward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);
  // Configure LEDC channels and Attach PWM channels
  ledcSetup(CH_LF, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(left_motor_forward, CH_LF);
  ledcSetup(CH_LB, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(left_motor_backward, CH_LB);
  ledcSetup(CH_RF, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(right_motor_forward, CH_RF);
  ledcSetup(CH_RB, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(right_motor_backward, CH_RB);
  // Create encoder task on Core 0
  xTaskCreatePinnedToCore(
    EncoderTask,  // Task function
    "EncoderTask",
    4096,  // Stack size
    NULL,
    2,  // Priority
    &EncoderTaskHandle,
    0  // Core 0
  );
}


void loop() {
  //start task when button is clicked
  if (digitalRead(button1) == 0) {
    left_pulses = 0;
    right_pulses = 0;
    EncoderSum = 0;

    motor(-60, -60);  //back

    //run motor untill the target pulses are completed
    while (1) {
      EncoderSum = abs(right_pulses) + abs(left_pulses);
      if ((EncoderSum / 2) >= target_pulse) break;
    }
    motor(0, 0); //motor stop
  }
}
