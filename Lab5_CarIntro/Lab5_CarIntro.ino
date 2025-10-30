// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
#define MotorPWM_A 4  //left motor
#define MotorPWM_B 5  //right motor

#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36
#define FRONT_LEFT_TURN 49
#define FRONT_LEFT_HIGH 47
#define FRONT_LEFT_LOW 45
#define FRONT_RIGHT_TURN 43
#define FRONT_RIGHT_HIGH 41
#define FRONT_RIGHT_LOW 39
#define REAR_RIGHT_TURN 37
#define REAR_RIGHT_REVERSE 35
#define REAR_RIGHT_BRAKE 33
#define REAR_LEFT_TURN 31
#define REAR_LEFT_REVERSE 29
#define REAR_LEFT_BREAK 27

#define ENCODER_LEFT 2
#define ENCODER_RIGHT 3
static volatile int16_t count_left = 0;
static volatile int16_t count_right = 0;

// 3.215 = (60sec/0.1sec)/(48gear ratio * 4pulses/rev)
float rotation = 3.125;
float RPM = 0;
uint8_t speed = 0;

#define BAUD_RATE 38400

void setup() {
  pinMode(MotorPWM_A, OUTPUT);
  pinMode(MotorPWM_B, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);

  pinMode(FRONT_LEFT_TURN, OUTPUT);
  pinMode(FRONT_LEFT_HIGH, OUTPUT);
  pinMode(FRONT_LEFT_LOW, OUTPUT);
  pinMode(FRONT_RIGHT_TURN, OUTPUT);
  pinMode(FRONT_RIGHT_HIGH, OUTPUT);
  pinMode(FRONT_RIGHT_LOW, OUTPUT);
  pinMode(REAR_RIGHT_TURN, OUTPUT);
  pinMode(REAR_RIGHT_REVERSE, OUTPUT);
  pinMode(REAR_RIGHT_BRAKE, OUTPUT);
  pinMode(REAR_LEFT_TURN, OUTPUT);
  pinMode(REAR_LEFT_REVERSE, OUTPUT);
  pinMode(REAR_LEFT_BREAK, OUTPUT);

  pinMode(ENCODER_LEFT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), ISRMotorLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), ISRMotorRight, FALLING);
  Serial.begin(BAUD_RATE);
}

/***************************************/
// This is the Interrupt Service Routine
// for the left motor.
/***************************************/
void ISRMotorLeft() {
  count_left++;
}

void ISRMotorRight() {
  count_right++;
}

// Method: Forward
// Input: speed – value [0-255]
// Rotate the motor in a clockwise fashion
void Forward(int speed) {
  analogWrite(MotorPWM_A, speed);
  analogWrite(MotorPWM_B, speed);

  // Left Motor
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  // Right Motor
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
}

void rightTurn() {
  digitalWrite(FRONT_RIGHT_TURN, HIGH);
  digitalWrite(REAR_RIGHT_TURN, HIGH);
}

void leftTurn() {
  digitalWrite(FRONT_LEFT_TURN, HIGH);
  digitalWrite(REAR_LEFT_TURN, HIGH);
}

//encoder reading to RPM
void loop() {
  leftTurn();
  count_left = 0;
  Forward(speed);
  speed += 5;
  delay(100);
  Serial.print(speed);
  Serial.print(", ");
  RPM = count_left * rotation;
  Serial.println(RPM);
}
