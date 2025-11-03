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

#define LEFT_ENCODER_FRONT 2
#define LEFT_ENCODER_REAR 3

#define RIGHT_ENCODER_FRONT 18
#define RIGHT_ENCODER_REAR 19


const uint8_t countPerRotation = 180;
const uint8_t wheelCircumference = 223.84;  //mm

static volatile int16_t count_left = 0;
static volatile int16_t count_right = 0;

int error = 0;

// 3.215 = (60sec/0.1sec)/(48gear ratio * 4pulses/rev)
float rotation = 3.125;
float RPM = 0;

uint8_t base_speed = 155;
uint8_t right_speed = 155;
uint8_t left_speed = 155;

#define BAUD_RATE 9600

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

  pinMode(LEFT_ENCODER_FRONT, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_FRONT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_FRONT), ISRMotorLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_FRONT), ISRMotorRight, FALLING);
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
void Forward() {
  analogWrite(MotorPWM_A, left_speed);
  analogWrite(MotorPWM_B, right_speed);

  // Left Motor
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  // Right Motor
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
}

void rightTurn() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(FRONT_RIGHT_TURN, HIGH);
    digitalWrite(REAR_RIGHT_TURN, HIGH);
    delay(100);
    digitalWrite(FRONT_RIGHT_TURN, LOW);
    digitalWrite(REAR_RIGHT_TURN, LOW);
    delay(100);
  }
}

void leftTurn() {
  // Set motors to turn left

  for (int i = 0; i < 10; i++) {
    digitalWrite(FRONT_LEFT_TURN, HIGH);
    digitalWrite(REAR_LEFT_TURN, HIGH);
    delay(100);
    digitalWrite(FRONT_LEFT_TURN, LOW);
    digitalWrite(REAR_LEFT_TURN, LOW);
    delay(100);
  }
}

void reverse() {
  digitalWrite(REAR_LEFT_REVERSE, HIGH);
  digitalWrite(REAR_RIGHT_REVERSE, HIGH);
  delay(1000);
  digitalWrite(REAR_LEFT_REVERSE, LOW);
  digitalWrite(REAR_RIGHT_REVERSE, LOW);
}

void brake() {
  digitalWrite(REAR_RIGHT_BRAKE, HIGH);
  digitalWrite(REAR_LEFT_BREAK, HIGH);
  delay(1000);
  digitalWrite(REAR_RIGHT_BRAKE, LOW);
  digitalWrite(REAR_LEFT_BREAK, LOW);
}

int Kp = 1;
int Kd = 0;
int Ki = 0;

float integral = 0;

void pidSpeedAdjust() {
  static unsigned long prevTime = millis();
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;
  static int last_error = error;
  error = count_left - count_right;

  integral += error * dt;
  float derivative = (error - last_error) / dt;

  float output = Kp * error + Ki * integral + Kd * derivative;
  Serial.println(output);

  left_speed = base_speed + output;
  right_speed = base_speed - output;
}

//encoder reading to RPM
void loop() {
  count_left = count_left % 180;
  count_right = count_right % 180;
  pidSpeedAdjust();
  Forward();
}
