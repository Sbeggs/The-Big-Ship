//--------------------------------------------------------------------------------
// 4 MOTOR RUNNING CODE - VERSION 4 - UNO COMPATIBLE
//--------------------------------------------------------------------------------

//INFORMATION: --> w = forward, s = backwards, a = left, d = right
//             --> q = shift right, e = shift left, f = shift forward, r = shift backward
//             --> Board 1 controls forwards and backwards movement
//             --> Board 2 controls left and right movement

//PWM VARIABLES
unsigned int pwm_A1 = 200;
unsigned int pwm_B1 = 200;
unsigned int pwm_A2 = 200;
unsigned int pwm_B2 = 200;

//BOARD 1 - FORWARD AND BACKWARD
int PWMpinA1 = 3;
int PWMpinB1 = 5;
int brakeAmotorA1 = 12;
int brakeBmotorB1 = 13;
int dirAmotorA1 = 10;
int dirBmotorB1 = 11;

//BOARD 2 - LEFT AND RIGHT
int PWMpinA2 = 6;
int PWMpinB2 = 9;

//COMMANDS
char start_command;
char next_command;
char Kyles_Clear;

//SHIFT TIMES
#define left_shift_time 300
#define right_shift_time 300
#define forward_shift_time 300
#define backward_shift_time 300
#define realignRight_time 150
#define realignLeft_time 150


//GYROSCOPE
#include <Wire.h>

//GYRO VALUES
float d_angle;
float c_angle; //current_angle
float s_angle; //straight angle
float t_angle = 5; //tolerance angle

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroXCalli = 0, gyroYCalli = 0, gyroZCalli = 0;
long gyroXPresent = 0, gyroYPresent = 0, gyroZPresent = 0;
long gyroXPast = 0, gyroYPast = 0, gyroZPast = 0;
float rotX, rotY, rotZ;

float angelX = 0, angelY = 0, angelZ = 0;
float angle, prev_angle = 0;

long timePast = 0;
long timePresent = 0;

//Gyro Wiring
// VCC - 5V
// GND - GND
// SCL - A5
// SDA - A4
// INT - D2

void setup() {

  //MOTOR SHIELD 1 ON DIGITAL PINS
  pinMode(brakeAmotorA1, OUTPUT); //Initiates motor 1 channel A brake on board 1
  pinMode(brakeBmotorB1, OUTPUT); // Initiates motor 2 channel B brake on board 1
  pinMode(dirAmotorA1, OUTPUT); // Initiates motor 1 dir A on board 1
  pinMode(dirBmotorB1, OUTPUT); // Initiates motor 1 dir B on board 1

  //MOTORSHIELD 2 ON ANALOG PINS
  pinMode(A0, OUTPUT); //DIRECTION MOTOR B - BOARD 2
  pinMode(A1, OUTPUT); //DIRECTION MOTOR A - BOARD 2
  pinMode(A2, OUTPUT); //BRAKE B - BOARD 2
  pinMode(A3, OUTPUT); //BRAKE A - BOARD 2

  //Initiate Serial Monitor
  Serial.begin(9600);

  //Initiate Gyroscope
  Wire.begin();
  setUpMPU();
  callibrateGyroValues();
  timePresent = millis();

//  Serial.print("Ready to go");
}


//--------------------------------------------------------------------------------
// RECIEVING COMMANDS LOOP
//--------------------------------------------------------------------------------

void loop() {

  if (Serial.available() > 0) start_command = Serial.read(); //GET STARTING COMMAND

  //MOVING FORWARDS
  if (start_command == 'w') {
//    Serial.print("Command Recieved: "); Serial.println(start_command);
    forward();
  }

  //MOVING RIGHT
  else if (start_command == 'd') {
    right();
  }

  //MOVING LEFT
  else if (start_command == 'a') {
    left();
  }

  //MOVING BACKWARD
  else if (start_command == 's') {
    backward();
  }
}


//--------------------------------------------------------------------------------
// DRIVING CODE - FORWARD,BACKWARD,LEFT,RIGHT
//--------------------------------------------------------------------------------

void forward() {
  //record straight angle
  s_angle = gyro(); // initialise what we consider to be straight
  
  //Resets PWM and Start Driving
  runA1(pwm_A1, true);
  runB1(pwm_B1, false);

  //read next_command
  if (Serial.available() > 0) next_command = Serial.read(); //GET NEXT COMMAND

  //Shift while driving
  while (next_command != 'z') { //SHIFT IF NEEDED UNTIL ITS TIME TO STOP
    //Shift if needed
    if (next_command == 'q') {
//      Serial.print("Command Recieved: "); Serial.println(next_command);
      shiftLeft();
    }
    else if (next_command == 'e') {
//      Serial.print("Command Recieved: "); Serial.println(next_command);
      shiftRight();
    }

    //Realign if needed then restart motor
    realign();
    runA1(pwm_A1, true);
    runB1(pwm_B1, false);  

    //Reset and read next_command
    next_command = 'n'; //Neutral Command to do nothing
    if (Serial.available() > 0) next_command = Serial.read();
  }

//  Serial.println("STOPPING");

  //Reset all commands and stop moving
  board1stop(); //STOP MOVING
  start_command = 'n';
  next_command = 'n';
}

void backward() {
  //record straight angle
  s_angle = gyro(); // initialise what we consider to be straight
  
  //Resets PWM and Start Driving
  runA1(pwm_A1, false);
  runB1(pwm_B1, true);

  //read next_command
  if (Serial.available() > 0) next_command = Serial.read(); //GET NEXT COMMAND

  //Shift if needed
  while (next_command != 'z') { //SHIFT IF NEEDED UNTIL ITS TIME TO STOP
    //Shift if needed
    if (next_command == 'q') shiftLeft();
    else if (next_command == 'e') shiftRight();

    //Realign if needed then restart motor
    realign();
    runA1(pwm_A1, false);
    runB1(pwm_B1, true); 

    //Reset and read next_command
    next_command = 'n'; //Neutral Command to do nothing
    if (Serial.available() > 0) next_command = Serial.read();
  }

  //Reset all commands and stop moving
  board1stop(); //STOP MOVING
  start_command = 'n';
  next_command = 'n';
}

void left() {
  //record straight angle
  s_angle = gyro(); // initialise what we consider to be straight
  
  //Resets PWM and Start Driving
  runA2(pwm_A2, true);
  runB2(pwm_B2, false);

  if (Serial.available() > 0) next_command = Serial.read();

  while (next_command != 'z') {
    //shift
    if (next_command == 'f') shiftForward(); //SHIFT FORWARD
    else if (next_command == 'r') shiftBackward(); //SHIFT REVERSE

    //Realign if needed then restart motor
    realign();
    runA2(pwm_A2, true);
    runB2(pwm_B2, false);

    //read command
    next_command = 'n'; //Neutral Command to do nothing
    if (Serial.available() > 0) next_command = Serial.read();
  }

  board2stop();
  start_command = 'n';
  next_command = 'n';
}

void right() {
  //record straight angle
  s_angle = gyro(); // initialise what we consider to be straight
  
  //Resets PWM and Start Driving
  runA2(pwm_A2, false);
  runB2(pwm_B2, true);

  if (Serial.available() > 0) next_command = Serial.read();

  while (next_command != 'z') {
    if (next_command == 'f') shiftForward(); //SHIFT FORWARD
    else if (next_command == 'r') shiftBackward(); //SHIFT REVERSE

    //Realign if needed then restart motor
    realign();
    runA2(pwm_A2, false);
    runB2(pwm_B2, true);

    next_command = 'n'; //Neutral Command to do nothing
    if (Serial.available() > 0) next_command = Serial.read();
  }

  board2stop();
  start_command = 'n';
  next_command = 'n';
}



//--------------------------------------------------------------------------------
// MOTOR CONTROL CODE
// --------------------------------------------------------------------------------

void runA1(int speed, boolean rev) {
  //Channel A
  if (rev) digitalWrite(dirAmotorA1, LOW);
  else digitalWrite(dirAmotorA1, HIGH);

  digitalWrite(brakeAmotorA1, LOW);   //Disengage the Brake for Channel A
  analogWrite(PWMpinA1, speed);   //Spins the motor on Channel A at full speed
}

void runB1(int speed, boolean rev) {
  if (rev) digitalWrite(dirBmotorB1, LOW);
  else digitalWrite(dirBmotorB1, HIGH);

  digitalWrite(brakeBmotorB1, LOW);   //Disengage the Brake for Channel A
  analogWrite(PWMpinB1, speed);   //Spins the motor on Channel A at full speed
}

void runA2(int speed, boolean rev) {
  if (rev) digitalWrite(A1, LOW);
  else digitalWrite(A1, HIGH);

  digitalWrite(A3, LOW);   //Disengage the Brake for Channel B
  analogWrite(PWMpinA2, speed);   //Spins the motor on Channel B at full speed
}


void runB2(int speed, boolean rev) {
  if (rev) digitalWrite(A0, LOW);
  else digitalWrite(A0, HIGH);

  digitalWrite(A2, LOW);   //Disengage the Brake for Channel B
  analogWrite(PWMpinB2, speed);   //Spins the motor on Channel B at full speed
}


//--------------------------------------------------------------------------------
// STOPPING FUNCTIONS
//--------------------------------------------------------------------------------

//BOARD 1 STOPPING FUNCTIONS - FORWARDS AND BACKWARDS
void board1stop() {
  digitalWrite(brakeAmotorA1, HIGH);
  digitalWrite(brakeBmotorB1, HIGH);

  runA1(0, true);
  runB1(0, true);
}

//BOARD 2 STOPPING FUNCTIONS - LEFT AND RIGHT
void board2stop() {
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);

  runA2(0, true);
  runB2(0, true);
}


//--------------------------------------------------------------------------------
// SHIFTING FUNCTIONS
//--------------------------------------------------------------------------------

void shiftLeft() {
  runA2(210, true);
  runB2(210, false);
  delay(left_shift_time);
  board2stop();
}

void shiftRight() {
  runA2(210, false);
  runB2(210, true);
  delay(right_shift_time);
  board2stop();
}

void shiftForward() {
  runA1(210, true);
  runB1(210, false);
  delay(forward_shift_time);
  board1stop();
}

void shiftBackward() {
  runA1(210, false);
  runB1(210, true);
  delay(backward_shift_time);
  board1stop();
}

//--------------------------------------------------------------------------------
//  REALIGN FUNCTIONs
//--------------------------------------------------------------------------------

void realign(){
  //Measure Current Angle
  c_angle=gyro();
  Serial.print("Straight Angle:"); Serial.println(s_angle);
  Serial.print("Current Angle: "); Serial.println (c_angle);
  
  if(c_angle>(s_angle+t_angle)){
    board1stop();
    realignLeft();
    Serial.println("Realign Left");
    Serial.println();
  }

  else if(c_angle<(s_angle-t_angle)) {
    board1stop();
    realignRight();
    Serial.println("Realign Right");
    Serial.println();
  }
}

void realignRight(){
    runA1(210,true);
    runB1(210,true);
    delay(realignRight_time);
    board1stop();
    delay(1000);
}


void realignLeft(){
    runA1(210,false);
    runB1(210,false);
    delay(realignLeft_time);
    board1stop();
    delay(1000);
}

//--------------------------------------------------------------------------------
// GYRO FUNCTIONS
//--------------------------------------------------------------------------------

float gyro() {
  readAndProcessAccelData();
  readAndProcessGyroData();
  angle = angelZ;
  if (abs(angle - prev_angle) < 1) {
    angle = prev_angle;
  }
  else {
    prev_angle = angle;
    //    Serial.print("Angle: "); Serial.println(angle);
  }
  //  printData();
  return angle;
}

void setUpMPU() {
  // power management
  Wire.beginTransmission(0b1101000);          // Start the communication by using address of MPU
  Wire.write(0x6B);                           // Access the power management register
  Wire.write(0b00000000);                     // Set sleep = 0
  Wire.endTransmission();                     // End the communication

  // configure gyro
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);                           // Access the gyro configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();

  // configure accelerometer
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);                           // Access the accelerometer configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();
}

void callibrateGyroValues() {
  for (int i = 0; i < 5000; i++) {
    getGyroValues();
    gyroXCalli = gyroXCalli + gyroXPresent;
    gyroYCalli = gyroYCalli + gyroYPresent;
    gyroZCalli = gyroZCalli + gyroZPresent;
  }
  gyroXCalli = gyroXCalli / 5000;
  gyroYCalli = gyroYCalli / 5000;
  gyroZCalli = gyroZCalli / 5000;
}

void readAndProcessAccelData() {
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);
  while (Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  processAccelData();
}

void processAccelData() {
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
}

void readAndProcessGyroData() {
  gyroXPast = gyroXPresent;                                   // Assign Present gyro reaging to past gyro reading
  gyroYPast = gyroYPresent;                                   // Assign Present gyro reaging to past gyro reading
  gyroZPast = gyroZPresent;                                   // Assign Present gyro reaging to past gyro reading
  timePast = timePresent;                                     // Assign Present time to past time
  timePresent = millis();                                     // get the current time in milli seconds, it is the present time

  getGyroValues();                                            // get gyro readings
  getAngularVelocity();                                       // get angular velocity
  calculateAngle();                                           // calculate the angle
}

void getGyroValues() {
  Wire.beginTransmission(0b1101000);                          // Start the communication by using address of MPU
  Wire.write(0x43);                                           // Access the starting register of gyro readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);                             // Request for 6 bytes from gyro registers (43 - 48)
  while (Wire.available() < 6);                               // Wait untill all 6 bytes are available
  gyroXPresent = Wire.read() << 8 | Wire.read();              // Store first two bytes into gyroXPresent
  gyroYPresent = Wire.read() << 8 | Wire.read();              // Store next two bytes into gyroYPresent
  gyroZPresent = Wire.read() << 8 | Wire.read();              //Store last two bytes into gyroZPresent
}

void getAngularVelocity() {
  rotX = gyroXPresent / 131.0;
  rotY = gyroYPresent / 131.0;
  rotZ = gyroZPresent / 131.0;
}

void calculateAngle() {
  // same equation can be written as
  // angelZ = angelZ + ((timePresentZ - timePastZ)*(gyroZPresent + gyroZPast - 2*gyroZCalli)) / (2*1000*131);
  // 1/(1000*2*131) = 0.00000382
  // 1000 --> convert milli seconds into seconds
  // 2 --> comes when calculation area of trapezium
  // substacted the callibated result two times because there are two gyro readings
  angelX = angelX + ((timePresent - timePast) * (gyroXPresent + gyroXPast - 2 * gyroXCalli)) * 0.00000382;
  angelY = angelY + ((timePresent - timePast) * (gyroYPresent + gyroYPast - 2 * gyroYCalli)) * 0.00000382;
  angelZ = angelZ + ((timePresent - timePast) * (gyroZPresent + gyroZPast - 2 * gyroZCalli)) * 0.00000382;
}

//--------------------------------------------------------------------------------
// ARCHIVED CODE
//--------------------------------------------------------------------------------

////time in millis to execute each of the drive functions for 13.3V battery
//#define left_turning_time 650
//#define right_turning_time 700
//#define forwards_time 500
//#define backwards_time 500
//#define right_align_time 90
//#define left_align_time 90
