/*
  Into Robotics
*/

//servo.write for library only takes in doubles as an angle

#include <Servo.h>  //add '<' and '>' before and after servo.h

char servoCommand;
double servoAngle;
Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(9600);
  servo1.attach(8);
  servo2.attach(9);
}

void loop() {
  if (Serial.available() > 0)servoCommand = Serial.read();
  if (servoCommand == 'u' || servoCommand == 'd'){ //If its servo for up down
    if (servoCommand == 'u') servoAngle = 20000;
    else if (servoCommand == 'd') servoAngle = 0;
    gripperUpDown();
  }
  if (servoCommand == 'o' || servoCommand == 'c'){ //If its servo for open close
    if (servoCommand == 'c') servoAngle = 20000;
    else if (servoCommand == 'o') servoAngle = 0;
    gripperOpenClose();
  }
  
  Serial.println(servoCommand);
}

void gripperUpDown(){
  servo1.write(servoAngle);      // Turn SG90 servo back to 90 degrees (center position)
  delay(800);
}

void gripperOpenClose(){
  servo2.write(servoAngle);      // Turn SG90 servo back to 90 degrees (center position)
  delay(800);
}
