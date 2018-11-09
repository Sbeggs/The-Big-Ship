//time in millis to execute each of the drive functions for 13.3V battery
#define right_turning_time 570
#define left_turning_time 550
#define forwards_time 500
#define short_forwards_time 169
#define right_align_time 90
#define left_align_time 90
 
// PWM
unsigned int pwm_R = 200;
unsigned int pwm_L = 200;

//Gyroscope values
float d_angle; 
float c_angle; //current_angle
float s_angle; //straight angle
float t_angle=5; //tolerance angle
float K=1;

//serial inputs
char incomingByte;
char command;

//gyroscope
#include <Wire.h>

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroXCalli = 0, gyroYCalli = 0, gyroZCalli = 0;
long gyroXPresent = 0, gyroYPresent = 0, gyroZPresent = 0;
long gyroXPast = 0, gyroYPast = 0, gyroZPast = 0;
float rotX, rotY, rotZ;

float angelX = 0, angelY = 0, angelZ = 0;
float angle,prev_angle = 0;

long timePast = 0;
long timePresent = 0;
 
void setup() {
  
  //Setup Motor Channels
  pinMode(13, OUTPUT); //Initiates Motor Channel B pin
  pinMode(8, OUTPUT);  //Initiates Brake Channel B pin
  pinMode(12, OUTPUT); //Initiates the Motor Channel A pin
  pinMode(9, OUTPUT); //Initiates the Brake channel A pin

  //Initiate Serial Monitor
  Serial.begin(9600);

  //Initiate Gyroscope
  Wire.begin();
  setUpMPU();
  callibrateGyroValues();
  timePresent = millis();
}
 
void loop() {
  
  //waiting for command
  if (Serial.available()>0){
    incomingByte=Serial.read();
    command=incomingByte;
    }

//  //For debugging
//  command='w';
////  delay(1000);
  
  //moving forward
  if (command == 'w') {
//    Serial.println("Commencing Forward Sequence");

    runR(pwm_R, true);
    runL(pwm_L, false);

    drive_straight(forwards_time);
    
    fs();
    
    //reset command
    command = 'z';
//    Serial.println("z"); //Send serial monitor on Mega z to indicate move is done
    }

  //moving backwards
  if (command == 's') {
    //Serial.println("Commencing Backward Sequence");
    
    // move backwards until some distance
    runR(pwm_R, true);
    runL(pwm_L, false);

    delay(short_forwards_time);

    //full stop
    fs();

    //reset command
    command = 'z';
    Serial.println("z");  //send mega value 
    }

  //right turn
  if (command == 'd') {
      //Serial.println("Commencing Right Turn");
  
      // spin right some distance
      runR(pwm_R, false);
      runL(pwm_L, false);

      delay(right_turning_time);

      //full stop
      fs();
      
      //reset command
      command = 'z';
      Serial.println("z");  //send mega confirmation
      }
  
  //left turn 
  if (command == 'a') {
      //Serial.println("Commencing Left Turn");
  
      // spin left some distance
        runR(pwm_R, true);
        runL(pwm_L, true);

        delay(left_turning_time);
          
        //full stop
        fs();
      
        //reset command
        command = 'z';
        Serial.println("z");  //send mega confirmation
      }
      
   //align right
   if (command == 'e') {
      //Serial.println("Commencing Right Turn");
  
      // spin right some distance
      runR(pwm_R, false);
      runL(pwm_L, false);

      delay(right_align_time);

      //full stop
      fs();

      //reset command
      command = 'z';
      Serial.println("z");  //send mega confirmation
      }
     

   //align left
   if (command == 'q') {
      //Serial.println("Commencing Left Turn");
  
      // spin left some distance
        runR(pwm_R, true);
        runL(pwm_L, true);

        delay(left_align_time);
        //full stop
        fs();
      
        //reset command
        command = 'z';
        Serial.println("z");  //send mega confirmation
      }

}

void runL(int speed, boolean rev) {
  //Channel A
  if(rev) {
    //Establishes backward direction of Channel A
    digitalWrite(12, LOW);
  }else {
    //Establishes forward direction of Channel A
    digitalWrite(12, HIGH); 
  }
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, speed);   //Spins the motor on Channel A at full speed
}  

void runR(int speed, boolean rev) {
  //Channel B
  if(rev) {
    digitalWrite(13, LOW); //Establishes backwards direction of Channel B
  }else{
    digitalWrite(13, HIGH); //Establishes forward direction of Channel B
  }
   
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, speed);   //Spins the motor on Channel B at full speed 
}

//fullstop
void fs(){
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);

  runR(0,true);
  runL(0,true);
}
void drive_straight(int delay_time){
  int i;
  s_angle = abs(gyro());
  for(;i<(delay_time/10);i++){
      c_angle = abs(gyro());
      Serial.println();
      Serial.print("Straight Angle: "); Serial.println(s_angle);
      Serial.print("Current Angle: "); Serial.println(c_angle);
      if (c_angle<(s_angle-t_angle)){
        //adjust left motor
        pwm_L=pwm_L+2;

        //Keep pwm [0 255]
        if (pwm_L>255){
          pwm_L=255;
        }
        else if(pwm_L<0){
          pwm_L=0;
        }
        runL(pwm_L,false);
        Serial.print("Increased pwm_L: ");
        Serial.println(pwm_L);
      }

      else if(c_angle>(s_angle+t_angle)){
        //adjust left motor
        pwm_L=pwm_L-2;

        //Keep pwm [0 255]
        if (pwm_L>255){
          pwm_L=255;
        }
        else if(pwm_L<0){
          pwm_L=0;
        }
        runL(pwm_L,false);
        Serial.print("Decreased pwm_L: ");
        Serial.println(pwm_L);
      }
      
      delay(10);
  }
}

 float gyro(){
  readAndProcessAccelData();
  readAndProcessGyroData();
  angle=angelZ;
  if(abs(angle-prev_angle)<1){
    angle=prev_angle;
  }
  else{
    prev_angle=angle;
//    Serial.print("Angle: "); Serial.println(angle);
  }
//  printData();
  return angle;
}

//_______________________________________________________________________________________________
//
//                          GYROSCOPE NESTED FUNCTIONS
//_______________________________________________________________________________________________

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
    for (int i=0; i<5000; i++) {
      getGyroValues();
      gyroXCalli = gyroXCalli + gyroXPresent;
      gyroYCalli = gyroYCalli + gyroYPresent;
      gyroZCalli = gyroZCalli + gyroZPresent;
    }
    gyroXCalli = gyroXCalli/5000;
    gyroYCalli = gyroYCalli/5000;
    gyroZCalli = gyroZCalli/5000;
}

void readAndProcessAccelData() {
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x3B); 
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); 
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); 
  accelY = Wire.read()<<8|Wire.read(); 
  accelZ = Wire.read()<<8|Wire.read(); 
  processAccelData();
}

void processAccelData() {
  gForceX = accelX/16384.0;
  gForceY = accelY/16384.0; 
  gForceZ = accelZ/16384.0;
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
  Wire.requestFrom(0b1101000,6);                              // Request for 6 bytes from gyro registers (43 - 48)
  while(Wire.available() < 6);                                // Wait untill all 6 bytes are available
  gyroXPresent = Wire.read()<<8|Wire.read();                  // Store first two bytes into gyroXPresent
  gyroYPresent = Wire.read()<<8|Wire.read();                  // Store next two bytes into gyroYPresent
  gyroZPresent = Wire.read()<<8|Wire.read();                  //Store last two bytes into gyroZPresent
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
  angelX = angelX + ((timePresent - timePast)*(gyroXPresent + gyroXPast - 2*gyroXCalli)) * 0.00000382;
  angelY = angelY + ((timePresent - timePast)*(gyroYPresent + gyroYPast - 2*gyroYCalli)) * 0.00000382;
  angelZ = angelZ + ((timePresent - timePast)*(gyroZPresent + gyroZPast - 2*gyroZCalli)) * 0.00000382;
}

void printData() {
//  Serial.println("Gyro (deg/sec)");
//  Serial.print(" X=");
//  Serial.print(rotX);
//  Serial.print(" Y=");
//  Serial.println(rotY); 
//  Serial.print("Angle (deg)");
//  Serial.print(" Z=");
//  Serial.println(rotZ);

//  Serial.println("Angular displacement wrt started position (deg)");
//  Serial.print("angel of X axis=");
//  Serial.print(angelX);
//  Serial.print(" angel of Y axis=");
//  Serial.print(angelY);
  Serial.print(" angel of Z axis=");
  Serial.println(angelZ);

//  Serial.println("Acceleration (g)");
//  Serial.print(" X=");
//  Serial.print(gForceX);
//  Serial.print(" Y=");
//  Serial.print(gForceY);
//  Serial.print(" Z=");
//  Serial.println(gForceZ);
}
