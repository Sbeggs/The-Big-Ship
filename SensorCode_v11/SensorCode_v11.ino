//--------------------------------------------------------------------------------
// SENSOR DECISION MAKING - VERSION 10 - FIXED DELAYS
//--------------------------------------------------------------------------------

//INFORMATION: --> Modified consistent readings to ignore big changes in the value unless the change persists for two readings in a row
//             --> Issue is with the backwards and left directions when it gets stuck, cant read really long values consistently so never moves

//DECLARING NEW SERIAL COMMUNICATION
#include <SoftwareSerial.h>
 SoftwareSerial sendUno(10,11); //Rx/Tx

//PINS FOR ULTRASONIC SENSORS
#define TrigB 34
#define EchoB 35
#define TrigFR 26
#define EchoFR 27
#define TrigFL 24
#define EchoFL 25
#define TrigF 32
#define EchoF 33
#define TrigR 28
#define EchoR 29
#define TrigL 30
#define EchoL 31

//SENSOR READING VALUES
#define WaitForEcho 10 // how long the US sensors wait for the echo (in microseconds)
#define WackValue 100000 // the Value above which we consider the value not real
#define DurationCheck 40 // Check to see if old duration is different enough from current reading

//WALL AVOIDANCE VARIABLES
#define minMovableDistance 20
#define stoppingDistance 13
#define minWallDistanceFB 5 // was once 6
#define minWallDistanceLR 5

#define shiftTime 200

//USABLE SENSOR VALUES VARIABLES
double F;
double FR;
double FL;
double R;
double L;
double B;

//DISTANCE ARRAY VARIABLES
double DisArray[6];
// DisArray is indexed as follows *0* is Forward, *1* is Forward Right, *2* is Forward Left, *3* Rear R, *4* Rear L, *5* is backwards
char Names [8] = {'F', 'F', 'R', 'F', 'L', 'R', 'L', 'B'}; // names of Values for`` read outs, just for debugging

//ULTRASONIC SENSOR FUNCTIONAL VARIABLES
long durationF;
double distanceF;
long old_durationF = 0;
long durationFR;
double distanceFR;
long old_durationFR = 0;
long durationFL;
double distanceFL;
long old_durationFL = 0;
long durationR;
double distanceR;
long old_durationR = 0;
long durationL;
double distanceL;
long old_durationL = 0;
long durationB;
double distanceB;
long old_durationB = 0;

void setup() {
  pinMode(TrigF, OUTPUT);
  pinMode(EchoF, INPUT);
  pinMode(TrigFR, OUTPUT);
  pinMode(EchoFR, INPUT);
  pinMode(TrigFL, OUTPUT);
  pinMode(EchoFL, INPUT);
  pinMode(TrigR, OUTPUT);
  pinMode(EchoR, INPUT);
  pinMode(TrigL, OUTPUT);
  pinMode(EchoL, INPUT);
  pinMode(TrigB, OUTPUT);
  pinMode(EchoB, INPUT);
  Serial.begin(9600);
  //delay(3000); //EVERYONE RELAX JESUS
  sendUno.begin(9600);
}

void loop() {
  
  readSensorValues();
//  Serial.println("HERE");

  if (F > minMovableDistance) driveForward();
  else if (FR > minMovableDistance) driveRight();
  else if (FL > minMovableDistance) driveLeft();
  else if (B > minMovableDistance) driveBackward();
  
}


//--------------------------------------------------------------
// DRIVING FUNCTIONS
//--------------------------------------------------------------

void driveForward(){
  Serial.print("w");
  sendUno.println("w");

  while (F > stoppingDistance){
    readSensorValues();
    if (FR < minWallDistanceLR || R < minWallDistanceLR) {
      Serial.print("q"); //SHIFT LEFT
      sendUno.print("q");
      delay(shiftTime);
    }
    else if (FL < minWallDistanceLR || L < minWallDistanceLR) {
      Serial.print("e"); //SHIFT RIGHT
      sendUno.print("e");
      delay(shiftTime);
    }
  }
  Serial.print("z");
  sendUno.println("z");
}

void driveBackward(){
  Serial.print("s");
  sendUno.println("s");
  while (B > stoppingDistance){
    if (R < minWallDistanceLR || FR < minWallDistanceLR) {
      Serial.print("q"); //SHIFT LEFT
      sendUno.print("q");
      delay(shiftTime);
    }
    else if (L < minWallDistanceLR || FL < minWallDistanceLR) {
      Serial.print("e"); //SHIFT RIGHT
      sendUno.print("e");
      delay(shiftTime);
    }
    readSensorValues();
  }
  Serial.print("z");
  sendUno.println("z");
}

void driveRight(){
  Serial.print("d");
  sendUno.println("d");
  while (FR > stoppingDistance || R > stoppingDistance){
    if (F < minWallDistanceFB){
      Serial.print("r"); //SHIFT BACKWARD
      sendUno.print("r");
      delay(shiftTime);
    }
    else if (B < minWallDistanceFB){
      Serial.print("f"); //SHIFT FORWARD
      sendUno.print("f");
      delay(shiftTime);
    }
    readSensorValues();
  }
  Serial.print("z");
  sendUno.println("z");
}

void driveLeft(){
  Serial.print("a");
  sendUno.println("a");
  while (FL > stoppingDistance || L > stoppingDistance){
    if (F < minWallDistanceFB){
      Serial.print("r"); //SHIFT BACKWARD
      sendUno.print("r");
      delay(shiftTime);
    }
    else if (B < minWallDistanceFB){ 
      Serial.print("f"); //SHIFT FORWARD
      sendUno.println("f");
      delay(shiftTime);
    }
    readSensorValues();
  }
  Serial.print("z");
  sendUno.println("z");
}


//--------------------------------------------------------------
// READ SENSOR VALUE FUNCTIONS
//--------------------------------------------------------------

void readSensorValues() {
  //turns off trigger
  digitalWrite(TrigF, LOW);
  digitalWrite(TrigB, LOW);
  digitalWrite(TrigFR, LOW);
  digitalWrite(TrigFL, LOW);
  digitalWrite(TrigR, LOW);
  digitalWrite(TrigL, LOW);
  delayMicroseconds(2);

  // Sets TriggerF on for WaitForEcho useconds
  digitalWrite(TrigF, HIGH);
  delayMicroseconds(WaitForEcho);
  digitalWrite(TrigF, LOW);
  // reads the time it took for the EchoF to reach back
  durationF = pulseIn(EchoF, HIGH);
  // Sets TriggerB on for WaitForEcho useconds, can't do two at same time for w.e reason
  digitalWrite(TrigB, HIGH);
  delayMicroseconds(WaitForEcho);
  digitalWrite(TrigB, LOW);
  // reads the time it took for the EchoB to reach back
  durationB = pulseIn(EchoB, HIGH);
  // Sets TriggerFR on for WaitForEcho useconds, can't do two at same time for w.e reason
  digitalWrite(TrigFR, HIGH);
  delayMicroseconds(WaitForEcho);
  digitalWrite(TrigFR, LOW);
  // reads the time it took for the EchoFR to reach back
  durationFR = pulseIn(EchoFR, HIGH);
  // Sets TriggerFL on for WaitForEcho useconds, can't do two at same time for w.e reason
  digitalWrite(TrigFL, HIGH);
  delayMicroseconds(WaitForEcho);
  digitalWrite(TrigFL, LOW);
  // reads the time it took for the EchoFL to reach back
  durationFL = pulseIn(EchoFL, HIGH);
  // Sets TriggerR on for WaitForEcho useconds, can't do two at same time for w.e reason
  digitalWrite(TrigR, HIGH);
  delayMicroseconds(WaitForEcho);
  digitalWrite(TrigR, LOW);
  // reads the time it took for the EchoB to reach back
  durationR = pulseIn(EchoR, HIGH);
  // Sets TriggerL on for WaitForEcho useconds, can't do two at same time for w.e reason
  digitalWrite(TrigL, HIGH);
  delayMicroseconds(WaitForEcho);
  digitalWrite(TrigL, LOW);
  // reads the time it took for the EchoL to reach back
  durationL = pulseIn(EchoL, HIGH);

  // Filters out wack values, may not need to exist
  if (durationF > WackValue) durationF = old_durationF;
  if (durationB > WackValue) durationB = old_durationB;
  if (durationFR > WackValue) durationFR = old_durationFR;
  if (durationFL > WackValue) durationFL = old_durationFL;
  if (durationR > WackValue) durationR = old_durationR;
  if (durationL > WackValue) durationL = old_durationL;

  // Checks to see if change in duration, if not calculate distance and print!
  if (abs(old_durationF - durationF) > DurationCheck) distanceF = .0343 * durationF / 2;
  if (abs(old_durationB - durationB) > DurationCheck) distanceB = .0343 * durationB / 2;
  if (abs(old_durationFR - durationFR) > DurationCheck) distanceFR = .0343 * durationFR / 2;
  if (abs(old_durationFL - durationFL) > DurationCheck) distanceFL = .0343 * durationFL / 2;
  if (abs(old_durationR - durationR) > DurationCheck) distanceR = .0343 * durationR / 2;
  if (abs(old_durationL - durationL) > DurationCheck) distanceL = .0343 * durationL / 2;

  DisArray[0] = distanceF;
  DisArray[1] = distanceFR;
  DisArray[2] = distanceFL;
  DisArray[3] = distanceR;
  DisArray[4] = distanceL;
  DisArray[5] = distanceB;

////  //  //  never delete this!!!!!!!!! Debugging uses Prints out array of distances for easy debugging
//    Serial.print(Names[0]);
//    Serial.print(",    ");
//    Serial.print(Names[1]);
//    Serial.print(Names[2]);
//    Serial.print(",    ");
//    Serial.print(Names[3]);
//    Serial.print(Names[4]);
//    Serial.print(",    ");
//    Serial.print(Names[5]);
//    Serial.print(",    ");
//    Serial.print(Names[6]);
//    Serial.print(",    ");
//    Serial.print(Names[7]);
//    Serial.println("");
//    for (int i = 0; i <= 5; i++) {
//      Serial.print(DisArray[i]);
//      Serial.print(", ");
//    }
//    Serial.println("");
//    Serial.println("");
  
  old_durationF = durationF;
  old_durationB = durationB;
  old_durationFR = durationFR;
  old_durationFL = durationFL;
  old_durationR = durationR;
  old_durationL = durationL;

  F = DisArray[0];
  FR = DisArray[1];
  FL = DisArray[2];
  R = DisArray[3];
  L = DisArray[4];
  B = DisArray[5];
}
//
//void sensorValues() {
//  double sensorComparisonArray[2][6];
//  int i = 0;
//  int j = 0;
//  double allowedSensorError = 5;
//  double sum;
//  double checkValue = 0;
//  int consistency;
//  bool consistentReadings = false;
//  
//  while (consistentReadings == false) {
//    consistency = 0;
//    sum = 0;
//    for (i = 0; i < 2; i++) {
//      readSensorValues();
//      for (j = 0; j < 6; j++) {
//        sensorComparisonArray[i][j] = DisArray[j];
//      }
//    }
//    for (i = 0; i < 6; i++) {
//      for (j = 0; j < 2; j++) {
//        sum = sum + sensorComparisonArray[j][i];
//        //Serial.println(sensorComparisonArray[j][i]);
//      }
//      if ((sum < ((sensorComparisonArray[0][i] * 2) + allowedSensorError)) && sum > ((sensorComparisonArray[2][i] * 2) - allowedSensorError)) {
//        consistency = consistency + 1;
//      }
//      sum = 0;
//    }
//    
//    if (consistency == 6) {
//      consistentReadings = true;
//    }
//    else {
////            Serial.println("FALSE!");
//    }
//    
//    F = DisArray[0];
//    FR = DisArray[1];
//    FL = DisArray[2];
//    R = DisArray[3];
//    L = DisArray[4];
//    B = DisArray[5];
//
////    Serial.println(F);
// 
//  //  //  never delete this!!!!!!!!! Debugging uses Prints out array of distances for easy debugging
////    Serial.print(Names[0]);
////    Serial.print(",    ");
////    Serial.print(Names[1]);
////    Serial.print(Names[2]);
////    Serial.print(",    ");
////    Serial.print(Names[3]);
////    Serial.print(Names[4]);
////    Serial.print(",    ");
////    Serial.print(Names[5]);
////    Serial.print(",    ");
////    Serial.print(Names[6]);
////    Serial.print(",    ");
////    Serial.print(Names[7]);
////    Serial.println("");
////    Serial.print(F);
////    Serial.print(", ");
////Serial.print(FR);
////    Serial.print(", ");
////Serial.print(FL);
////    Serial.print(", ");
////Serial.print(R);
////    Serial.print(", ");
////Serial.print(L);
////    Serial.print(", ");
////Serial.print(B);
////    Serial.print(", ");
////    
////    Serial.println("");
////    Serial.println("");
//  
//
//
//  }
//}
