// define all Digital Pins for Mega(Even is Trig, Odd is Echo)
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
 
 //need to add a function of realign that realigns to skew away from a wall if within too close a distance of it
//Would be realign away from wall, medium forward, realign back once in the opposite direction
//Assuming initial condition was its too close to the wall but properly aligned
 
 // define various Parameters
#define WaitForEcho 10 // how long the US sensors wait for the echo (in microseconds)
#define WackValue 100000 // the Value above which we consider the value not real
#define DurationCheck 40 // Check to see if old duration is different enough from current reading
#define MinObsDis 12// distance we consider to be unsafe in cm for basic cardinal directions
#define MinWallDis 7 // closest to the wall we want to get to 6.5
#define MinForwardDis 17 // when it will switch over to short forwarding
#define MinForwardShortDis 9 // minimum distance which we're comfortable short forward
#define MinDistance 7 // What we consider too close to wall for allignment
#define MaxWallDis 9 // max distance we want to be away from a wall post turn
#define MinDistanceAfterTurn 12

#define TurningTime 3000 // in millisex
#define RealignValue 1 // the difference between two values that triggers a re-allign

 //Realign Value Variables
double F;
double FR;
double FL;
double R;
double L;
int alignedOrNot;
bool RightOrLeft; // Right is true Left is false
 //Declaring the Distance Array
double DisArray[6];
// DisArray is indexed as follows *0* is Forward, *1* is Forward Right, *2* is Forward Left, *3* Rear R, *4* Rear L, *5* is backwards
char Names [8] = {'F', 'F', 'R', 'F', 'L', 'R', 'L', 'B'}; // names of Values for`` read outs, just for debugging
unsigned int TimeOut = 0;
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
int KylesClear;
 double theta = 0;
 bool realigned;
bool didSomething;
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
 }
 void loop() {
  /* The loop is where we will read all the sensor data, this is the default state of everything, once it receives all the values
      it checks forward. T
  */
  realign();
   tooCloseToWall();
   checkForward(DisArray); // checks forwards path and moves wherever
 }
 
 void realign() {
   sensorValues();
  aligned();
   realigned = false;
   didSomething == true;
   while (didSomething == true) {
     while (alignedOrNot != 2) { //break if aligned returns 2 which corresponds to no response needed
       realigned = true;
       if (alignedOrNot == 0) { //if it returns 0 need to turn right
        realignRight();
      }
      if (alignedOrNot == 1) { //if it returns 1 need to turn left
        realignLeft();
      }
      sensorValues();
      aligned();
    }
     if (realigned == true) {
      didSomething = true;
    }
     else {
      didSomething = false;
    }
  }
}
 //--------------------------------------------------------------
// CHECKING AVAILABLE MOVEMENTS FUNCTIONS
//--------------------------------------------------------------
 void checkForward(double A[6]) {
  /* function that checks the forward IR sensor (first value in array) is sensing anything in front of it,
      if not move forward function, wait for forward function to complete and then return to read values
      if so it begins side check
  */
  //Serial.println("Checking Forward!");
  bool didItAlign = false;
   if (A[0] >= MinForwardDis) {
    moveForward();
  }
  else if (A[0] >= MinForwardShortDis) {
    shortForward();
  }
   else {
    checkSides(A);
  }
}
 void checkSides(double A[6]) {
  /* Checks to see if Right side has anything within distance, if so turn right wait for that to complete and then check allignment
     if not check left, if left is good turn left (make three right turns) and return to read some values
     if not turn around and go backwards and return to read some distances
  */
  if (A [1] >= MinObsDis) {
    turnRight();
    RightOrLeft = true;
    realignAfterTurn();
//    realign();
//    tooFarToWall();
    }
    else if (A [2] >= MinObsDis) {
    turnLeft();
    RightOrLeft = false;
    realignAfterTurn();
//    realign();
//    tooFarToWall();
  }
   else {
    turnRight();
    turnRight();
    realignAfterTurn();
  }
 }
 //--------------------------------------------------------------
// MOVING FUNCTIONS
//--------------------------------------------------------------
 void moveForward() {
  Serial.println("w"); // sends move Forward to the Uno
  waitForResponse();
}
void turnRight() {
  Serial.println("d"); // sends turn right function to other guy
  waitForResponse();
}
void turnLeft() {
  Serial.println("a"); // sends turn left function to other guy
  waitForResponse();
}
void shortForward() {
  Serial.println("s"); // sends move back command to other guy
  waitForResponse();
}
 //--------------------------------------------------------------
// ALIGNMENT FUNCTIONS
//--------------------------------------------------------------
 void realignRight() {
  Serial.println("e"); // sends move back command to other guy
  waitForResponseRealign();
}
 void realignLeft() {
  Serial.println("q"); // sends move back command to other guy
  waitForResponseRealign();
}
 void aligned () {
   FR = DisArray[1];
  FL = DisArray[2];
  R = DisArray[3];
  L = DisArray[4];
   alignedOrNot = 2;
   if (FL < MinDistance || L < MinDistance) {
    if (abs(L - FL) < 7) { // look at this value
      if ((L - FL) > RealignValue) {
        alignedOrNot = 0;
      }
    }
  }
  else if (FR < MinDistance || R < MinDistance) {
    if (abs(R - FR) < 7) {
      if ((R - FR) > RealignValue) {
        alignedOrNot = 1;
      }
    }
  }
   delay(500);
 }
 void tooCloseToWall() {
   sensorValues();
   while (FL < MinWallDis || L < MinWallDis) {
    if (F > MinForwardShortDis) {
      realignRight();
      shortForward();
      realignLeft();
      sensorValues();
    }
    else {
      break;
    }
  }
   while (FR < MinWallDis || R < MinWallDis) {
    if (F > MinForwardShortDis) {
      realignLeft();
      shortForward();
      realignRight();
      sensorValues();
    }
    else {
      break;
    }
  }
  
  realign();
}

 void tooFarToWall() {
 //  sensorValues();
//
//  while (FL > MaxWallDis || L > MaxWallDis) {
//    if (F > MinForwardShortDis) {
//      realignLeft();
//      shortForward();
//      realignRight();
//      sensorValues();
//    }
//    else {
//      break;
//    }
//  }
//
//
//  if (F > MinForwardShortDis) {
//    if (RightOrLeft) {
//      if (FL > MaxWallDis || L > MaxWallDis) {
//        realignLeft();
//        shortForward();
//        realignRight();
//      }
//
//    }
//    if (!RightOrLeft) {
//      if (FR > MaxWallDis || R > MaxWallDis) {
//        realignRight();
//        shortForward();
//        realignLeft();
//      }
//
//    }
//  }
}
 //--------------------------------------------------------------
// WAIT FOR RESPONSE FUNCTIONS
//--------------------------------------------------------------
 void waitForResponse() {
  // Function that waits for a response
  delay(200);
  //  KylesClear = Serial.read();
  while (Serial.available() == 0) {
    if (++TimeOut == TurningTime)break; // waits for turning time, if matt+katherine's code blows up, this resets us
    delay(1);
  }
  KylesClear = Serial.read();
  TimeOut = 0;
  delay(500);
}
 void waitForResponseRealign() {
  // Function that waits for a response
  delay(200);
  //  KylesClear = Serial.read();
  while (Serial.available() == 0) {
    if (++TimeOut == TurningTime)break; // waits for turning time, if matt+katherine's code blows up, this resets us
    delay(1);
  }
  KylesClear = Serial.read();
  TimeOut = 0;
  delay(500);
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
   //  //  never delete this!!!!!!!!! Debugging uses Prints out array of distances for easy debugging
  //  Serial.print(Names[0]);
  //  Serial.print(",    ");
  //  Serial.print(Names[1]);
  //  Serial.print(Names[2]);
  //  Serial.print(",    ");
  //  Serial.print(Names[3]);
  //  Serial.print(Names[4]);
  //  Serial.print(",    ");
  //  Serial.print(Names[5]);
  //  Serial.print(",    ");
  //  Serial.print(Names[6]);
  //  Serial.print(",    ");
  //  Serial.print(Names[7]);
  //  Serial.println("");
  //  for (int i = 0; i <= 5; i++) {
  //    Serial.print(DisArray[i]);
  //    Serial.print(", ");
  //  }
  //  Serial.println("");
  //  Serial.println("");
   old_durationF = durationF;
  old_durationB = durationB;
  old_durationFR = durationFR;
  old_durationFL = durationFL;
  old_durationR = durationR;
  old_durationL = durationL;
}
 /*Function that takes in 3 consecutive sensor values and checks that there consistent in all directions before
  allowing the program to continue and use them
*/
void sensorValues() {
   double sensorComparisonArray[3][6];
  int i = 0;
  int j = 0;
  double allowedSensorError = 3;
  double sumOf3;
  double checkValue = 0;
   int consistency;
   bool consistentReadings = false;
   while (consistentReadings == false) {
     consistency = 0;
    sumOf3 = 0;
     for (i = 0; i < 3; i++) {
      readSensorValues();
      for (j = 0; j < 6; j++) {
        sensorComparisonArray[i][j] = DisArray[j];
      }
    }
     for (i = 0; i < 6; i++) {
      for (j = 0; j < 3; j++) {
        sumOf3 = sumOf3 + sensorComparisonArray[j][i];
        //Serial.println(sensorComparisonArray[j][i]);
      }
      //Serial.println(sumOf3);
       if ((sumOf3 < ((sensorComparisonArray[0][i] * 3) + allowedSensorError)) && sumOf3 > ((sensorComparisonArray[2][i] * 3) - allowedSensorError)) {
        consistency = consistency + 1;
      }
      sumOf3 = 0;
    }
     //Serial.println(consistency);
     if (consistency == 6) {
      consistentReadings = true;
      //      Serial.println("TRUE!");
    }
    else {
      //      Serial.println("FALSE!");
    }
     F = DisArray[0];
    FR = DisArray[1];
    FL = DisArray[2];
    R = DisArray[3];
    L = DisArray[4];
   }
 }


 //--------------------------------------------------------------
// REALIGNED AFTER TURN
//--------------------------------------------------------------

 void alignedAfterTurn () {
   FR = DisArray[1];
  FL = DisArray[2];
  R = DisArray[3];
  L = DisArray[4];
   alignedOrNot = 2;
   if (FL < MinDistanceAfterTurn || L < MinDistanceAfterTurn) {
    if (abs(L - FL) < 10) { // look at this value
      if ((L - FL) > RealignValue) {
        alignedOrNot = 0;
      }
    }
  }
  else if (FR < MinDistanceAfterTurn || R < MinDistanceAfterTurn) {
    if (abs(R - FR) < 10) {
      if ((R - FR) > RealignValue) {
        alignedOrNot = 1;
      }
    }
  }
   delay(500);
 }

  void realignAfterTurn() {
   sensorValues();
   alignedAfterTurn();
   realigned = false;
   didSomething == true;   
   
   while (didSomething == true) {
     while (alignedOrNot != 2) { //break if aligned returns 2 which corresponds to no response needed
       realigned = true;
       if (alignedOrNot == 0) { //if it returns 0 need to turn right
        realignRight();
      }
      if (alignedOrNot == 1) { //if it returns 1 need to turn left
        realignLeft();
      }
      sensorValues();
      alignedAfterTurn();
    }
     if (realigned == true) {
      didSomething = true;
    }
     else {
      didSomething = false;
    }
  }
}
