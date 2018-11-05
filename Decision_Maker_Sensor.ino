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

// define various Parameters
#define WaitForEcho 10 // how long the US sensors wait for the echo (in microseconds)
#define WackValue 10000 // the Value above which we consider the value not real
#define DurationCheck 10 // Check to see if old duration is different enough from current reading
#define MinObsDis 10 // distance we consider to be unsafe in cm
#define Close 4 // What we consider too close to wall
#define SensorSpace 10 // space between sensors
#define TurningTime 2500 // in millisex

//Declaring the Distance Array
double DisArray[6];
// DisArray is indexed as follows *0* is Forward, *1* is Forward Right, *2* is Forward Left, *3* Rear R, *4* Rear L, *5* is backwards
char Names [8] = {'F', 'F', 'R', 'F', 'L', 'R', 'L', 'B'}; // names of Values for read outs, just for debugging
int cnt = 0;

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
  pinMode(52, OUTPUT);
  Serial.begin(9600);


}

void loop() {
  /* The loop is where we will read all the sensor data, this is the default state of everything, once it receives all the values
      it checks forward. T
  */

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


  // Filters out wack values
  if (durationF > WackValue) durationF = old_durationF;
  if (durationB > WackValue) durationB = old_durationB;
  if (durationFR > WackValue) durationFR = old_durationFR;
  if (durationFL > WackValue) durationFL = old_durationFL;
  if (durationR > WackValue) durationR = old_durationR;
  if (durationL > WackValue) durationL = old_durationL;

  // Checks to see if change in duration, if not calculate distance and print!
  if (abs(old_durationF - durationF) > DurationCheck) {
    distanceF = .0343 * durationF / 2;

  }
  if (abs(old_durationB - durationB) > DurationCheck) {
    distanceB = .0343 * durationB / 2;

  }

  if (abs(old_durationFR - durationFR) > DurationCheck) {
    distanceFR = .0343 * durationFR / 2;

  }
  if (abs(old_durationFL - durationFL) > DurationCheck) {
    distanceFL = .0343 * durationFL / 2;

  }
  if (abs(old_durationR - durationR) > DurationCheck) {
    distanceR = .0343 * durationR / 2;

  }
  if (abs(old_durationL - durationL) > DurationCheck) {
    distanceL = .0343 * durationL / 2;

  }
  DisArray[0] = distanceF;
  DisArray[1] = distanceFR;
  DisArray[2] = distanceFL;
  DisArray[3] = distanceR;
  DisArray[4] = distanceL;
  DisArray[5] = distanceB;
  //delay(100);

    ////  never delete this!!!!!!!!! Debugging uses Prints out array of distances for easy debugging
    Serial.print(Names[0]);
    Serial.print(",    ");
    Serial.print(Names[1]);
    Serial.print(Names[2]);
    Serial.print(",    ");
    Serial.print(Names[3]);
    Serial.print(Names[4]);
    Serial.print(",    ");
    Serial.print(Names[5]);
    Serial.print(",    ");
    Serial.print(Names[6]);
    Serial.print(",    ");
    Serial.print(Names[7]);
    Serial.println("");
    for (int i = 0; i <= 5; i++) {
      Serial.print(DisArray[i]);
      Serial.print(", ");
    }
    Serial.println("");
    Serial.println("");



  checkForward(DisArray); // checks forward path

  old_durationF = durationF;
  old_durationB = durationB;
  old_durationFR = durationFR;
  old_durationFL = durationFL;
  old_durationR = durationR;
  old_durationL = durationL;


}

//
void checkForward(double A[6]) {
  /* function that checks the forward IR sensor (first value in array) is sensing anything in front of it,
      if not move forward function, wait for forward function to complete and then return to read values
      if so it begins side check
  */
  //Serial.println("Checking Forward!");
  if (A[0] >= MinObsDis) {
    moveForward();
    return;
  }
  else {
    // stopLocomotion(); possible function that sends stop signal
    checkSides(A);

    }

}

void checkSides(double A[6]) {
  /* Checks to see if Right side has anything within distance, if so turn right wait for that to complete and then return to
      measure distances again
     if not check left, if left is good turn left (make three right turns) and return to read some values
     if not turn around and go backwards (make two right turns) and return to read some distances
  */
  //Serial.println("Checking Sides!");
  if (A [1] >= MinObsDis) {
    // BackRight Should also be factored in
    turnRight();
    return;

  }
  else if (A [2] >= MinObsDis) {
    //Serial.println("Turning Left!");
    turnLeft();
    return;
  }
  else if ( A [5] >= MinObsDis) {
    // Serial.println("Going Back!");
    //    turnRight();
    //    delay(200); // wait for the turn to happen (maybe the uno tells us once this is done)
    //    turnRight ();
    //    delay(200);// wait for the turn to happen (maybe the uno tells us once this is done)
    return;
  }
}

void moveForward() {
  Serial.println("w"); // sends move Forward to the Uno
  waitForResponse();
}


void turnRight() {

  Serial.println("d"); // sends turn right function to other guy
  waitForResponse();
}

void turnLeft() {
  Serial.println("a"); // sends turn right function to other guy
  waitForResponse();
}

void checkAlign () {
  /* Function that checks allignment, looks to see if the two R sensors are within a range (small enough = close to wall),
      checks how different they are and sends value to reallign, if they are out of range we don't care
     Does the same for the left
  */
  double Difference;
  //bool SpinRight;
  double Angle = 0;
  if (DisArray[1] < Close && DisArray[3] < Close ) {
    Difference = (DisArray[1] - DisArray [3]);
    if (abs(Difference) > 1.5) {
      //if (Difference > 0) SpinRight = true;
      //else SpinRight = false;
      Angle = asin (Difference / SensorSpace);
    }



  }
  else if (DisArray[2] < Close && DisArray[4] < Close) {
    Difference = DisArray[2] - DisArray [4];
    if (abs(Difference) > 1.5) {
      //if (Difference > 0) SpinRight = false;
      // else SpinRight = true;
      Angle = asin(Difference / SensorSpace);
    }

  }

  return;
}

void waitForResponse() {
  // Function that waits for a response
  delay(100);
  int KylesClear = Serial.read();
  int StartTime = millis();
  while (Serial.available() == 0) {
    //if ((millis()-StartTime) > TurningTime)break; // waits for turning time, if matt+katherine's code blows up, this fixes us
    }
  KylesClear = Serial.read();
  //Serial.flush();
}


