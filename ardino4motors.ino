#define travel_distance 50
#define left_turn_distance 25
#define right_turn_distance 25

//time in millis to execute each of the drive functions for 13.3V battery
#define left_turning_time 650 
#define right_turning_time 700
#define forwards_time 500
#define backwards_time 500
#define right_align_time 90
#define left_align_time 90
 
// PID Variables
unsigned int pwm_A1 = 230;
unsigned int pwm_B1 = 230;
unsigned int pwm_A2 = 230;
unsigned int pwm_B2 = 230;

//setting up motor variables, motor A, motorB on boards 1 and 2
int brakeAmotorA1 = 22;
int brakeBmotorB1 = 23;

int dirAmotorA1 = 24;
int dirBmotorB1 = 25;

int brakeAmotorA2 = 30;
int brakeBmotorB2 = 31;

int dirAmotorA2 = 32;
int dirBmotorB2 = 33;

  //Declaring the PWM pins for the motors
  int PWMA1 = 2;
  int PWMB1 = 3;
  int PWMA2 = 4;
  int PWMB2 = 5;
  

//serial inputs

char command;
 
void setup() {
  pinMode(brakeAmotorA1, OUTPUT); //Initiates motor 1 channel A brake on board 1
  pinMode(brakeBmotorB1, OUTPUT); // Initiates motor 2 channel B brake on board 1
 
  pinMode(dirAmotorA1, OUTPUT); // Initiates motor 1 dir A on board 1
  pinMode(dirBmotorB1, OUTPUT); // Initiates motor 1 dir B on board 1

  pinMode(brakeAmotorA2, OUTPUT); //Initiates motor 1 channel A brake on board 2
  pinMode(brakeBmotorB2 , OUTPUT); // Initiates motor 2 channel B brake on board 2

  pinMode(dirAmotorA2, OUTPUT); // Initiates motor 1 dir A on board 2
  pinMode(dirBmotorB2, OUTPUT); // Initiates motor 1 dir B on board 2

  //Initiate Serial Monitor
  Serial.begin(9600);
}
 
void loop() {
  
  //waiting for command
  if (Serial.available()>0){
    command = Serial.read();
    //incomingByte = Serial.read();
    //command=incomingByte;
    }

   //moving backwards
  if (command == 'w') {
    //Serial.println("Commencing Backward Sequence");

    // move backwards until some distance
    runA1(pwm_A1, true);
    runB1(pwm_B1, false);

    delay(backwards_time);

    //full stop
    forwardstop();

    //reset command
    command = 'z';
    Serial.println("z");  //send mega value 
    }

  //moving backwards
  if (command == 's') {
    //Serial.println("Commencing Backward Sequence");

    // move backwards until some distance
    runA1(pwm_A1, false);
    runB1(pwm_B1, true);

    delay(backwards_time);

    //full stop
    forwardstop();

    //reset command
    command = 'z';
    Serial.println("z");  //send mega value 
    }

  //shift right
  if (command == 'd') {
      //Serial.println("shift right");
  
      // spin right some distance
      runA2(pwm_A2, false);
      runB2(pwm_B2, true);

      delay(right_turning_time);

      //full stop
      shiftingstop();
      
      //reset command
      command = 'z';
      Serial.println("z");  //send mega confirmation
      }
  
  //shift left 
  if (command == 'a') {
      //Serial.println("shift left");
  
      // spin left some distance
        runA2(pwm_A2, true);
        runB2(pwm_B2, false);

        delay(left_turning_time);
          
        //full stop
        shiftingstop();
      
        //reset command
        command = 'z';
        Serial.println("z");  //send mega confirmation
      }
      
}

//motors to go forward on board 1
void runA1(int speed, boolean rev) {
  //Channel A
  if(rev) {
    //Establishes backward direction of Channel A
    digitalWrite(dirAmotorA1, LOW);
  }else {
    //Establishes forward direction of Channel A
    digitalWrite(dirAmotorA1, HIGH); 
  }
  digitalWrite(brakeAmotorA1, LOW);   //Disengage the Brake for Channel A
  analogWrite(PWMA1, speed);   //Spins the motor on Channel A at full speed
}  

void runB1(int speed, boolean rev) {
  //Channel B
  if(rev) {
    //Establishes backward direction of Channel A
    digitalWrite(dirBmotorB1, LOW);
  }else {
    //Establishes forward direction of Channel A
    digitalWrite(dirBmotorB1, HIGH); 
  }
  digitalWrite(brakeBmotorB1, LOW);   //Disengage the Brake for Channel A
  analogWrite(PWMB1, speed);   //Spins the motor on Channel A at full speed
}  

//motors to shift
void runA2(int speed, boolean rev) {
  //Channel A
  if(rev) {
    digitalWrite(dirAmotorA2, LOW); //Establishes backwards direction of Channel B
  }else{
    digitalWrite(dirAmotorA2, HIGH); //Establishes forward direction of Channel B
  }
   
  digitalWrite(brakeAmotorA2, LOW);   //Disengage the Brake for Channel B
  analogWrite(PWMA2, speed);   //Spins the motor on Channel B at full speed 
}


void runB2(int speed, boolean rev) {
  //Channel B
  if(rev) {
    digitalWrite(dirBmotorB2, LOW); //Establishes backwards direction of Channel B
  }else{
    digitalWrite(dirBmotorB2, HIGH); //Establishes forward direction of Channel B
  }
   
  digitalWrite(brakeBmotorB2, LOW);   //Disengage the Brake for Channel B
  analogWrite(PWMB2, speed);   //Spins the motor on Channel B at full speed 
}


//
//forward stop
void forwardstop(){
  digitalWrite(brakeAmotorA1, HIGH);
  digitalWrite(brakeBmotorB1, HIGH);

  runA1(0,true);
  runB1(0,true);
}

//shifting stop
void shiftingstop(){
  digitalWrite(brakeAmotorA2, HIGH);
  digitalWrite(brakeBmotorB2, HIGH);

  runA2(0,true);
  runB2(0,true);
}
