// pins for the encoder inputs
#define RH_ENCODER_A 6 
#define RH_ENCODER_B 7
#define LH_ENCODER_A 2
#define LH_ENCODER_B 4

#define travel_distance 100
#define left_turn_distance 50
#define right_turn_distance 50
#define t_interval 10
 
// Encoder variables
long RHcounter = 0;
long LHcounter = 0;

int RHState;
int RHLastState;
int LHState;
int LHLastState;

long counterarray[2];
long last_counterarray[2];

// PID Variables
unsigned int pwm_R = 100;
unsigned int pwm_L = 100;
int speed_L=0;
int speed_R=0;
int Kp=2; 
int error_L;

//serial inputs
char incomingByte;
char command;
 
void setup() {
  //Setup Encoder
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);

  //Setup Motor Channels
  pinMode(13, OUTPUT); //Initiates Motor Channel B pin
  pinMode(8, OUTPUT);  //Initiates Brake Channel B pin
  pinMode(12, OUTPUT); //Initiates the Motor Channel A pin
  pinMode(9, OUTPUT); //Initiates the Brake channel A pin

  //Set last_counterarray to zero
  last_counterarray[0]={0};
  last_counterarray[1]={0};
  

  //Initiate Serial Monitor
  Serial.begin(9600);
  Serial.println("Ready to initiate");
  Serial.println("Waiting for command");
}
 
void loop() {
  //waiting for command
  if (Serial.available()>0){
    incomingByte=Serial.read();
    command=incomingByte;
    Serial.print("Incoming Command: ");
    Serial.println(incomingByte);
    }

  counterarray[0] = {0};
  counterarray[1] = {0};
  RHcounter = 0;
  LHcounter = 0;

  //moving forward
  if (command == 'w') {
    Serial.println("Commencing Forward Sequence");

    // move forward until some distance
    runR(pwm_R, true);
    runL(pwm_L, false);
    
    while(abs(counterarray[1])<travel_distance){
      encoder();
      Serial.print("RHcounter: ");
      Serial.println(counterarray[0]);
      Serial.print("LHcounter: ");
      Serial.println(counterarray[1]);

      //RH motor --> master, LH motor --> slave
      speed_R = counterarray[0]-last_counterarray[0];
      speed_L = counterarray[1]-last_counterarray[1];
      error_L = (abs(speed_R)-abs(speed_L))*Kp;
      pwm_L = pwm_L + error_L;
      
      Serial.print("error_L: ");
      Serial.println(error_L);
      Serial.print("pwm_L: ");
      Serial.println(pwm_L);

      //store last counter value
      last_counterarray[0]=counterarray[0];
      last_counterarray[1]=counterarray[1];

      //set left motor to new values
      runL(pwm_L, false);
    }

      //full stop
      fs();
      runR(0,true);
      runL(0,true);
    
      command = 'z';
      Serial.println(counterarray[0]);
      Serial.println("Forward Sequence Complete");
    }

    //moving forward
  if (command == 's') {
    Serial.println("Commencing Backward Sequence");

    // move forward until some distance
    while(abs(counterarray[1])<travel_distance){
      runR(pwm_R, false);
      runL(pwm_L, true);
    
      encoder();
      Serial.print("RHcounter: ");
      Serial.println(counterarray[0]);
      Serial.print("LHcounter: ");
      Serial.println(counterarray[1]);
    }

      //full stop
      fs();
      runR(0,false);
      runL(0,true);
    
      command = 'z';
      Serial.println("Backward Sequence Complete");  
    }

if (command == 'd') {
    Serial.println("Commencing Right Turn");

    // spin right some distance
    while(abs(counterarray[1])<right_turn_distance){
      runR(pwm_R, false);
      runL(pwm_L, false);
    
      encoder();
      Serial.print("RHcounter: ");
      Serial.println(counterarray[0]);
      Serial.print("LHcounter: ");
      Serial.println(counterarray[1]);
    }

      //full stop
      fs();
      runR(0,true);
      runL(0,true);
    
      //reset command
      command = 'z';
      
      Serial.println("Right Turn Sequence Complete");  
    }

if (command == 'a') {
    Serial.println("Commencing Right Turn");

    // spin left some distance
    while(abs(counterarray[1])<left_turn_distance){
      runR(pwm_R, true);
      runL(pwm_L, true);
    
      encoder();
      Serial.print("RHcounter: ");
      Serial.println(counterarray[0]);
      Serial.print("LHcounter: ");
      Serial.println(counterarray[1]);
    }

      //full stop
      fs();
      runR(0,false);
      runL(0,false);
    
      //reset command
      command = 'z';
      
      Serial.println("Right Turn Sequence Complete");  
    }
}

void runR(int speed, boolean rev) {
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

void runL(int speed, boolean rev) {
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
}

//encoder code
void encoder() {
  //record start time
  int start_time=millis();
  //keep running encoder for time interval
  while(millis()-start_time<t_interval){
   RHState = digitalRead(RH_ENCODER_A ); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (RHState != RHLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(RH_ENCODER_B) != RHState) { 
       RHcounter ++;
     } else {
       RHcounter --;
     }
   } 
   RHLastState = RHState; // Updates the previous state of the outputA with the current state

   LHState = digitalRead(LH_ENCODER_A ); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (LHState != LHLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(LH_ENCODER_B) != LHState) { 
       LHcounter ++;
     } else {
       LHcounter --;
     }
   } 
   LHLastState = LHState; // Updates the previous state of the outputA with the current state
  }
  counterarray[0] = {RHcounter};
  counterarray[1] = {LHcounter};
 }



