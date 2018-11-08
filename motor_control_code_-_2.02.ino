//time in millis to execute each of the drive functions for 13.3V battery
#define right_turning_time 510
#define left_turning_time 500
#define forwards_time 350
#define short_forwards_time 169
#define right_align_time 90
#define left_align_time 90
 
// PWM
unsigned int pwm_R = 240;
unsigned int pwm_L = 230;

//Gyroscope values
float c_angle; //current_angle
float s_angle; //straight angle
float t_angle=3; //tolerance angle
float K=1;

//serial inputs
char incomingByte;
char command;
 
void setup() {
  
  //Setup Motor Channels
  pinMode(13, OUTPUT); //Initiates Motor Channel B pin
  pinMode(8, OUTPUT);  //Initiates Brake Channel B pin
  pinMode(12, OUTPUT); //Initiates the Motor Channel A pin
  pinMode(9, OUTPUT); //Initiates the Brake channel A pin

  //Initiate Serial Monitor
  Serial.begin(9600);
}
 
void loop() {
  
  //waiting for command
  if (Serial.available()>0){
    incomingByte=Serial.read();
    command=incomingByte;
    }

  //moving forward
  if (command == 'w') {
    //Serial.println("Commencing Forward Sequence");
    
    runR(pwm_R, true);
    runL(pwm_L, false);

    delay(short_forwards_time);
    
    fs();
    
    //reset command
    command = 'z';
    Serial.println("z"); //Send serial monitor on Mega z to indicate move is done
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

