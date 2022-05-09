// arduino code to take serial commands fromm the raspberry pi and translate them into motor driver voltages

//our L298N control pins
const int LeftMotorForward = 5;
const int LeftMotorBackward = 4;
const int RightMotorForward = 3;
const int RightMotorBackward = 2;

bool hasturned = false;
double pwm = 0;

void setup() {
  Serial.begin(9600);

  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);

  delay(1000); // code rotates to wheels to show Arduino is connected properly
  forward(130);
  delay(250);
  stopNow();
  delay(1500);
  reverse(130);
  delay(250);
  stopNow();
}

void loop() {

    if(Serial.available() > 0)
    {
      
       // read the incoming byte:
       String command = Serial.readString();
       char checkserial = command.charAt(0);
       double angle = command.toDouble();

       // check serial commands :
       
       if (checkserial == 'f')
       {
        forward(110); 
        delay(250);
        forward(100); 
       }

       else if (checkserial == 'r')
       {
        reverse(130);
       }
       
       else if (checkserial == 's')
       {
        stopNow();
        delay(2000);
       }
       
       else
       {  

           if(angle < 0)
           {
             stopNow();
             delay(500);
             turn(true);
           } 
           else if(angle > 0)
           {
             stopNow();
             delay(500);
             turn(false);
           }
           
       }
       while (Serial.available() > 0) {
        Serial.read();
       }
    
}
}

// code for moving the robot

void forward(double x) {
  analogWrite(RightMotorForward, x);
  analogWrite(LeftMotorForward, x);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}

void reverse(double x) {
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  analogWrite(LeftMotorBackward, x);
  analogWrite(RightMotorBackward, x);
}

void clockwise(double x) {
  digitalWrite(RightMotorForward, LOW);
  analogWrite(LeftMotorForward, x);
  digitalWrite(LeftMotorBackward, LOW);
  analogWrite(RightMotorBackward, LOW);
}

void anticlockwise(double x) {
  analogWrite(RightMotorForward, x);
  digitalWrite(LeftMotorForward, LOW);
  analogWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}

void stopNow() {
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}

void turn(bool turnleft) {


    // increment pwm which represents the pwm voltage value supplied to motors
    // until it reaches 100 and then decrease it down to 0. 
    // Variables modified so it's roughly turning an angle of pi/4.
      
    for(int pwm = 90; pwm <= 110; pwm+=10)
    {
    
      if(turnleft)
      {
        anticlockwise(pwm);
      }
      else
      {
        clockwise(pwm);
      }
      delay(200);
    }

    stopNow();
        
    

}
