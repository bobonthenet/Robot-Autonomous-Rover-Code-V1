//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Robot Code v1.0                                                                                                          //
// Written by Robert Bedard                                                                                                 //
// Originally written on: July 8th, 2011                                                                                    //
// Last Edited on: July 8th, 2011                                                                                           //
// Last tried to take over the world on:                                                                                    //
// A robot may not injure a human being or, through inaction, allow a human being to come to harm.                          //
// A robot must obey any orders given to it by human beings, except where such orders would conflict with the First Law.    //
// A robot must protect its own existence as long as such protection does not conflict with the First or Second Law.        //
//                                                                                                                          //
// Change Log:  7/13/2011 - Added new function for basic heuristic learning capabilities -->  heuristics()                  //
//                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#include <Servo.h> //Includes servo.h
 
Servo myservo;  // create servo object to control a servo 
int IR_SENSOR = 0; //IR sensor reading Higher values = closer.
int pwm_a = 3;  //PWM control for motor outputs 1 and 2 is on digital pin 3
int pwm_b = 11;  //PWM control for motor outputs 3 and 4 is on digital pin 11
int dir_a = 12;  //direction control for motor outputs 1 and 2 is on digital pin 12
int dir_b = 13;  //direction control for motor outputs 3 and 4 is on digital pin 13
int turn = 1;    //This variable will be used to determine which way to turn.
int distleft = 0;  // stores distance when looking to the left.
int distright = 0;  // stores distance when looking to the right.
int IrVal = 0;  //This is the distance measured by the IR sensor.  Higher values = closer.
int leftmopowa = 255; //Variable for storing motor duty cycle.
int rightmopowa = 255; //Variable for storing motor duty cycle.
int rightturns = 0;
int leftturns = 0;

void heuristics()  //Extremely simple learning function to help the robot drive straight.
{
	if(rightturns > leftturns)  //If the robot is making more right turns than left turns
		leftmopowa--;  //slow the left motor down.
	else
		leftmopowa++;  //speed the left motor up.
}

void findnewroute()  //This function is used for determining which way to go when an obstacle is detected.
{ 
    analogWrite(pwm_a, 0);  //set both motors to run at (100/255 = 39)% duty cycle (slow)
    analogWrite(pwm_b, 0); 
  
    myservo.write(5);              // tell servo to go to position 45. 
    delay(1000);                       // waits 1000ms for the servo to reach the position
    distright = analogRead(IR_SENSOR);  //get IR reading when looking left.
                                     
    myservo.write(170);              // tell servo to go to position 125. 
    delay(1000);                       // waits 1000ms for the servo to reach the position 
    distleft = analogRead(IR_SENSOR);  //get IR reading when looking right.

    myservo.write(90);  //tell servo to go to position 90, look forward.

    if(distleft < distright)  //Decides which way to turn.
        turn = 1;
    else
        turn = 0;
    
    return;
}

void waytoturn()  //Turns the robot.
{
  
  if(turn == 1)
        {
            analogWrite(pwm_a, leftmopowa);  //set both motors to run at (100/255 = 39)% duty cycle (slow)
            analogWrite(pwm_b, rightmopowa); 
            digitalWrite(dir_a, HIGH);  //Set motor direction, 1 low, 2 high
            digitalWrite(dir_b, LOW);  //Set motor direction, 3 high, 4 low
            delay(900);   //Turns for this long.
			rightturn++;
        }
    else
        {
            analogWrite(pwm_a, leftmopowa);  //set both motors to run at (100/255 = 39)% duty cycle (slow)
            analogWrite(pwm_b, rightmopowa); 
            digitalWrite(dir_a, LOW);  //Reverse motor direction, 1 high, 2 low
            digitalWrite(dir_b, HIGH);  //Reverse motor direction, 3 low, 4 high
            delay(900);  //Turns for this long.
			leftturn++;
        }

}

void moveforward()
{
digitalWrite(dir_a, LOW);  //Set motor direction
digitalWrite(dir_b, LOW);  //Set motor direction
  analogWrite(pwm_a, leftmopowa);  //set both motors to run at (100/255 = 39)% duty cycle (slow)
  analogWrite(pwm_b, rightmopowa); 
}



void setup() 
{
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  pinMode(pwm_a, OUTPUT);  //Set control pins to be outputs
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);
  
  analogWrite(pwm_a, leftmopowa);  //set both motors to run at (100/255 = 39)% duty cycle (slow)
  analogWrite(pwm_b, rightmopowa);
  myservo.write(90);    //tell servo to go to position 90, look forward.
  Serial.begin(9600);   //Starts serial output, really only used for testing when the robot is plugged into a PC.
}
 
void loop() //Main Loop.
{
  IrVal = analogRead(IR_SENSOR);    
  Serial.println(IrVal);

  while (IrVal < 325)
    {
    moveforward();
    IrVal = analogRead(IR_SENSOR);    
    Serial.println(IrVal);
    }
  findnewroute();
  waytoturn();
}



