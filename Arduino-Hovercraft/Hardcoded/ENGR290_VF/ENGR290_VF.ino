#include <avr/io.h>
#include <Servo.h>
#include <TimerOne.h>

// global timer variables
int timer1 = 7000;
int timer2 = 6000;

// directions
int straight = 90;
int left = 0;
int right = 180;

//Library objects
Servo servo1;

//Pin assignments
int lift_fan = 8;             //Lift fan = arduino pin 8
int thrust_fan = 6;           //Thrust fan = arduino pin 6 //onoff is pin 7

void setup() {
  //Serial port initialization
  Serial.begin(9600);
                                                                      
  //Initial settings for the servo-motor (PD6 => PW2)
  servo1.attach(9);
     
  //Initial settings port for thrust fan (ON/OFF switch1) PD7
  pinMode(thrust_fan, OUTPUT);         //Initialize port as output (PD6)
  analogWrite(thrust_fan, 220);        // Run fan at 220 / 255
  //digitalWrite(thrust_fan, HIGH);    // If thrust is connected to ON/OFF 1  
                                       
  //Initial setting for the lift fan (ON/OFF switch1) PB0
  pinMode(lift_fan, OUTPUT);           //Initialize port as output (PB0)
  digitalWrite(lift_fan, HIGH);        //Lift fan port is HIGH on startup. Speed is always MA
}

void loop() {
// go straight until we hit wall
    servo1.write(straight);
    delay(timer1);
// turn right
    servo1.write(right);
    delay(timer1);            // 7s
// go straight until we hit wall
    servo1.write(straight);
    delay(timer2);            // 6s
//  turn left
    servo1.write(left);
    delay(timer1);            // 7s
}
