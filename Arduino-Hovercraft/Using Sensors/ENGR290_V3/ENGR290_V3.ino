//#include <avr/io.h>
#include <ServoTimer2.h> 
#include <TimerOne.h>

//Flags                           //The flags help define machine states. The HC does not behave the same way in a straight line and in a corner
boolean turn_flag = false; 

//Library objects
ServoTimer2 thrustServo;          //Create object to control a servo-motor on timer2 

//Global variables and control variables 

//Servo control variables
int servo_pos = 90;                    //Variable will hold the desired servo position
int turn_left = 1750;                  
int turn_right = 1250;
String heading = "Forward";            //Variable will hold the current state of the robot. Debug purposes

//Distance control variables
int min_front_distance = 30;          
int min_side_distance = 10;

//Ultrasonic sensor variables
const int numReadings = 10;               //Number of readings computed in the running  average
volatile int front_index = 0;             //Index variable for front sensor
volatile int side_index = 0;              //index varaible for side sensor

float front_buffer[numReadings];          //Buffer for front sensor readings
float side_buffer[numReadings];           //Buffer for side sensor readings

volatile float front_start;               //Front sensor pulse timestamp
volatile float front_stop;

volatile float side_start;                //Side sensor pulse timestamp
volatile float side_stop;

volatile float front_distance;            //Calculated front distance (latest reading)
volatile float av_front_distance;         //Running average of the last numReadings front readings
volatile float side_distance;             //Calculated side distance (latest reading)
volatile float av_side_distance;          //Running average of the last numReadings side readings

//Pin assignments
int lift_fan = 8;             //Lift fan = arduino pin 8
int thrust_fan = 7;           //Thrust fan = arduino pin 6
int front_sensor = 2;         //Front sensor is connected to interrupt capable pin 2 (PD2)
int side_sensor = 3;          //Side sensor is connected to interrupt capable pin 3 (PD3)

void setup() {
  //Serial port initialization
  Serial.begin(9600);

  //Timer initializing (For timed interrupts)
  Timer1.initialize(100000);   //Timer based interrupt is triggered every 0.1 sec. Evderytime the timer reaches the value store in the compare register, an ISR is invoked
                               //The ISR controls the refresh rate of the sensor. Min refresh rate = 20/s
                                                                                                   
  //Initial settings for the servo-motor (PD6 => PW2)
  thrustServo.attach(6);        //Attach the Arduino PWM capable pin to the servo object. Now the object controls the pin.
  
  //Initial settings port for thrust fan (ON/OFF switch1) PD7
  pinMode(thrust_fan, OUTPUT);         //Initialize port as output (PD6)
  digitalWrite(thrust_fan, HIGH);      //thrust_fan is HIGH on startup.
                                       
  //Initial setting for the lift fan (ON/OFF switch1) PB0
  pinMode(lift_fan, OUTPUT);           //Initialize port as output (PB0)
  digitalWrite(lift_fan, HIGH);        //Lift fan port is HIGH on startup. Speed is always MAX
   
  //Interrupt definitions
  attachInterrupt(0, front_sensor_isr, CHANGE);     //Interrupt when an input change occurs on front sensor pin
  attachInterrupt(1, side_sensor_isr, CHANGE);      //Interrupt when an input change occurs on side sensor pin
  Timer1.attachInterrupt(update_buffer);            //Attached an interupt routine on timer one
}

void loop() {
// HC is approaching a wall and must turn
  //Serial.print(av_front_distance);
  //Serial.print("         ");
  //Serial.println(av_side_distance);
  
  while(av_front_distance <= min_front_distance){
    // if there is no wall to the right
    Serial.println("Front sensor tripped");

    if(av_side_distance >= min_side_distance){
      // we must turn right
      Serial.println("Turn right");
      thrustServo.write(1250); 
    }
    else{
      // we just turn left
      Serial.println("Turn left");
      thrustServo.write(1750); 
      }
  }
  // after turn or if no turn is taken, go back straight
  Serial.println("Going straight");
  thrustServo.write(1500);   
}


//Interrupt Service Routines
void update_buffer(){                                 
  av_front_distance = front_average();
  av_side_distance = side_average();
}

void front_sensor_isr(){                      //External interrupt 0 = Side sensor
  if(digitalRead(front_sensor) == HIGH){
    front_start = micros(); 
  }
  else{
    front_stop = micros();
    front_distance = ((front_stop - front_start)/147);
  }
}

void side_sensor_isr(){                       //External interrupt 1 = Side sensor 
  if(digitalRead(side_sensor) == HIGH){
    side_start = micros();  
  }
  else{
    side_stop = micros();
    side_distance = ((side_stop - side_start)/147);
  }
}

//Functions
float front_average(){                              //Function takes the running average of each sensor readings to smooth it
   front_buffer[front_index] = front_distance;      //This eliminates jitters and unstability in the readings
                                                    
   if(front_index == (numReadings - 1)){            //Index manipulation => FIFO
      front_index = 0;
    }
    else{
    front_index++;                                                                    
    }
    float temp = 0;
    //Compute the distance as the average of the last numReadings readings
    for(int i = 0; i < numReadings; i++){
      temp = temp + front_buffer[i];
    }   
    return (temp / numReadings);
}

float side_average(){
   side_buffer[side_index] = side_distance;
   //Serial.print(side_distance);
   if(side_index == (numReadings - 1)){            //Index manipulation => FIFO
      side_index = 0;
    }
    else{
    side_index++;                                                                    
    }
    float temp = 0;
    //Compute the distance as the average of the last 5 readings
    for(int i = 0; i < numReadings; i++){

      temp = temp + side_buffer[i];
    } 
    return (temp / numReadings);
}
