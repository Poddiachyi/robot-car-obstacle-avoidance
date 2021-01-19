#include <Wire.h>   
#include <Servo.h>

Servo servo;  
                                              
int pos = 0; 

int inputPin = A0;  // ultrasonic module   ECHO to A0
int outputPin = A1;  // ultrasonic module  TRIG to A1 

int servoPin = 3; 
int turningAngle = 3;


const int numAngles = 3;
int angles[numAngles] = {0, 90, 180};  // only three directions for now

int* lookAround();
int lookInDirection(int angle);
void resetServo();


void setup() {
  Serial.begin(9600);
  
  servo.attach(servoPin); 
  
  pinMode(inputPin, INPUT);      
  pinMode(outputPin, OUTPUT); 
}

void loop() {
  int *obs = lookAround();

  if (obs[1] > 10) {
    Serial.println("Moving forward");
  } else {
    if (obs[0] >= obs[2]) {
      Serial.println("Moving right");
    } else {
      Serial.print("Moving left");
    }
  }
  
//  for(int i = 0; i < numAngles; i++) {
//    Serial.println("Agnle is " + String(angles[i]));
//    Serial.println("Distance is " + String(obs[i]));
//  }

  Serial.println();
  delay(5000);
  delete[] obs;
}


int* lookAround() {
  resetServo();

  // length equals to number of angles, values are distances in diff directions
  int *observations = new int[numAngles]; 
  
  for(int i = 0; i < numAngles; i++) {
    int distance = lookInDirection(angles[i]);
    observations[i] = distance;
  }
  
  resetServo();

  return observations;
}


int lookInDirection(int angle) {
  servo.write(angle);
  delay(300);

  // this is how sound pulse works https://bit.ly/3iruAAs
  digitalWrite(outputPin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(outputPin, HIGH); 
  delayMicroseconds(10); 

  float pulseTime = pulseIn(inputPin, HIGH);
  int distance = (int) pulseTime * 0.034 / 2; // convertion of time to dist in cm

  return distance;
}

void resetServo() {
//  Serial.println("Reseting");
  servo.write(90);
  delay(300);
}
