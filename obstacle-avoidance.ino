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
int chooseAction(int *obs);


void setup() {
  Serial.begin(9600);
  
  servo.attach(servoPin); 
  
  pinMode(inputPin, INPUT);      
  pinMode(outputPin, OUTPUT); 
}

void loop() {
  int *obs = lookAround();

  int action = chooseAction(obs);
  Serial.println("Action is " + String(action));

  Serial.println();
  delay(5000);
  delete[] obs;
}


int chooseAction(int *obs) {
  int action = -1;
  if (obs[1] > 10) {
    Serial.println("Moving forward");
    action = 0;
  } else {
    if (obs[0] >= obs[2]) {
      Serial.println("Moving right");
      action = 1;
    } else {
      Serial.println("Moving left");
      action = 2;
    }
  }
  return action;
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
