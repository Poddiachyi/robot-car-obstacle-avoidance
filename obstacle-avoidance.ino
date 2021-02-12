#include <Wire.h>   
#include <Servo.h>

Servo servo;  
                                              
int inputPin = A0;  // ultrasonic module   ECHO to A0
int outputPin = A1;  // ultrasonic module  TRIG to A1 

int servoPin = 3; 
int turningAngle = 3;

int wheelLF = 8;
int wheelLB = 7;
int wheelRF = 4;
int wheelRB = 2;

int leftSideENA = 13; // allows to control speed
int rightSideENA = 5;

const int numAngles = 3;
int angles[numAngles] = {0, 90, 180};  // only three directions for now

int* lookAround();
int lookInDirection(int angle);
void resetServo();
int chooseAction(int *obs);
void pinWheels();
void setWheelsSpeed();

void moveForward();
void moveBackward();
void turn90Right();
void turn90Left();
void stopMoving();


void setup() {
  Serial.begin(9600);

  pinServoAndUltrSonic();
  pinWheels();
  setWheelsSpeed();

}

void loop() {

  int *obs = lookAround();

  int action = chooseAction(obs);
  Serial.println("Action is " + String(action));

  Serial.println();
  delay(5000);
  delete[] obs;
}

void moveForward() {
  digitalWrite(wheelRF, HIGH);
  digitalWrite(wheelRB, LOW);

  digitalWrite(wheelLF, HIGH);
  digitalWrite(wheelLB, LOW);
}


void moveBackward() {
  digitalWrite(wheelRF, LOW);
  digitalWrite(wheelRB, HIGH);

  digitalWrite(wheelLF, LOW);
  digitalWrite(wheelLB, HIGH);
}

void stopMoving() {
  digitalWrite(wheelRF, HIGH);
  digitalWrite(wheelRB, HIGH);

  digitalWrite(wheelLF, HIGH);
  digitalWrite(wheelLB, HIGH);
}

void turn90Left() {
  digitalWrite(wheelRF, HIGH);
  digitalWrite(wheelRB, LOW);

  digitalWrite(wheelLF, LOW);
  digitalWrite(wheelLB, HIGH);

  delay(470); // for completing the maneuver
  stopMoving();
}

void turn90Right() {
  digitalWrite(wheelRF, LOW);
  digitalWrite(wheelRB, HIGH);

  digitalWrite(wheelLF, HIGH);
  digitalWrite(wheelLB, LOW);

  delay(700); // for completing the maneuver
  stopMoving();
}


void pinServoAndUltrSonic() {
  servo.attach(servoPin); 
  pinMode(inputPin, INPUT);      
  pinMode(outputPin, OUTPUT);   
}


void pinWheels() {
  pinMode(wheelLF, OUTPUT);
  pinMode(wheelLB, OUTPUT);
  pinMode(wheelRF, OUTPUT);
  pinMode(wheelRB, OUTPUT);

  pinMode(leftSideENA, OUTPUT);  
  pinMode(rightSideENA, OUTPUT); 
}


void setWheelsSpeed() {
  analogWrite(rightSideENA, 250);
  analogWrite(leftSideENA, 250);  
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
  servo.write(90);
  delay(300);
}
