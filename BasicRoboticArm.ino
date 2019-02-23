// LIBRARIES
#include <math.h>
#include <Servo.h> 

// OBJECTS
Servo servoMotor1;
Servo servoMotor2;
Servo servoMotor3;

// VARIABLES

// pin potentiometers are attached
double potentiometerXpin = A0;
double potentiometerYpin = A1;
double potentiometerZpin = A2;

// potentiometers signals values
double potentiometerXsignal = 0;
double potentiometerYsignal = 0;
double potentiometerZsignal = 0;

double currentPotentiometerYsignal = 0;

String messageToComputer;

boolean positionPossible = true;

// P point to reach coordonates
double pX = 0;
double pY = 0;
double pZ = 0;

// Robot articulations angles
double a1;
double a2;
double a3;

double currentA1 = 90;
double currentA2 = 135;
double currentA3 = 109;

// Robot articulations length
double d1 = 72;
double d2 = 93.3;

// variables to make computations
double dx = 0;
double pW = 0;
double aW1 = 0;
double aW2 = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(potentiometerXpin, INPUT);
  pinMode(potentiometerYpin, INPUT);
  pinMode(potentiometerZpin, INPUT);

  // motors
  servoMotor1.attach(9);
  servoMotor2.attach(10);
  servoMotor3.attach(11);
}

void loop() {
  
  // STEP 1 : read the P point coordonates from potentiometers

  pX = analogRead(potentiometerXpin) * 0.5 - 255; // Get a value from -255 to 255
  pY = analogRead(potentiometerYpin) * 0.5 - 255;
  pZ = analogRead(potentiometerZpin) * 0.5 - 255;
  
  if(pY < 30){
    pY = 30;
  }
  
  // STEP 2 : compute articulations angles
  
  //-- a1 --
  a1 = atan2(pY, pX);
  
  
  //-- a2 --
  pW = sqrt(sq(pX) + sq(pY));
  dx = sqrt(sq(pZ) + sq(pW));

  positionPossible = true;
  if(dx > d1 + d2){ // impossible situation beacause d1, d2 and dx have to make a triangle
    positionPossible = false;
  }
  
  aW1 = atan2(pW, pZ);

  if(pW < 0){
    aW1 *= -1;
  }

  if(positionPossible){
    aW2 = acos((sq(d1) + sq(dx) - sq(d2)) / (2 * d1 * dx));
  }
  
  a2 = aW1 + aW2;

  //-- a3 --
  if(positionPossible){
    a3 = acos((sq(d2) + sq(d1) - sq(dx)) / (2 * d2 * d1));
  }


  // STEP 3 : Convert angles to PWM signals
  if(positionPossible){
  
    //-- Servomotor 1 --
    a1 = a1 * 57.3; // radian to degree
    if(a1 < 0 && a1 > -90){
      a1 = 0;
    }
    if(a1 < -90){
      a1 = 180; 
    }
    currentA1 = currentA1 * 0.5 + 0.5 * a1; // filter
    potentiometerXsignal = map(currentA1, 0, 180, 800, 2400);  
    servoMotor1.writeMicroseconds(potentiometerXsignal);
  
    //-- Servomotor 2 --
    a2 = a2 * 57.3; // radian to degree
    currentA2 = currentA2 * 0.5 + 0.5 * a2; // filter
    potentiometerYsignal = map(currentA2, 45, 225, 700, 2200); // for 45° signal = 700; for 225° signal = 2200
    servoMotor2.writeMicroseconds(potentiometerYsignal);
  
    //-- Servomotor 3 --
    a3 = a3 * 57.3; // radian to degree
    currentA3 = currentA3 * 0.5 + 0.5 * a3; // filter
    potentiometerZsignal = map(a3, 19, 199, 700, 2300);
    servoMotor3.writeMicroseconds(potentiometerZsignal);
  }

  // send datas to computer
  messageToComputer =
    String(pX) + ',' +
    String(pY) + ',' +
    String(pZ) + ',' +
    String("-") + ',' +
    String(d1) + ',' +
    String(d2) + ',' +
    String(dx) + ',' +
    String("-") + ',' +
    String(a1) + ',' +
    String(a2) + ',' +
    String(a3) + ',' +
    String("-") + ',' +
    String(aW1) + ',' +
    String(aW2);
    
  Serial.println(messageToComputer);

  delay(10);
}
