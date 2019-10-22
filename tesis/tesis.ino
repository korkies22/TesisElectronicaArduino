// PWM
int pinPwmIzqF=3;
int pinPwmIzqB=5;
int pinPwmDerF=6;
int pinPwmDerB=9;

/* -------------------------------
  Encoders
-----------------------------------*/
//Pin definition
int pinEncoderDerF=4;
int pinEncoderDerB=2;
int pinEncoderIzqB=7;
int pinEncoderIzqF=8;

#define readDerF bitRead(pinEncoderDerF,4)
#define readDerB bitRead(pinEncoderDerB,2)
#define readIzqB bitRead(pinEncoderIzqB,7)
#define readIzqF bitRead(pinEncoderIzqF,8)

//Counting variables
volatile int encoderCountIzq = 0;
volatile int encoderCountDer = 0;
int previousCountIzq = 0;
int previousCountDer = 0;

//Functions
void countEncoderDerF() {
  if(readDerF != readDerB) {
    encoderCountDer ++;
  } else {
    encoderCountDer --;
  }
}
void countEncoderDerB() {
  if (readDerF == readDerB) {
    encoderCountDer ++;
  } else {
    encoderCountDer --;
  }
}
void countEncoderIzqF() {
  if(readIzqF != readIzqB) {
    encoderCountIzq ++;
  } else {
    encoderCountIzq --;
  }
}
void countEncoderIzqB() {
  if (readIzqF == readIzqB) {
    encoderCountIzq ++;
  } else {
    encoderCountIzq --;
  }
}
void readEncoders(){
  if(encoderCountDer != previousCountDer) {
    Serial.println(encoderCountDer);
  }
  if(encoderCountIzq != previousCountIzq) {
    Serial.println(encoderCountIzq);
  }
  previousCountDer = encoderCountDer;
  previousCountIzq = encoderCountIzq;
}

/* -------------------------------
  Position
-----------------------------------*/

float curX=0;
float curY=0;
float curTheta=0;

float traveledDer=0;
float traveledIzq=0;
float ds=0;
float dTheta=0;
/* -------------------------------
  Car parameters
-----------------------------------*/
float b=0.18;
float carRadius=0.065;

/* -------------------------------
  Gradient function
-----------------------------------*/
//Function = (x-2)^2+y^2
float gradX(){
  return 2*(curX-2);
}

float gradY(){
  return 2*curY;
}

float[] grad(){
  return [-gradX(),-gradY()];
}

/* -------------------------------
  Odometry
-----------------------------------*/
void odometry(){
  updateWheelsMovement();
  calcNewPosition();
}
void updateWheelsMovement(){
  traveledDer= 2*PI*carRadius*previousCountDer/442;
  previousCountDer=0;
  traveledIzq= 2*PI*carRadius*previousCountIzq/442;
  previousCountIzq=0;
}

void calcNewPosition(){
  ds=(traveledIzq+traveledDer)/2;
  dTheta=(traveledDer-traveledIzq)/b;

  curX=curX+ds*cos(curTheta+dTheta/2);
  curY=curY+ds*sin(curTheta+dTheta/2);
  curTheta=dTheta;
}

void setup() {
  Serial.begin (115200);
  
  pinMode(pinPwmIzqF, OUTPUT);
  pinMode(pinPwmIzqB, OUTPUT);
  pinMode(pinPwmDerF, OUTPUT);
  pinMode(pinPwmDerB, OUTPUT);

  pinMode(pinEncoderDerF, INPUT_PULLUP);
  pinMode(pinEncoderDerB, INPUT_PULLUP);
  pinMode(pinEncoderIzqB, INPUT_PULLUP);
  pinMode(pinEncoderIzqF, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinEncoderDerF), countEncoderDerF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncoderDerB), countEncoderDerB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncoderIzqB), countEncoderIzqB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncoderIzqF), countEncoderIzqF, CHANGE);

}

void loop() {
  float[] gradient= grad();

  readEncoders();

  odometry();
  
}
