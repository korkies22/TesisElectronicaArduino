#include <PinChangeInt.h>
/* -------------------------------
  Other parameters
-----------------------------------*/
const float pointReachedThreshold = 0.1;

bool finish = false;

bool countingTimeToEnd = false;
float timeToEnd = 0;


long timeC= 0;
long timeControl=0;

/* -------------------------------
  LEDS
-----------------------------------*/
int pinLedMoving = 10;
int pinLedFinishing = 11;
int pinLedFinished = 12;

/* -------------------------------
  PWM
-----------------------------------*/
int pinPwmIzqF = 5;
int pinPwmIzqB = 3;
int pinPwmDerF = 6;
int pinPwmDerB = 9;

const int maxPWM = 255;

int pwmValIzq = 0;
int pwmValDer = 0;

/* -------------------------------
  Encoders
-----------------------------------*/
//Pin definition
int pinEncoderDerF = 4;
int pinEncoderDerB = 2;
int pinEncoderIzqB = 7;
int pinEncoderIzqF = 8;


//Counting variables
volatile int encoderCountIzq = 0;
volatile int encoderCountDer = 0;
int previousCountIzq = 0;
int previousCountDer = 0;

long timeCountSpeedIzq=0;
long timeCountSpeedDer=0;

//Functions
void countEncoderDerF()
{
  if (digitalRead(pinEncoderDerB)==LOW)
  {
    //Serial.println("llego der2");
    encoderCountDer = 1;
  }
  else
  {
    //Serial.println("llego der3");
    encoderCountDer = -1;
  }
}
void countEncoderIzqF()
{
  //Serial.println("llego izq1");
  if (digitalRead(pinEncoderIzqB)==LOW)
  {
    encoderCountIzq = 1;
  }
  else
  {
    encoderCountIzq = -1;
  }
}
void readEncoders()
{
  if (encoderCountDer != 0)
  {
    previousCountDer += encoderCountDer;
    encoderCountDer = 0;
  }
  if (encoderCountIzq != 0)
  {
    previousCountIzq += encoderCountIzq;
    encoderCountIzq = 0;
  }
}

/* -------------------------------
  Control
-----------------------------------*/
float kr = 1;
float ka = 0.5;
float kb = 0.02;

float vRefDer = 0;
float vRefIzq = 0;

const int errorsLength = 30;

float errorIzq = 0;
float errorsIzqArray[errorsLength];
float prevErrIzq = 0;
float errorSignalIzq = 0;

float kpIzq = 0.0001;
float kiIzq = 0.0000002;
float kdIzq = 0.00002;

float errorDer = 0;
float errorsDerArray[errorsLength];
float prevErrDer = 0;
float errorSignalDer = 0;

float kpDer = 0.0001;
float kiDer = 0.0000002;
float kdDer = 0.00002;

long errorTime = 0;
long integralIndex = 0;

/* -------------------------------
  Position
-----------------------------------*/

float curX = 0;
float curY = 0;
float curTheta = 0;

long timePassedDer = 0;
long timePassedIzq = 0;
float speedIzq = 0;
float speedDer = 0;
float dsIzq = 0;
float dsDer = 0;
float ds = 0;
float dTheta = 0;

float rho = 0;
float alpha = 0;
float beta = 0;
/* -------------------------------
  Car parameters
-----------------------------------*/
float b = 0.27;
float l = b / 2;
float wheelRadius = 0.047;

/* -------------------------------
  Gradient function
-----------------------------------*/
//Function = (x-2)^2+y^2
float gradX()
{
  return 2 * (curX - 1);
}

float gradY()
{
  return 1 * curY;
}

const float finalX = 1;
const float finalY = 0;

bool reachedNewPoint = true;
float newX = 0;
float newY = 0;

/* -------------------------------
  Odometry
-----------------------------------*/
void odometry()
{
  updateWheelsMovement();
  calcNewPosition();
}
void updateWheelsMovement()
{
  if (previousCountIzq > 10 || previousCountIzq < -10)
  {
    speedIzq = 2*(float)previousCountIzq * PI * wheelRadius*1000000 / (442*(micros()-timeCountSpeedIzq));
    timeCountSpeedIzq=micros();
    //Serial.print("izq count ");
    //Serial.println(previousCountIzq);
    //Serial.print("izq speed ");
    //Serial.println(speedIzq);
    previousCountIzq = 0;
  }
  else if(micros()-timeCountSpeedIzq>100000){
    speedIzq=0;
    previousCountIzq = 0;
    timeCountSpeedIzq=micros();
  }
  if (previousCountDer > 10 || previousCountDer < -10)
  {
    speedDer = 2*(float)previousCountDer* PI * wheelRadius*1000000 / (442*(micros()-timeCountSpeedDer));
    timeCountSpeedDer=micros();
    //Serial.print("der count ");
    //Serial.println(previousCountDer);
    //Serial.print("der speed ");
    //Serial.println(speedDer);
    previousCountDer = 0;
  }
  else if(micros()-timeCountSpeedDer>100000){
    speedDer=0;
    previousCountDer = 0;
    timeCountSpeedDer=micros();
  }
}

void calcNewPosition()
{
  long timeBetweenIzq = micros() - timePassedIzq;
  timePassedIzq = micros();
  dsIzq = timeBetweenIzq * speedIzq / 1000000;

  long timeBetweenDer = micros() - timePassedDer;
  timePassedDer = micros();
  dsDer = timeBetweenDer * speedDer / 1000000;

  ds = (dsIzq + dsDer) / 2;
  dTheta = (dsDer - dsIzq) / b;

  curX += ds * cos(curTheta + dTheta / 2);
  curY += ds * sin(curTheta + dTheta / 2);
  curTheta += dTheta;

  curTheta=curTheta>PI?curTheta-PI:curTheta;
  curTheta=curTheta<-PI?curTheta+PI:curTheta;
}

void calcNewPoint(float grad[])
{
  newX = grad[0];
  newY = grad[1];
  float norm = sqrt((grad[0] * grad[0]) + (grad[1] * grad[1]));
  if (norm > 0.0001)
  {
    newX = newX / norm;
    newY = newY / norm;
  }

  newX *= 0.1;
  newY *= 0.1;
}

void calcControlVariables()
{
  float dX = newX - curX;
  float dY = newY - curY;

  rho = sqrt((dX * dX) + (dY * dY));

  if (rho < pointReachedThreshold)
  {
    reachedNewPoint = true;
    return;
  }
  alpha = atan2(dY, dX) - curTheta;
  while (alpha > PI)
  {
    alpha -= 2 * PI;
  }
  while (alpha <= -PI)
  {
    alpha += 2 * PI;
  }
  beta = -alpha - curTheta;
  while (beta > PI)
  {
    beta -= 2 * PI;
  }
  while (beta <= -PI)
  {
    beta += 2 * PI;
  }
}

void calcRefVelocities()
{
  float v = kr * rho;
  float w = ka * alpha + kb * beta;

  vRefDer = v - l * w;
  vRefIzq = v - l * w;
}

void controlOdometry()
{
  calcControlVariables();
  if (reachedNewPoint)
    return;
  calcRefVelocities();
}

void control()
{
  errorIzq = vRefIzq*1000 - speedIzq*1000;
  errorDer = vRefDer*1000 - speedDer*1000;

  long deltaErrTime = micros() - errorTime;
  deltaErrTime = deltaErrTime;
  errorTime = micros();

  float errorTimedIzq=errorIzq * deltaErrTime;
  float errorTimedDer=errorDer * deltaErrTime;

  if(errorTimedIzq<0.001 && errorIzq>0.01){
    errorTimedIzq=0.001;
  }

  if(errorTimedDer<0.001 && errorDer>0.01){
    errorTimedDer=0.001;
  }

  errorsIzqArray[integralIndex] = errorTimedIzq;
  errorsDerArray[integralIndex] = errorTimedDer;
  integralIndex++;
  if (integralIndex == errorsLength)
  {
    integralIndex = 0;
  }
  float integralErrorIzq = 0;
  float integralErrorDer = 0;
  for (int i = 0; i < errorsLength; i++)
  {
    integralErrorIzq += errorsIzqArray[i];
  }
  for (int i = 0; i < errorsLength; i++)
  {
    integralErrorDer += errorsDerArray[i];
  }
  float errorIzqDerivative = (errorIzq - prevErrIzq) / deltaErrTime;
  float errorDerDerivative = (errorDer - prevErrDer) / deltaErrTime;
  prevErrIzq = errorIzq;
  prevErrDer = errorDer;
  errorSignalIzq = kpIzq * errorIzq + kiIzq * integralErrorIzq + kdIzq * errorIzqDerivative;
  errorSignalDer = kpDer * errorDer + kiDer * integralErrorDer + kdDer * errorDerDerivative;

  if (errorSignalIzq < 0.1 && errorSignalIzq > -0.1)
  {
    errorSignalIzq = 0;
  }
  if (errorSignalDer < 0.1 && errorSignalDer > -0.1)
  {
    errorSignalDer = 0;
  }

  pwmValIzq += errorSignalIzq;
  pwmValDer += errorSignalDer;

  
  
  if(micros()-timeC>10000){
    Serial.print("curX ");
    Serial.println(curX);
    Serial.print("curY ");
    Serial.println(curY);
    Serial.print("curTheta ");
    Serial.println(curTheta);
    timeC=micros();
    Serial.print("Ñam ");
    Serial.println(vRefIzq);
    Serial.print("Ñam1.5 ");
    Serial.println(speedIzq);
    Serial.print("Ñam2 ");
    Serial.println(errorIzq);
  }

}

void moveCar()
{

  int curPinPwmIzq = pwmValIzq >= 0 ? pinPwmIzqF : pinPwmIzqB;
  int curPinPWMOffIzq = pwmValIzq >= 0 ? pinPwmIzqB : pinPwmIzqF;
  int curPWMValIzq = pwmValIzq >= 0 ? pwmValIzq : -pwmValIzq;

  if (vRefIzq == 0)
  {
    curPWMValIzq = 0;
  }

  curPWMValIzq = curPWMValIzq > maxPWM ? maxPWM : curPWMValIzq;

  int curPinPwmDer = pwmValDer >= 0 ? pinPwmDerF : pinPwmDerB;
  int curPinPWMOffDer = pwmValDer >= 0 ? pinPwmDerB : pinPwmDerF;
  int curPWMValDer = pwmValDer >= 0 ? pwmValDer : -pwmValDer;

  if (vRefDer == 0)
  {
    curPWMValDer = 0;
  }

  curPWMValDer = curPWMValDer > maxPWM ? maxPWM : curPWMValDer;

  analogWrite(curPinPWMOffIzq, 0);
  analogWrite(curPinPwmIzq, curPWMValIzq);

  analogWrite(curPinPWMOffDer, 0);
  analogWrite(curPinPwmDer, curPWMValDer);

 /* if(micros()-timeC>10000){
    Serial.print("curX ");
    Serial.println(curX);
    Serial.print("curY ");
    Serial.println(curY);
    Serial.print("curTheta ");
    Serial.println(curTheta);
    timeC=micros();
    Serial.print("Ñam ");
    Serial.println(curPWMValIzq);
    Serial.print("Ñam2 ");
    Serial.println(curPinPwmIzq);
  }*/
}

float calcDistanceToEnd()
{
  float dX = finalX - curX;
  float dY = finalY - curY;
  float norm = sqrt((dX * dX) + (dY * dY));
  return norm;
}


void stopCar()
{
  analogWrite(pinPwmIzqB, 0);
  analogWrite(pinPwmIzqF, 0);

  analogWrite(pinPwmDerB, 0);
  analogWrite(pinPwmDerF, 0);
}

void setup()
{
  Serial.begin(115200);

  pinMode(pinPwmIzqF, OUTPUT);
  pinMode(pinPwmIzqB, OUTPUT);
  pinMode(pinPwmDerF, OUTPUT);
  pinMode(pinPwmDerB, OUTPUT);

  pinMode(pinLedMoving, OUTPUT);
  pinMode(pinLedFinishing, OUTPUT);
  pinMode(pinLedFinished, OUTPUT);

  pinMode(pinEncoderDerF, INPUT);
  pinMode(pinEncoderDerB, INPUT);
  pinMode(pinEncoderIzqB, INPUT);
  pinMode(pinEncoderIzqF, INPUT);
  digitalWrite(pinEncoderIzqB, HIGH);
  digitalWrite(pinEncoderIzqF, HIGH);
  digitalWrite(pinEncoderDerF, HIGH);
  digitalWrite(pinEncoderDerB, HIGH);

  PCintPort::attachInterrupt(pinEncoderDerF, countEncoderDerF, RISING);
  PCintPort::attachInterrupt(pinEncoderIzqF, countEncoderIzqF,RISING); // attach a PinChange Interrupt to our pin on the rising edge

  while (Serial.available() <= 0) {
    // read the incoming byte:
    int incomingByte = Serial.read();
    if(incomingByte>0) break;
  }
  timeCountSpeedIzq=micros();
  timeCountSpeedDer=micros();

  timeC = micros();
  Serial.println("Empezó");

}
void loop()
{
 
  if (finish)
    return;
  readEncoders();

  odometry();

  float trueGradX = -gradX();
  float trueGradY = -gradY();

  float grad[2] = {trueGradX, trueGradY};

  if (reachedNewPoint)
  {
    reachedNewPoint = false;
    calcNewPoint(grad);
  }

  controlOdometry();

  if(micros()-timeControl>100000){
    control();
  }
  moveCar();
  if (calcDistanceToEnd() < 0.1)
  {
    digitalWrite(pinLedFinishing, HIGH);
    if (countingTimeToEnd == false)
    {
      countingTimeToEnd = true;
      timeToEnd = micros();
    }
    else if (micros() - countingTimeToEnd > 1000000)
    {
      finish = true;
      digitalWrite(pinLedFinished, HIGH);
      stopCar();
    }
  }
  else
  {
    countingTimeToEnd = false;
    digitalWrite(pinLedFinishing, LOW);
  }
}
