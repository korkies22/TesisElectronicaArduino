#include <PinChangeInt.h>
/* -------------------------------
  Other parameters
-----------------------------------*/
const float pointReachedThreshold = 0.05;
long timeToNewPoint=0;

bool finish = false;

bool countingTimeToEnd = false;
long timeToEnd = 0;


long timeControl=0;

/* -------------------------------
  LEDS
-----------------------------------*/

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
float kr = 2;
float ka = 1;
float kb = 0.04;

float vRefDer = 0;
float vRefIzq = 0;

const int errorsLength = 30;

float errorIzq = 0;
float prevErrIzq = 0;
float errorSignalIzq = 0;

float kpIzq = 0.2;
float kiIzq = 2;
float kdIzq = 0.02;

float errorDer = 0;
float prevErrDer = 0;
float errorSignalDer = 0;

float kpDer = 0.2;
float kiDer = 2;
float kdDer = 0.02;

float integralErrorIzq = 100/kiIzq;
float integralErrorDer = 100/kiDer;

long errorTime = 0;

/* -------------------------------
  Position
-----------------------------------*/

float curX = 0;
float curY = 0;
float curTheta = 0;

float v=0;
float w=0;

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
float b = 0.22;
float l = b / 2;
float wheelRadius = 0.034;

/* -------------------------------
  Gradient function
-----------------------------------*/

float grad[2] = {0,0};
//Function = (x-2)^2+y^2 Le faltan -7
//3d plot (x-2)^2+y^2+10*e^(-((x-0.3)^2+(y-0.5)^2)*2)+12*e^(-((x-1.2)^2+(y+0.7)^2)*2) from -1 to 3
float gradX()
{
  return 2*(curX+1);
  //return 4*(curX-2) -100*(curX)*exp(-5*(pow(curX,2) + pow(curY - 0.7,2))) -120*(curX)*exp(-5*(pow(curX,2) + pow(curY + 1,2))) -120*(curX - 1.2)*exp(-5*(pow(curX - 1.2,2) + pow(curY + 1,2)));
}

float gradY()
{
  return 2*(curY);
  //return 4*(curY )  -100*(curY - 0.7)*exp(-5*(pow(curX,2) + pow(curY - 0.7,2))) -120*(curY +1)*exp(-5*(pow(curX,2) + pow(curY + 1,2))) -120*(curY + 1)*exp(-5*(pow(curX - 1.2,2) + pow(curY + 1,2)));
}

const float finalX = -1;
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

  curTheta=curTheta>2*PI?curTheta-2*PI:curTheta;
  curTheta=curTheta<-2*PI?curTheta+2*PI:curTheta;
}

void calcNewPoint()
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



  newX+=curX;
  newY+=curY;
}

void calcControlVariables()
{
  float dX = newX - curX;
  float dY = newY - curY;

  rho = sqrt((dX * dX) + (dY * dY));

  if (rho < pointReachedThreshold || micros()-timeToNewPoint>1000000)
  {
    timeToNewPoint=micros();
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
  v = kr * rho;
  w = ka * alpha + kb * beta;

  vRefDer = (v +l * w);
  vRefIzq = v - l * w;

  if(vRefIzq>0.3) vRefIzq=0.3;
  if(vRefIzq<-0.3) vRefIzq=-0.3;

  if(vRefDer>0.3) vRefDer=0.3;
  if(vRefDer<-0.3) vRefDer=-0.3;
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

  integralErrorIzq+= errorIzq/10000;
  integralErrorDer+= errorDer/10000;
  float errorIzqDerivative = (errorIzq - prevErrIzq);
  float errorDerDerivative = (errorDer - prevErrDer);
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

  pwmValIzq = errorSignalIzq;
  pwmValDer = errorSignalDer;

  
  
 /* if(micros()-timeC>300000){
    Serial.print("curX ");
    Serial.println(curX);
    Serial.print("curY ");
    Serial.println(curY);
    Serial.print("curTheta ");
    Serial.println(curTheta*180/PI,4);
    //Serial.print("dsIzq ");
    //Serial.println(dsIzq,7);
    //Serial.print("pwmValDer ");
    //Serial.println(pwmValDer,7);
    //Serial.print("Ñam ");
    //Serial.println(vRefIzq);
     //Serial.print("Ñam1.1 ");
    //Serial.println(speedIzq);
    //Serial.print("Ñam1.5 ");
    //Serial.println(integralErrorIzq);
    //Serial.print("Ñam1.6 ");
    //Serial.println(errorIzq);
    //Serial.print("Ñam1.7 ");
    //Serial.println(errorDerDerivative);
    //Serial.print("Ñam2 ");
    //Serial.println(errorSignalIzq);
    //Serial.print("Ñam3 ");
    //Serial.println(pwmValIzq);
    //Serial.print("Rho ");
    //Serial.println(rho);
    //Serial.print("Alpha ");
    //Serial.println(alpha*180/PI);
    //Serial.print("Beta ");
    //Serial.println(beta*180/PI);
    //Serial.print("New X ");
    //Serial.println(newX);
    //Serial.print("New Y ");
    //Serial.println(newY);
    Serial.print("GradX ");
    Serial.println(grad[0]);
    Serial.print("GradY ");
    Serial.println(grad[1]);
    //Serial.print("w ");
    //Serial.println(w,4);
    //Serial.print("v ");
    //Serial.println(v,4);
    //Serial.print("vRefDer ");
    //Serial.println(vRefDer,4);
    //Serial.print("vRefIzq ");
    //Serial.println(vRefIzq,4);
    
    
    timeC=micros();
  } */

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

  timeToNewPoint=micros();
  //Serial.println("Empezó");

}
int cycles=0;
void loop()
{ 
  if (finish)
    return;
  readEncoders();

  odometry();

  float trueGradX = -gradX();
  float trueGradY = -gradY();

  grad[0] = trueGradX;
  grad[1] = trueGradY;

  if (reachedNewPoint)
  {
    reachedNewPoint = false;
    calcNewPoint();
  }

  controlOdometry();

  if(micros()-timeControl>400000){
    control();
  }
  moveCar();
  if (calcDistanceToEnd() < 0.1)
  {
    //digitalWrite(pinLedFinishing, HIGH);
    if (countingTimeToEnd == false)
    {
      countingTimeToEnd = true;
      timeToEnd = micros();
      //Serial.println("counting end");
    }
    else if (micros() - timeToEnd > 500000)
    {
      finish = true;
      //digitalWrite(pinLedFinished, HIGH);
      stopCar();
     /* Serial.println("Finished");
      Serial.print("CurX ");
      Serial.println(curX);
      Serial.print("CurY ");
      Serial.println(curY);*/
    }
  }
  else
  {
    countingTimeToEnd = false;
    //digitalWrite(pinLedFinishing, LOW);
  }
}
