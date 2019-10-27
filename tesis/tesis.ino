/* -------------------------------
  Other parameters
-----------------------------------*/
const float pointReachedThreshold = 0.1;

bool finish = false;

bool countingTimeToEnd=false;
float timeToEnd=0;

/* -------------------------------
  LEDS
-----------------------------------*/
int pinLedMoving=10;
int pinLedFinishing=11;
int pinLedFinished=12;

/* -------------------------------
  PWM
-----------------------------------*/
int pinPwmIzqF = 3;
int pinPwmIzqB = 5;
int pinPwmDerF = 6;
int pinPwmDerB = 9;

const int maxPWM = 100;

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

#define readDerF bitRead(pinEncoderDerF, 4)
#define readDerB bitRead(pinEncoderDerB, 2)
#define readIzqB bitRead(pinEncoderIzqB, 7)
#define readIzqF bitRead(pinEncoderIzqF, 8)

//Counting variables
volatile int encoderCountIzq = 0;
volatile int encoderCountDer = 0;
int previousCountIzq = 0;
int previousCountDer = 0;

//Functions
void countEncoderDerF()
{
  if (readDerF != readDerB)
  {
    encoderCountDer = 1;
  }
  else
  {
    encoderCountDer = -1;
  }
}
void countEncoderDerB()
{
  if (readDerF == readDerB)
  {
    encoderCountDer = 1;
  }
  else
  {
    encoderCountDer = -1;
  }
}
void countEncoderIzqF()
{
  if (readIzqF != readIzqB)
  {
    encoderCountIzq = 1;
  }
  else
  {
    encoderCountIzq = -1;
  }
}
void countEncoderIzqB()
{
  if (readIzqF == readIzqB)
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
    Serial.println(encoderCountDer);
    previousCountDer += encoderCountDer;
    encoderCountDer = 0;
  }
  if (encoderCountIzq != 0)
  {
    Serial.println(encoderCountIzq);
    previousCountIzq += encoderCountIzq;
    encoderCountIzq = 0;
  }
}

/* -------------------------------
  Control
-----------------------------------*/
float kr = 0.3;
float ka = 0.5;
float kb = 0.01;

float vRefDer = 0;
float vRefIzq = 0;

const int errorsLength=10;

float errorIzq = 0;
float errorsIzqArray[errorsLength];
float prevErrIzq = 0;
float errorSignalIzq = 0;

float kpIzq = 0.2;
float kiIzq = 0.000001;
float kdIzq = 0.01;

float errorDer = 0;
float errorsDerArray[errorsLength];
float prevErrDer = 0;
float errorSignalDer = 0;

float kpDer = 0.2;
float kiDer = 0.000001;
float kdDer = 0.01;

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
float b = 0.18;
float l = b / 2;
float carRadius = 0.065;

/* -------------------------------
  Gradient function
-----------------------------------*/
//Function = (x-2)^2+y^2
float gradX()
{
  return 2 * (curX - 2);
}

float gradY()
{
  return 2 * curY;
}

const float finalX = 2;
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
    speedIzq = 884 * PI * carRadius / previousCountIzq;
    previousCountIzq = 0;
  }
  if (previousCountDer > 10 || previousCountDer < -10)
  {
    speedDer = 884 * PI * carRadius / previousCountDer;
    previousCountDer = 0;
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
  alpha = atan2(dY , dX) - curTheta;
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
  errorIzq = vRefIzq - speedIzq;
  errorDer = vRefDer - speedDer;

  float deltaErrTime = micros() - errorTime;
  deltaErrTime = deltaErrTime / 1000000;
  errorTime = micros();

  errorsIzqArray[integralIndex] = errorIzq * deltaErrTime;
  errorsDerArray[integralIndex] = errorDer * deltaErrTime;
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
}

void moveCar()
{
  pinPwmIzqF = 0;

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

  pinMode(pinEncoderDerF, INPUT_PULLUP);
  pinMode(pinEncoderDerB, INPUT_PULLUP);
  pinMode(pinEncoderIzqB, INPUT_PULLUP);
  pinMode(pinEncoderIzqF, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinEncoderDerF), countEncoderDerF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncoderDerB), countEncoderDerB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncoderIzqB), countEncoderIzqB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncoderIzqF), countEncoderIzqF, CHANGE);

  int arr[20];
  memset(arr, 0, sizeof arr);

  while (Serial.available() <= 0) {
    // read the incoming byte:
    int incomingByte = Serial.read();
    if(incomingByte>0) break;
  }

  digitalWrite(pinLedMoving, HIGH);

  timePassedIzq = micros();
  timePassedDer = micros();
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

  control();
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
