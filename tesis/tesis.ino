/* -------------------------------
  Other parameters
-----------------------------------*/
const float pointReachedThreshold = 0.1;

/* -------------------------------
  PWM
-----------------------------------*/
int pinPwmIzqF = 3;
int pinPwmIzqB = 5;
int pinPwmDerF = 6;
int pinPwmDerB = 9;

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

//control
float kr=0.13;
float ka=0.12;
float kb=0.01;

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

void calcNewPoint(grad)
{
  newX = grad[0];
  newY = grad[1];
  float norm = sqrt(grad[0] * grad[0] + grad[1] * grad[1]);
  if (norm > 0.0001)
  {
    newX = newX / norm;
    newY = newY / norm;
  }
}

void calcControlVariables()
{
  float dX = newX - curX;
  float dY = newY - curY;

  rho = sqrt(dX * dX + dY * dY);

  if (rho < pointReachedThreshold)
  {
    reachedNewPoint = true;
    return;
  }
  alpha = atan2(dY / dX) - curTheta;
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

void calcRefVelocities(){
  float v = kr*rho;
  float w = ka*alpha+kb*beta;

  float vRefDer=v-l*w;
  float vRefIzq=v-l*w;
}

void controlOdometry()
{
  calcControlVariables();
  calcRefVelocities();
}

void control()
{
}

void setup()
{
  Serial.begin(115200);

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

  timePassedIzq = micros();
  timePassedDer = micros();
}

void loop()
{

  readEncoders();

  odometry();

  float trueGradX = -gradX();
  float trueGradY = -gradY();

  float grad[2] = {trueGradX, trueGradY};

  if (reachedNewPoint)
  {
    calcNewPoint(grad);
    reachedNewPoint = false;
  }

  controlOdometry();

  control();
}
