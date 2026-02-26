#define LED_BUILTIN 2
// Headers
#include <L298N.h>
#include <QTRSensors.h>

// Motor pins
#define ENA 4
#define AIN1 16
#define AIN2 17

#define ENB 19
#define BIN1 5
#define BIN2 18

// Motors
L298N motor1(ENA, AIN1, AIN2);
L298N motor2(ENB, BIN1, BIN2);

//Light Sensor
#define s0 13
#define s1 23
#define s2 22
#define s3 21
#define sensorOut 34

int redMin = 13;
int redMax = 46;
int greenMin = 18;
int greenMax = 65;
int blueMin = 14;
int blueMax = 51;

int redPW = 0;
int greenPW = 0;
int bluePW = 0;

int red, green, blue;

// IR Sensors
#define IR1 32 //right most
#define IR2 25 //right
#define IR3 26 //middle
#define IR4 27 //left
#define IR5 14 //left most

boolean onoff = true;

boolean colorDetected = false;

QTRSensors qtr;
//min 80 max 2500

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

//PID
double Kp = 0.4;
double Kd = 0.0;
double previousError = 0;
unsigned char MAX_SPEED = 150;

//Nodes
int nodes[4][4] = { {1,-1,-1,-1},
                    {2,4,0,9},
                    {3,5,1,8},
                    {-1,6,2,7}};

int getRedPW() {
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  int pW = pulseIn(sensorOut, LOW);
  return pW;
}

int getGreenPW() {
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  int pW = pulseIn(sensorOut, LOW);
  return pW;
}

int getBluePW() {
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  int pW = pulseIn(sensorOut, LOW);
  return pW;
}

int getRGB(){
  redPW = getRedPW();
  red = map(redPW, redMin, redMax, 255, 0);
  delay(200);

  greenPW = getGreenPW();
  green = map(greenPW, greenMin, greenMax, 255, 0);
  delay(200);

  bluePW = getBluePW();
  blue = map(bluePW, blueMin, blueMax, 255, 0);

  if(red <= 50 && blue <= 50 && green <=50)
    return 4;
  else if(red >=235 && blue>=235 && green >=235)
    return -1;
  else if(red > blue && red > green && red >150 )
    return 1;
  else if(green > blue && green> red && green>150)
    return 2;
  else if(blue> red && blue> green && blue>150)
    return 3;
  else
    return -1;
}

void followLine(int error) {
  double PIDvalue = error*Kp + (error - previousError)*Kd;
  previousError = error;
  motor1.setSpeed(constrain(MAX_SPEED+PIDvalue,0,MAX_SPEED));
  motor2.setSpeed(constrain(MAX_SPEED-PIDvalue,0,MAX_SPEED));
  motor1.forward();
  motor2.forward();
}

void rotateRight(int neighbours[]){
  int last = neighbours[3];
  for(int i=3;i>0;i--)
  {
    neighbours[i]=neighbours[i-1];
  }
  neighbours[0]=last;
}

void rotateLeft(int neighbours[]){
  int first = neighbours[0];
  for(int i=1;i<4;i++)
  {
    neighbours[i-1]=neighbours[i];
  }
  neighbours[3]=first;
}

void rightTurn(int neighbours[]) {
  rotateRight(neighbours);
  Serial.print(" Right ");
  motor1.setSpeed(0);
  motor2.setSpeed(MAX_SPEED/2);
  motor1.forward();
  motor2.forward();
}

void leftTurn(int neighbours[]) {
  rotateLeft(neighbours);
  Serial.print(" Left ");
  motor1.setSpeed(MAX_SPEED/2);
  motor2.setSpeed(0);
  motor1.forward();
  motor2.forward();
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  //Sensor init
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){IR5,IR4,IR3,IR2,IR1}, SensorCount);
  //Sensor calibration
  calibrateSensors();
  //Colour
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  pinMode(sensorOut, INPUT);

  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);
}

void loop() {
  int col = getRGB();
  uint16_t position = qtr.readLineBlack(sensorValues);

  if(col != -1){
    MAX_SPEED = 100;
    colorDetected = true;
  }
  else{
    MAX_SPEED = 150;
  }

  int error = 2000 - position;
  Serial.print("Error = ");
  Serial.print(error);
  Serial.print(" Color = ");
  Serial.println(col);

  if(sensorValues[1]>=980 && sensorValues[2]>=980 && sensorValues[3]>=980 && sensorValues[4] >=980 && sensorValues[5] >=980)
  {
    motor1.stop();
    motor2.stop();
  }
  if(sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 && sensorValues[4] ==0 && sensorValues[5] == 0 && colorDetected)
  {
    motor1.backward();
    motor2.forward();
    delay(200);
    motor1.stop();
    motor2.stop();
    colorDetected = false;
  }
  else
  {
    followLine(error);
  }
}

void calibrateSensors() {
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
    Serial.println(i);
  }

  digitalWrite(LED_BUILTIN, LOW);
}