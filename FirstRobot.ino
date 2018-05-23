#include <Ultrasonic.h>
#include <AFMotor.h>
#include <Servo.h>
#include <Wire.h>

//Component pins definition
#define RearLeftMotorPin 2
#define BumperLeftMotorPin 3

#define RearRightMotorPin 1
#define BumperRightMotorPin 4

#define ServoMotorPin 10

#define UltrasonicSensorTriggerPin A0
#define UltrasonicSensorEchoPin A1

#define SerialDataSDA A4
#define SerialClockSCL A5

#define MapperAmbientAddress 0x08

//Components Init
AF_DCMotor bumperLeftMotor(BumperLeftMotorPin);
AF_DCMotor bumperRightMotor(BumperRightMotorPin);

AF_DCMotor rearLeftMotor(RearLeftMotorPin);
AF_DCMotor rearRightMotor(RearRightMotorPin);

Servo servoMotor;

Ultrasonic eyesUltrasonicSensor(UltrasonicSensorTriggerPin, UltrasonicSensorEchoPin);

//Globals
int const Forward = 1;
int const Backward = 2;
int const Left = 3;
int const Right = 4;

int velocity = 255; //0 to 255

int steeringSensibility = 250; //TODO: se sobrar portas calibrar para posição central do potenciometro

int lastMovement = 0;

int currentX = 0;
int currentY = 0;
int currentState = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin(); //inicia comunicação I2C

  servoMotor.attach(ServoMotorPin);
  servoMotor.write(90);

  pinMode(BumperLeftMotorPin, OUTPUT);
  pinMode(BumperRightMotorPin, OUTPUT);
  pinMode(RearLeftMotorPin, OUTPUT);
  pinMode(RearRightMotorPin, OUTPUT);

  bumperLeftMotor.run(RELEASE);
  bumperRightMotor.run(RELEASE);
  rearLeftMotor.run(RELEASE);
  rearRightMotor.run(RELEASE);
}

void loop() {
  Serial.println(getNextObstacleDistance());
  if (lastMovement != Forward)
    runForward();
  if (getNextObstacleDistance() <= 50)
    chooseBestWay();
}

void runForward() {
  if (velocity > 0 && lastMovement != Forward)
  {
    bumperLeftMotor.run(RELEASE);
    bumperRightMotor.run(RELEASE);
    rearLeftMotor.run(RELEASE);
    rearRightMotor.run(RELEASE);
    delay(300);
  }

  lastMovement = Forward;

  bumperLeftMotor.setSpeed(velocity);
  bumperRightMotor.setSpeed(velocity);
  rearLeftMotor.setSpeed(velocity);
  rearRightMotor.setSpeed(velocity);

  bumperLeftMotor.run(FORWARD);
  bumperRightMotor.run(FORWARD);
  rearLeftMotor.run(FORWARD);
  rearRightMotor.run(FORWARD);
}

void runBackward() {
  if (velocity > 0 && lastMovement != Backward)
  {
    bumperLeftMotor.run(RELEASE);
    bumperRightMotor.run(RELEASE);
    rearLeftMotor.run(RELEASE);
    rearRightMotor.run(RELEASE);
    delay(300);
  }

  lastMovement = Backward;

  bumperLeftMotor.setSpeed(velocity);
  bumperRightMotor.setSpeed(velocity);
  rearLeftMotor.setSpeed(velocity);
  rearRightMotor.setSpeed(velocity);

  bumperLeftMotor.run(BACKWARD);
  bumperRightMotor.run(BACKWARD);
  rearLeftMotor.run(BACKWARD);
  rearRightMotor.run(BACKWARD);
}

void turnLeft() {
  bumperLeftMotor.run(FORWARD);
  rearLeftMotor.run(FORWARD);
  bumperRightMotor.run(BACKWARD);
  rearRightMotor.run(BACKWARD);

  lastMovement = Left;
  
  for (int i = 0; i < 255; i++)
  {
    delayMicroseconds(5000);
    bumperLeftMotor.setSpeed(i);
    bumperRightMotor.setSpeed(i);
    rearLeftMotor.setSpeed(i);
    rearRightMotor.setSpeed(i);
  }

  delay(steeringSensibility);
}

void turnRight() {
  bumperRightMotor.run(FORWARD);
  rearRightMotor.run(FORWARD);
  bumperLeftMotor.run(BACKWARD);
  rearLeftMotor.run(BACKWARD);

  lastMovement = Right;
  
  for (int i = 0; i < 255; i++)
  {
    delayMicroseconds(5000);
    bumperLeftMotor.setSpeed(i);
    bumperRightMotor.setSpeed(i);
    rearLeftMotor.setSpeed(i);
    rearRightMotor.setSpeed(i);
  }

  delay(steeringSensibility);
}

void chooseBestWay() {
  bumperLeftMotor.run(RELEASE);
  bumperRightMotor.run(RELEASE);
  rearLeftMotor.run(RELEASE);
  rearRightMotor.run(RELEASE);

  servoMotor.write(0);

  float leftObstacleDistance = getNextObstacleDistance();
  Serial.print("Esquerda: ");
  Serial.print(leftObstacleDistance);
  Serial.println("Cm");
  delay(500);

  servoMotor.write(180);
  float rightObstacleDistance = getNextObstacleDistance();
  Serial.print("Direita: ");
  Serial.print(rightObstacleDistance);
  Serial.println("Cm");
  delay(500);

  servoMotor.write(90);

  if (leftObstacleDistance < 50 && rightObstacleDistance < 50)
  {
    Serial.println("Dando ré");
    runBackward();
    delay(1500);
    chooseBestWay();
  }
  else if (leftObstacleDistance > rightObstacleDistance)
  {
    Serial.println("Virando a esquerda");
    turnLeft();
  }
  else if (leftObstacleDistance < rightObstacleDistance)
  {
    Serial.println("Virando a direita");
    turnRight();
  }
}

float getNextObstacleDistance() {
  Serial.println(eyesUltrasonicSensor.convert(eyesUltrasonicSensor.timing(), Ultrasonic::CM));
  return eyesUltrasonicSensor.convert(eyesUltrasonicSensor.timing(), Ultrasonic::CM);
}

//void sendCurrentCoordinates(int x, int y, boolean state)
//{
//  Wire.beginTransmission(MapperAmbientAddress);
//  Wire.write(x);
//  Wire.write(y);
//  Wire.write(state);
//  Wire.endTransmission();
//}
