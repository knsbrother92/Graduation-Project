#include <SoftwareSerial.h>
#include <Wire.h>
#include "Kalman.h"

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; // Angle calculate using the accelerometer
double temp; // Temperature
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
////////////////////////////

/* 블루투스 모듈 */
int blueTx=5;   //Tx (보내는핀 설정)at
int blueRx=4;   //Rx (받는핀 설정)
SoftwareSerial BTSerial(blueTx, blueRx);  //시리얼 통신을 위한 객체선언

/* 충격 센서 */
int EP = 3;

/* 함수 정의 */
float ultrasonic_setup(void);

//초음파 센서의 핀번호를 설정한다.
int echoPin = 12;
int trigPin = 13;

void setup() {
  /* 충격 센서 setup */
  pinMode(EP, INPUT); //set EP input for measurment

  /* 통신 setup */
  Serial.begin(9600);
  BTSerial.begin(9600); 

  /* 초음파 핀 설정 */
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  Wire.begin();
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x03; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;

  kalmanX.setAngle(accXangle); // Set starting angle
  kalmanY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;
  compAngleX = accXangle;
  compAngleY = accYangle;

  timer = micros();
}

void loop() {
  float distance = 0; // 초음파 센서
  int sensorval = analogRead(A0); // 압력 센서
  long measurement =TP_init();
  
  distance = ultrasonic_setup();
  if((distance > 1.0 && distance < 10.0) && measurement != 0){ //초음파 거리 이내로 충돌하면
    if((sensorval == 0 || sensorval != 0)){ // 에어백 작동여부와 상관 없이 신고
      Serial.println('Y');
      BTSerial.println('Y');
    }
  }
  /* 칼만 필터를 적용한 균형 측정 */
  angleValue();
  
  //Serial.print(kalAngleX);
  //Serial.print('\t');
  //Serial.print(kalAngleY);
  //Serial.println('\t');
  
  /* 전방 후방 */
  if((kalAngleY < 120.00) || (kalAngleY > 250.00)){
    Serial.println('Y');
    BTSerial.println('Y');
  }
  /* 좌측면 우측면 */
  if((kalAngleX > 250.00) || (kalAngleX < 120.00)){
    Serial.println('Y');
    BTSerial.println('Y');
  }
  /* 전방 좌측면 */
  if((kalAngleX > 220.00) && (kalAngleY < 120.00)){
    Serial.println('Y');
    BTSerial.println('Y');
  }
  /* 전방 우측면 */
  if((kalAngleX < 140.00) && (kalAngleY < 140.00)){
    Serial.println('Y');
    BTSerial.println('Y');
  }
  /* 후방 좌측면 */
  if((kalAngleX > 220.00) && (kalAngleY > 220.00)){
    Serial.println('Y');
    BTSerial.println('Y');
  }
  /* 후방 우측면 */
  if((kalAngleX < 120.00) && (kalAngleY > 220.00)){
    Serial.println('Y');
    BTSerial.println('Y');
  }
}

void angleValue() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = ((i2cData[6] << 8) | i2cData[7]);
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);

  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;

  double gyroXrate = (double)gyroX / 131.0;
  double gyroYrate = -((double)gyroY / 131.0);
  gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000);


  compAngleX = (0.93 * (compAngleX + (gyroXrate * (double)(micros() - timer) / 1000000))) + (0.07 * accXangle); // Calculate the angle using a Complimentary filter
  compAngleY = (0.93 * (compAngleY + (gyroYrate * (double)(micros() - timer) / 1000000))) + (0.07 * accYangle);

  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer) / 1000000);
  timer = micros();

  temp = ((double)tempRaw + 12412.0) / 340.0;


}
/* 초음파 센서 setup function */
float ultrasonic_setup(void){ 
  float duration, distance;
  digitalWrite(trigPin, HIGH);
  delay(10);
  digitalWrite(trigPin, LOW);
  // echoPin 이 HIGH를 유지한 시간을 저장 한다.
  duration = pulseIn(echoPin, HIGH); 
  // HIGH 였을 때 시간(초음파가 보냈다가 다시 들어온 시간)을 가지고 거리를 계산 한다.
  distance = ((float)(340 * duration) / 10000) / 2;  
  return distance;
}

/* 충격 센서 Init function */
long TP_init(){
  long measurement=pulseIn (EP, HIGH);  //wait for the pin to get HIGH and returns measurement
  return measurement;
}





