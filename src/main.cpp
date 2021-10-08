/*
 * @Author: KHADRAOUI Ibrahim
 * @Date: 2021-07-09 01:34:33
 * @Last Modified by: KHADRAOUI Ibrahim
 * @Last Modified time: 2021-10-08 11:59:11
 */

/*Libraries*/
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <QTRSensors.h>

/*Defines*/
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5
#define ENA 6
#define ENB 9
#define RX_BT 10
#define TX_BT 11
#define SL3 A5
#define SL2 A4
#define SL1 A3
#define SL0 A2
#define SR0 A1
#define SR1 A0
#define SR2 A7
#define SR3 A6
/* SL3 SL2 SL1 SL0 SR0 SR1 SR2 SR3 */

/*Instances*/
SoftwareSerial SSerial(RX_BT, TX_BT);
QTRSensors qtr;

/*Variables*/
float Kp = 10, Ki = 0.1, Kd = 0.0;//change the value of kp ,ki and kd factors randomly and find a set of these value witch works good for your robot 
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;//defining the initial value 0
float previous_error = 0, previous_I = 0;//defining initially values of previous_error and previous_I 0 
int initial_motor_speed = 170;//defining the initial value of the motor speed as 100,can be changed
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
bool Black_Line[SensorCount];
int left_motor_speed, right_motor_speed;

/*Functions*/
void calculate_pid(void);//function that calculates the pid value
void motor_control(void);//function that perform motor control action
void bluetooth_values();//function that get pid constants from HC-05
void track_black_line();
void emptyBuff();
void motor(int vR, int vL);

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  SSerial.begin(9600);
  Serial.begin(9600);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) { SL3, SL2, SL1, SL0, SR0, SR1, SR2, SR3 }, 8);
  qtr.setSamplesPerSensor(8);

  //pid values from phone
  //bluetooth_values();

  /*SSerial.println("Calibration started for 10 secondes");
  Serial.println("Calibration started for 10 secondes");
  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 8 samples per sensor read * 8 sensors
  // * 10 reads per calibrate() call = 64 ms per calibrate() call.
  // Call calibrate() 156 times to make calibration take about 10 seconds.
  for(uint16_t i = 0; i < 156; i++) {
    qtr.calibrate();
  }
  SSerial.println("Calibration terminated");
  Serial.println("Calibration terminated");

  SSerial.println("Sensor calibration minimum:");
  Serial.println("Sensor calibration minimum:");
  for(uint8_t i = 0; i < SensorCount; i++) {
    SSerial.print(qtr.calibrationOn.minimum[i]);
    SSerial.print(" ");
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(" ");
  }
  SSerial.println();
  SSerial.println("Sensor calibration maximum:");
  Serial.println();
  Serial.println("Sensor calibration maximum:");
  for(uint8_t i = 0; i < SensorCount; i++) {
    SSerial.print(qtr.calibrationOn.maximum[i]);
    SSerial.print(" ");
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(" ");
  }
  SSerial.println();
  Serial.println();*/
  
  //value after calibration
  qtr.calibrate();
  for(uint8_t i = 0; i < SensorCount; i++) {
    qtr.calibrationOn.minimum[i] = 150;
  }
  for(uint8_t i = 0; i < SensorCount; i++) {
    qtr.calibrationOn.maximum[i] = 350;
  }

}

void loop() {

  //Print raw Sensor data
  /*qtr.read(sensorValues, QTRReadMode::On);
  SSerial.println("\n\rsensorValues:");
  Serial.println("\n\rsensorValues:");
  for(uint8_t i = 0; i < SensorCount; i++) {
    SSerial.print(sensorValues[i]);
    SSerial.print(',');
    Serial.print(sensorValues[i]);
    Serial.print(',');
  }*/

  //Print black line position
  /*SSerial.println("\n\rBlack_Line:");
  Serial.println("\n\rBlack_Line:");
  for(uint8_t i = 0; i < SensorCount; i++) {
    SSerial.print(Black_Line[i]);
    SSerial.print(',');
    Serial.print(Black_Line[i]);
    Serial.print(',');
  }
  track_black_line();*/

  //Print the error
  /*SSerial.println("\n\rerror:");
  Serial.println("\n\rerror:");
  SSerial.print(error);
  Serial.print(error);
  calculate_pid();

  SSerial.print(";\n\r******************");
  Serial.print(";\n\r******************");*/

  track_black_line();
  calculate_pid();
  motor_control();
  delay(1);
  
  //print the command sent to the motors
  SSerial.println("\n\rLeft Right Speed:");
  SSerial.print(left_motor_speed);SSerial.print("***");SSerial.print(right_motor_speed);
}

void calculate_pid() { //calculating pid
  if(Black_Line[0] == 0 && Black_Line[1] == 0 && Black_Line[2] == 0 && Black_Line[3] == 0 && Black_Line[4] == 0 && Black_Line[5] == 0 && Black_Line[6] == 0 && Black_Line[7] == 0) {
    error = 0;
  }
  else if(Black_Line[0] == 0 && Black_Line[1] == 0 && Black_Line[2] == 0 && Black_Line[3] == 1 && Black_Line[4] == 0 && Black_Line[5] == 0 && Black_Line[6] == 0 && Black_Line[7] == 0) {
    error = -1;
  }
  else if(Black_Line[0] == 0 && Black_Line[1] == 0 && Black_Line[2] == 1 && Black_Line[3] == 1 && Black_Line[4] == 0 && Black_Line[5] == 0 && Black_Line[6] == 0 && Black_Line[7] == 0) {
    error = -2;
  }
  else if(Black_Line[0] == 0 && Black_Line[1] == 0 && Black_Line[2] == 1 && Black_Line[3] == 0 && Black_Line[4] == 0 && Black_Line[5] == 0 && Black_Line[6] == 0 && Black_Line[7] == 0) {
    error = -3;
  }
  else if(Black_Line[0] == 0 && Black_Line[1] == 1 && Black_Line[2] == 1 && Black_Line[3] == 0 && Black_Line[4] == 0 && Black_Line[5] == 0 && Black_Line[6] == 0 && Black_Line[7] == 0) {
    error = -4;
  }
  else if(Black_Line[0] == 0 && Black_Line[1] == 1 && Black_Line[2] == 0 && Black_Line[3] == 0 && Black_Line[4] == 0 && Black_Line[5] == 0 && Black_Line[6] == 0 && Black_Line[7] == 0) {
    error = -5;
  }
  else if(Black_Line[0] == 1 && Black_Line[1] == 1 && Black_Line[2] == 0 && Black_Line[3] == 0 && Black_Line[4] == 0 && Black_Line[5] == 0 && Black_Line[6] == 1 && Black_Line[7] == 0) {
    error = -6;
  }
  else if(Black_Line[0] == 1 && Black_Line[1] == 1 && Black_Line[2] == 1 && Black_Line[3] == 0 && Black_Line[4] == 0 && Black_Line[5] == 0 && Black_Line[6] == 0 && Black_Line[7] == 0) {
    error = -7;
  }
  else if(Black_Line[0] == 0 && Black_Line[1] == 0 && Black_Line[2] == 0 && Black_Line[3] == 0 && Black_Line[4] == 1 && Black_Line[5] == 0 && Black_Line[6] == 0 && Black_Line[7] == 0) {
    error = 1;
  }
  else if(Black_Line[0] == 0 && Black_Line[1] == 0 && Black_Line[2] == 0 && Black_Line[3] == 0 && Black_Line[4] == 1 && Black_Line[5] == 1 && Black_Line[6] == 0 && Black_Line[7] == 0) {
    error = 2;
  }
  else if(Black_Line[0] == 0 && Black_Line[1] == 0 && Black_Line[2] == 0 && Black_Line[3] == 0 && Black_Line[4] == 0 && Black_Line[5] == 1 && Black_Line[6] == 0 && Black_Line[7] == 0) {
    error = 3;
  }
  else if(Black_Line[0] == 1 && Black_Line[1] == 1 && Black_Line[2] == 1 && Black_Line[3] == 1 && Black_Line[4] == 0 && Black_Line[5] == 1 && Black_Line[6] == 1 && Black_Line[7] == 0) {
    error = 4;
  }
  else if(Black_Line[0] == 0 && Black_Line[1] == 0 && Black_Line[2] == 0 && Black_Line[3] == 0 && Black_Line[4] == 0 && Black_Line[5] == 0 && Black_Line[6] == 1 && Black_Line[7] == 0) {
    error = 5;
  }
  else if(Black_Line[0] == 0 && Black_Line[1] == 0 && Black_Line[2] == 0 && Black_Line[3] == 0 && Black_Line[4] == 0 && Black_Line[5] == 0 && Black_Line[6] == 1 && Black_Line[7] == 1) {
    error = 6;
  }
  else if(Black_Line[0] == 0 && Black_Line[1] == 0 && Black_Line[2] == 0 && Black_Line[3] == 0 && Black_Line[4] == 0 && Black_Line[5] == 0 && Black_Line[6] == 0 && Black_Line[7] == 1) {
    error = 7;
  }
  else if(Black_Line[0] == 1 && Black_Line[1] == 1 && Black_Line[2] == 1 && Black_Line[3] == 1 && Black_Line[4] == 0 && Black_Line[5] == 0 && Black_Line[6] == 0 && Black_Line[7] == 0) {
    error = -10;
  }
  else if(Black_Line[0] == 0 && Black_Line[1] == 0 && Black_Line[2] == 0 && Black_Line[3] == 0 && Black_Line[4] == 1 && Black_Line[5] == 1 && Black_Line[6] == 1 && Black_Line[7] == 1) {
    error = 10;
  }

  P = error;
  I = I + previous_I;
  D = error - previous_error;
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_I = I;
  previous_error = error;
}

void motor_control() { //motor control
  // Calculating the effective motor speed:
  left_motor_speed = initial_motor_speed + PID_value;
  right_motor_speed = initial_motor_speed - PID_value;

  // The motor speed should not exceed the max PWM value
  if (left_motor_speed > 255) left_motor_speed = 255;
  if (right_motor_speed > 255) right_motor_speed = 255;
  if (left_motor_speed < -255) left_motor_speed = -255;
  if (right_motor_speed < -255) right_motor_speed = -255;

  motor(right_motor_speed, left_motor_speed);
}

void bluetooth_values() {
  while(!SSerial.available()) {//waiting for HC-05
    delay(1);
  }
  emptyBuff();
  SSerial.println("Hello Enter following values");
  SSerial.println("Kp: ");
  while(SSerial.available() == 0) {
    delay(1);
  }
  Kp = SSerial.parseFloat();
  Serial.print("Kp = ");Serial.println(Kp);
  emptyBuff();
  SSerial.println("Ki: ");
  while(SSerial.available() == 0) {
    delay(1);
  }
  Ki = SSerial.parseFloat();
  Serial.print("Ki = ");Serial.println(Ki);
  emptyBuff();
  SSerial.println("Kd: ");
  while(SSerial.available() == 0) {
    delay(1);
  }
  Kd = SSerial.parseFloat();
  Serial.print("Kd = ");Serial.println(Kd);
  emptyBuff();
  SSerial.println("Base Speed: ");
  while(SSerial.available() == 0) {
    delay(1);
  }
  initial_motor_speed = SSerial.parseInt();
  Serial.print("base speed = ");Serial.println(initial_motor_speed);
  emptyBuff();
}

void track_black_line() {

  for(uint8_t i = 0; i < SensorCount; i++) {
    uint16_t value = sensorValues[i];
    qtr.readCalibrated(sensorValues, QTRReadMode::On);

    // only average in values that are above a noise threshold
    if(value > 50) {
      Black_Line[i] = 1;
    }
    else {
      Black_Line[i] = 0;
    }
  }
}

void emptyBuff(){
  char x;
  while (SSerial.available() != 0){
    x = SSerial.read();
  }
}

void motor(int vR, int vL) {

  if (vR > 0 && vL > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, vL);
    analogWrite(ENB, vR);
    Serial.println("straight");
  }
  else if (vR < 0 && vL > 0) {
    vR *= -1;
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, vL);
    analogWrite(ENB, vR);
    Serial.println("turn left");
  }
  else if (vR > 0 && vL < 0) {
    vL *= -1;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, vL);
    analogWrite(ENB, vR);
    Serial.println("turn right");
  }
  else if (vR < 0 && vL < 0) {
    vL *= -1;
    vR *= -1;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, vL);
    analogWrite(ENB, vR);
    Serial.println("backward");
  }

}
