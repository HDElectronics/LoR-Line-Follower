/*
 * @Author: KHADRAOUI Ibrahim
 * @Date: 2021-07-09 01:34:33
 * @Last Modified by: KHADRAOUI Ibrahim
 * @Last Modified time: 2021-07-10 02:18:27
 */

/*Libraries*/
#include <Arduino.h>
#include <SoftwareSerial.h>


/*Defines*/
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5
#define ENA 6
#define ENB 8
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
SoftwareSerial SSerial(TX_BT, RX_BT);

/*Variables*/
float Kp = 0, Ki = 0, Kd = 0;//change the value of kp ,ki and kd factors randomly and find a set of these value wich works good for your robot 
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;//defining the intial value 0
float previous_error = 0, previous_I = 0;//defining initially values of previous_error and previous_I 0 
int sensor[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };//defining the sensor arrey of 5 
bool sensor_state[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int initial_motor_speed = 100;//defining the initial value of the motor speed as 100,can be changed
int black_threshold = 0;

/*Functions*/
void read_sensor_values(void);//function that reads sensor values
void calculate_pid(void);//function that caluculates the pid value
void motor_control(void);//function that perform motor control action
void bluetooth_values();//function that get pid constants from HC-05
void test_sensors();//see the values of ir sensors HC-05
int get_black_threshold();

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  SSerial.begin(9600);
  //bluetooth_values();
}

void loop() {
  test_sensors();
  //read_sensor_values();//sensor data is read
  //calculate_pid();// pid is calculated
  //motor_control();//motor speed is controlled
}

void read_sensor_values() {
  sensor[0] = analogRead(SL3);//sensor data read from A0 arduino pin
  sensor[1] = analogRead(SL2);//sensor data read from A1 arduino pin
  sensor[2] = analogRead(SL1);//sensor data read from A2 arduino pin
  sensor[3] = analogRead(SL0);//sensor data read from A3 arduino pin
  sensor[4] = analogRead(SR0);//sensor data read from A4 arduino pin
  sensor[5] = analogRead(SR1);//sensor data read from A5 arduino pin
  sensor[6] = analogRead(SR2);//sensor data read from A6 arduino pin
  sensor[7] = analogRead(SR3);//sensor data read from A7 arduino pin

  for (int i = 0; i <= 7; i++)
  {
    sensor_state[i] = (sensor[i] > black_threshold) ? 1 : 0;//if sensor value exceed the black threshold so it's on black line and it's give true (ternary operator)
  }
  

  if((sensor_state[0] == 0) && (sensor_state[1] == 0) && (sensor_state[2] == 0) && (sensor_state[3] == 0) && (sensor_state[4] == 0) && (sensor_state[5] == 0) && (sensor_state[6] == 0) && (sensor_state[7] == 1))
    error = 4;
  else if(((sensor_state[0] == 0) && (sensor_state[1] == 0) && (sensor_state[2] == 0) && (sensor_state[3] == 0) && (sensor_state[4] == 0) && (sensor_state[5] == 0) && (sensor_state[6] == 1) && (sensor_state[7] == 1)))
    error = 3;
  else if(((sensor_state[0] == 0) && (sensor_state[1] == 0) && (sensor_state[2] == 0) && (sensor_state[3] == 0) && (sensor_state[4] == 0) && (sensor_state[5] == 1) && (sensor_state[6] == 1) && (sensor_state[7] == 0)))
    error = 2;
  else if(((sensor_state[0] == 0) && (sensor_state[1] == 0) && (sensor_state[2] == 0) && (sensor_state[3] == 0) && (sensor_state[4] == 1) && (sensor_state[5] == 1) && (sensor_state[6] == 0) && (sensor_state[7] == 0)))
    error = 1;
  else if(((sensor_state[0] == 0) && (sensor_state[1] == 0) && (sensor_state[2] == 0) && (sensor_state[3] == 1) && (sensor_state[4] == 1) && (sensor_state[5] == 0) && (sensor_state[6] == 0) && (sensor_state[7] == 0)))
    error = 0;
  else if(((sensor_state[0] == 0) && (sensor_state[1] == 0) && (sensor_state[2] == 1) && (sensor_state[3] == 1) && (sensor_state[4] == 0) && (sensor_state[5] == 0) && (sensor_state[6] == 0) && (sensor_state[7] == 0)))
    error = -1;
  else if(((sensor_state[0] == 0) && (sensor_state[1] == 1) && (sensor_state[2] == 1) && (sensor_state[3] == 0) && (sensor_state[4] == 0) && (sensor_state[5] == 0) && (sensor_state[6] == 0) && (sensor_state[7] == 0)))
    error = -2;
  else if(((sensor_state[0] == 1) && (sensor_state[1] == 1) && (sensor_state[2] == 0) && (sensor_state[3] == 0) && (sensor_state[4] == 1) && (sensor_state[5] == 1) && (sensor_state[6] == 1) && (sensor_state[7] == 1)))
    error = -3;
  else if(((sensor_state[0] == 1) && (sensor_state[1] == 0) && (sensor_state[2] == 0) && (sensor_state[3] == 0) && (sensor_state[4] == 0) && (sensor_state[5] == 0) && (sensor_state[6] == 0) && (sensor_state[7] == 0)))
    error = -4;
  else   {
    error = error;
  }

}

void calculate_pid()//calculating pid 
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_I = I;
  previous_error = error;
}

void motor_control()//motor control
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;

  // The motor speed should not exceed the max PWM value
  constrain(left_motor_speed, 0, 255);
  constrain(right_motor_speed, 0, 255);

  analogWrite(ENA, initial_motor_speed - PID_value);   //Left Motor Speed
  analogWrite(ENB, initial_motor_speed + PID_value);  //Right Motor Speed
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void bluetooth_values() {
  while(!SSerial.available()) //waiting for HC-05
  {
    delay(1);
  }
  SSerial.println("Hello Enter following values");
  SSerial.println("Kp: ");
  while(SSerial.available() == 0)   {
    delay(1);
  }
  Kp = SSerial.parseFloat();
  SSerial.println("Ki: ");
  while(SSerial.available() == 0)   {
    delay(1);
  }
  Ki = SSerial.parseFloat();
  SSerial.println("Kd: ");
  while(SSerial.available() == 0)   {
    delay(1);
  }
  Kd = SSerial.parseFloat();
  SSerial.println("Base Speed: ");
  while(SSerial.available() == 0)   {
    delay(1);
  }
  initial_motor_speed = SSerial.parseInt();
}

void test_sensors() {
  read_sensor_values();
  String ir_values = "";
  for (int i = 0; i <= 7; i++)
  {
    ir_values += sensor[i] + " ";
  }
  SSerial.println(ir_values);
  delay(100);
}

int get_black_threshold() {
  static unsigned long currT = millis();
  while (millis() - currT < 5000)
  {
    read_sensor_values();
    delay(10);
  }
  
}