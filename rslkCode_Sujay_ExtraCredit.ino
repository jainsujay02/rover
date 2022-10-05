// Sujay Jain 
// Anmol Gupta 
// ECE 3 - Final Project - Code

#include <ECE3.h>

uint16_t sensorValues[8]; // store values from all 8 sensors

// store min and max values for all 8 sensors from calibration testing for normalisation

float minSensorVals[8] = {620, 644, 574, 574, 574, 620, 621, 714}; 
float maxSensorVals[8] = {1271.8, 1066, 756, 851, 852, 877, 995, 1355};


float normalizedSensorVals[8];

float errorWeight[8] = { -8, -4, -2, 1 , 1, 2, 4, 8};

float finalSensorErrVal, prevErrVal;  // stores fused sensor values. 

float kp, kd, motor_speed, base_speed, sum_sensorValues; 

int linecount, readcount, turncheck;   

const int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
const int left_dir_pin = 29;
const int left_pwm_pin = 40;


const int right_nslp_pin = 11; // nslp ==> awake & ready for PWM
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

const int LED_RF = 41;

///////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_nslp_pin, HIGH);



  ECE3_Init();

  // set the data rate in bits/second for serial data transmission
  Serial.begin(9600);
  delay(2000); //Wait 2 seconds before starting

  // default values 
  kp = 0.035;
  kd = 0.15;
  base_speed = 55;

  prevErrVal = 0;
  sum_sensorValues = 0.0;
  
  readcount = 0; // used to keep track of number of times sum_sensorValues goes over turncheck 
  turncheck = 14500;  // used to check when the sensor is at turning/stopping point
}

void loop() {
  // put your main code here, to run repeatedly:

  ECE3_read_IR(sensorValues);


  for (int i = 0; i < 8; i++)
  {
    sum_sensorValues += sensorValues[i];

  }


  //If sensor values > some amount, increment
  //if sensor values > some amount, and linecount > 0, and readcount > 2
  if (sum_sensorValues > turncheck && sum_sensorValues != 20000) {
    readcount++;
  }
  else {
    readcount = 0;
  }

  if (sum_sensorValues > turncheck && linecount == 0 && readcount > 2)
  {
    resetEncoderCount_left();

    digitalWrite(right_dir_pin, HIGH);

    analogWrite(left_pwm_pin, 100);
    analogWrite(right_pwm_pin, 100);

    while (getEncoderCount_left() < 360)
    {

    }

    digitalWrite(right_dir_pin, LOW);

    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);

    linecount = 1;
  }
  else if (sum_sensorValues > turncheck && linecount > 0 && readcount > 2 && sum_sensorValues != 20000)
  {

    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);
    delay(200000);
  }

 if (getEncoderCount_right() >0 &&  getEncoderCount_right() <= 500)
  {
    base_speed = 150;
      kp = 0.03;
      kd = 0.3;
  }
   else if ( getEncoderCount_right() > 500 && getEncoderCount_right() <= 1500) {
    base_speed = 60;
    kp = 0.038;
    kd = 0.15;
  }
  
  if (getEncoderCount_right() > 1500 &&  getEncoderCount_right() <= 3500)
  {
    base_speed = 170;
      kp = 0.02;
      kd = 0.6;
  }
  else if ( getEncoderCount_right() > 3500 && getEncoderCount_right() <= 6500) {
    base_speed = 110;
       kp = 0.045;
       kd = 0.3;
  }

  else if ( getEncoderCount_right() > 6500 && getEncoderCount_right() <= 11500) {
    base_speed = 150;
    kp = 0.05;
    kd = 0.6;
  }
  else if ( getEncoderCount_right() > 11500 && getEncoderCount_right() <= 12500) {
    base_speed = 58;
    kp = 0.04;
    kd = 0.15;
  }
  else if (getEncoderCount_right() > 12500)
  {
    base_speed =110;
       kp = 0.03;
      kd = 0.25;
    }


for (int i = 0; i < 8; i++)
{
  normalizedSensorVals[i] = ((sensorValues[i] - minSensorVals[i]) / maxSensorVals[i]) * 1000;

}
for (int j = 0; j < 8; j++)
{
  finalSensorErrVal += normalizedSensorVals[j] * errorWeight[j];
}
finalSensorErrVal = finalSensorErrVal / 4;
motor_speed = (kp * finalSensorErrVal) + (kd * ( finalSensorErrVal - prevErrVal));

analogWrite(left_pwm_pin, base_speed - motor_speed);
analogWrite(right_pwm_pin, base_speed + motor_speed);

prevErrVal = finalSensorErrVal;

sum_sensorValues = 0;
finalSensorErrVal = 0;
}
