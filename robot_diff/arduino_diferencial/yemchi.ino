#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

//--------------------imu
#include <sensor_msgs/Imu.h>
#include "Simple_MPU6050.h"
#define MPU6050_DEFAULT_ADDRESS     0x68 
Simple_MPU6050 mpu;
//--------------------imu

#define MOTOR_PIN_1 9
#define MOTOR_PIN_2 10
#define DIR_PIN_1 7
#define DIR_PIN_2 6

// Encoder pins
#define LEFT_ENCODER_PIN_A 18
#define LEFT_ENCODER_PIN_B 19
#define RIGHT_ENCODER_PIN_A 2
#define RIGHT_ENCODER_PIN_B 3

volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;

// PID Variables for MOTOR_PIN_1
double Setpoint1, Input1, Output1;
double aggKp1 = 4, aggKi1 = 0.2, aggKd1 = 1;
double consKp1 = 1, consKi1 = 0.05, consKd1 = 0.25;
PID myPID1(&Input1, &Output1, &Setpoint1, consKp1, consKi1, consKd1, DIRECT);

// PID Variables for MOTOR_PIN_2
double Setpoint2, Input2, Output2;
double aggKp2 = 4, aggKi2 = 0.2, aggKd2 = 1;
double consKp2 = 1, consKi2 = 0.05, consKd2 = 0.25;
PID myPID2(&Input2, &Output2, &Setpoint2, consKp2, consKi2, consKd2, DIRECT);

// ROS NodeHandle
ros::NodeHandle nh;

std_msgs::Int16 left_wheel_msg;
std_msgs::Int16 right_wheel_msg;
ros::Publisher left_wheel_pub("lwheel", &left_wheel_msg);
ros::Publisher right_wheel_pub("rwheel", &right_wheel_msg);
//-------------------imu
sensor_msgs::Imu Imu_msg;
ros::Publisher imu_pub("imu/data", &Imu_msg);
//-------------------imu      
void velCallback(const geometry_msgs::Twist& msg);
ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", velCallback);

void setup()
{
  // Initialize motor control pins
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);

  // Initialize encoders
  pinMode(LEFT_ENCODER_PIN_A, INPUT);
  pinMode(LEFT_ENCODER_PIN_B, INPUT);
  pinMode(RIGHT_ENCODER_PIN_A, INPUT);
  pinMode(RIGHT_ENCODER_PIN_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_B), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN_A), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN_B), rightEncoderISR, CHANGE);

  // Initialize PID
  Input1 = 0;  // Initial encoder input value for MOTOR_PIN_1
  Setpoint1 = 0;  // Initial setpoint for MOTOR_PIN_1
  myPID1.SetMode(AUTOMATIC);

  Input2 = 0;  // Initial encoder input value for MOTOR_PIN_2
  Setpoint2 = 0;  // Initial setpoint for MOTOR_PIN_2
  myPID2.SetMode(AUTOMATIC);

  // Pre-tune the PID
  for (int i = 0; i < 100; i++) {
    myPID1.Compute();
    myPID2.Compute();
  }

  // Initialize ROS node and subscriber
  nh.initNode();
  nh.subscribe(vel_sub);
  nh.advertise(left_wheel_pub);
  nh.advertise(right_wheel_pub);
    //---------------imu
  nh.advertise(imu_pub);
  mpu.begin();
  mpu.Set_DMP_Output_Rate_Hz(100);         
  mpu.Set_DMP_Output_Rate_Seconds(10);   
  mpu.Set_DMP_Output_Rate_Minutes(5);    
  mpu.CalibrateMPU();                     
  mpu.load_DMP_Image();                   
  mpu.on_FIFO(publicar_imu); 
  //---------------imu

}

void loop()
{
  // Update Input with the encoder value for PID control
  Input1 = left_encoder_ticks;
  Input2 = right_encoder_ticks;

  // Process ROS callbacks
  nh.spinOnce();
  delay(10);// Small delay to prevent overwhelming the loop
       //-------------------------------imu
  mpu.dmp_read_fifo(false);
    //-------------------------------imu
  // Publish encoder values
  publishEncoders();
}
void publicar_imu(int16_t *gyro, int16_t *accel, int32_t *quat) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  mpu.GetQuaternion(&q, quat);
  mpu.GetGravity(&gravity, &q);
  mpu.GetYawPitchRoll(ypr, &q, &gravity);
  mpu.ConvertToDegrees(ypr, xyz);


  Imu_msg.header.stamp = nh.now();
  Imu_msg.header.frame_id = "imu_link";
  Imu_msg.linear_acceleration.x = accel[0]* (4.0 / 65536.0) * 9.81;
  Imu_msg.linear_acceleration.y = accel[1]* (4.0 / 65536.0) * 9.81;
  Imu_msg.linear_acceleration.z = accel[2]* (4.0 / 65536.0) * 9.81;

  Imu_msg.angular_velocity.x = gyro[0] * (4000.0/65536.0) * (3.14/180.0) * 25.0;
  Imu_msg.angular_velocity.y = gyro[1] * (4000.0/65536.0) * (3.14/180.0) * 25.0;
  Imu_msg.angular_velocity.z = gyro[2] * (4000.0/65536.0) * (3.14/180.0) * 25.0;

  Imu_msg.orientation.w = q.w;
  Imu_msg.orientation.x = q.x;
  Imu_msg.orientation.y = q.y;
  Imu_msg.orientation.z = q.z;

  

  imu_pub.publish(&Imu_msg);
  nh.spinOnce();
}

void velCallback(const geometry_msgs::Twist& msg)
{
  Serial.println("Received a message!");

  Setpoint1 = msg.linear.x;
  Setpoint2 = msg.linear.x;

  double gap1 = abs(Setpoint1 - Input1);
  double gap2 = abs(Setpoint2 - Input2);

  if (gap1 < 10) {
    myPID1.SetTunings(consKp1, consKi1, consKd1);
  } else {
    myPID1.SetTunings(aggKp1, aggKi1, aggKd1);
  }

  if (gap2 < 10) {
    myPID2.SetTunings(consKp2, consKi2, consKd2);
  } else {
    myPID2.SetTunings(aggKp2, aggKi2, aggKd2);
  }

  myPID1.Compute();
  myPID2.Compute();

  // Constrain outputs to be between 80 and 255
  Output1 = constrain(Output1, 120, 255);
  Output2 = constrain(Output2, 120, 255);

  if (msg.linear.x > 0 && msg.angular.z == 0) {
    analogWrite(MOTOR_PIN_1, Output1);
    analogWrite(MOTOR_PIN_2, Output2);
    digitalWrite(DIR_PIN_1, HIGH);
    digitalWrite(DIR_PIN_2, LOW);
  }
  else if (msg.linear.x < 0 && msg.angular.z == 0) {
    analogWrite(MOTOR_PIN_1, Output1);
    analogWrite(MOTOR_PIN_2, Output2);
    digitalWrite(DIR_PIN_1, LOW);
    digitalWrite(DIR_PIN_2, HIGH);
  }
  else if (msg.angular.z > 0 && msg.linear.x == 0) {
    analogWrite(MOTOR_PIN_1, Output1);
    analogWrite(MOTOR_PIN_2, Output2);
    digitalWrite(DIR_PIN_1, HIGH);
    digitalWrite(DIR_PIN_2, HIGH);
  }
  else if (msg.angular.z < 0 && msg.linear.x == 0) {
    analogWrite(MOTOR_PIN_1, Output1);
    analogWrite(MOTOR_PIN_2, Output2);
    digitalWrite(DIR_PIN_1, LOW);
    digitalWrite(DIR_PIN_2, LOW);
  }
  else if (msg.linear.x > 0 && msg.angular.z > 0) {
    analogWrite(MOTOR_PIN_1, Output1);
    analogWrite(MOTOR_PIN_2, Output2 / 2);
    digitalWrite(DIR_PIN_1, HIGH);
    digitalWrite(DIR_PIN_2, LOW);
  }
  else if (msg.linear.x > 0 && msg.angular.z < 0) {
    analogWrite(MOTOR_PIN_1, Output1 / 2);
    analogWrite(MOTOR_PIN_2, Output2);
    digitalWrite(DIR_PIN_1, HIGH);
    digitalWrite(DIR_PIN_2, LOW);
  }
  else if (msg.linear.x < 0 && msg.angular.z > 0) {
    analogWrite(MOTOR_PIN_1, Output1);
    analogWrite(MOTOR_PIN_2, Output2 / 2);
    digitalWrite(DIR_PIN_1, LOW);
    digitalWrite(DIR_PIN_2, HIGH);
  }
  else if (msg.linear.x < 0 && msg.angular.z < 0) {
    analogWrite(MOTOR_PIN_1, Output1 / 2);
    analogWrite(MOTOR_PIN_2, Output2);
    digitalWrite(DIR_PIN_1, LOW);
    digitalWrite(DIR_PIN_2, HIGH);
  }
  else if (msg.linear.x == 0 && msg.angular.z == 0) {
    analogWrite(MOTOR_PIN_1, 0);
    analogWrite(MOTOR_PIN_2, 0);
    digitalWrite(DIR_PIN_1, HIGH);
    digitalWrite(DIR_PIN_2, LOW);
  }
}


void publishEncoders()
{
left_wheel_msg.data = left_encoder_ticks;
right_wheel_msg.data = right_encoder_ticks;
left_wheel_pub.publish(&left_wheel_msg);
right_wheel_pub.publish(&right_wheel_msg);
}
void leftEncoderISR()
{
static uint8_t state = 0;
state = (state << 2) | (digitalRead(LEFT_ENCODER_PIN_A) << 1) | digitalRead(LEFT_ENCODER_PIN_B);
switch (state & 0b1111) {
case 0b0001:
case 0b0111:
case 0b1110:
case 0b1000:
left_encoder_ticks++;
break;
case 0b0010:
case 0b1011:
case 0b1101:
case 0b0100:
left_encoder_ticks--;
break;
}
}

void rightEncoderISR()
{
static uint8_t state = 0;
state = (state << 2) | (digitalRead(RIGHT_ENCODER_PIN_A) << 1) | digitalRead(RIGHT_ENCODER_PIN_B);
switch (state & 0b1111) {
case 0b0001:
case 0b0111:
case 0b1110:
case 0b1000:
right_encoder_ticks++;
break;
case 0b0010:
case 0b1011:
case 0b1101:
case 0b0100:
right_encoder_ticks--;
break;
}
}