#include "PinChangeInterrupt.h"
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include "Simple_MPU6050.h"
// encoder 1
#define encoder0PinA 2   
#define encoder0PinB 3

// encoder 2
#define encoder1PinA 18    
#define encoder1PinB 19

#define MPU6050_DEFAULT_ADDRESS     0x68 // address pin low (GND)

Simple_MPU6050 mpu;

ros::NodeHandle nh;

sensor_msgs::Imu Imu_msg;
ros::Publisher imu_pub("imu/data", &Imu_msg);



unsigned long currentMillis;
unsigned long previousArmMillis;
unsigned long previousMillis;

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

void imu_data(int16_t *gyro, int16_t *accel, int32_t *quat) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  mpu.GetQuaternion(&q, quat);
  mpu.GetGravity(&gravity, &q);
  mpu.GetYawPitchRoll(ypr, &q, &gravity);
  mpu.ConvertToDegrees(ypr, xyz);

  Imu_msg.linear_acceleration.x = accel[0];
  Imu_msg.linear_acceleration.y = accel[1];
  Imu_msg.linear_acceleration.z = accel[2];

  Imu_msg.angular_velocity.x = gyro[0];
  Imu_msg.angular_velocity.y = gyro[1];
  Imu_msg.angular_velocity.z = gyro[2];

  Imu_msg.orientation.w = q.w;
  Imu_msg.orientation.x = q.x;
  Imu_msg.orientation.y = q.y;
  Imu_msg.orientation.z = q.z;

  imu_pub.publish(&Imu_msg);
  nh.spinOnce();
}

void setup() {
  Serial.begin(57600);
  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins
  pinMode(encoder0PinB, INPUT_PULLUP);

  pinMode(encoder1PinA, INPUT_PULLUP); 
  pinMode(encoder1PinB, INPUT_PULLUP);
  
  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins
  pinMode(encoder0PinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoderB, CHANGE); 

  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoderC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoderD, CHANGE); 
  
  
  nh.initNode();
  nh.advertise(imu_pub);
 
   // Setup the MPU and TwoWire aka Wire library all at once
  mpu.begin();
  

  
  mpu.Set_DMP_Output_Rate_Hz(10);          // Set the DMP output rate from 200Hz to 5 Minutes.
  mpu.Set_DMP_Output_Rate_Seconds(10);   // Set the DMP output rate in Seconds
  mpu.Set_DMP_Output_Rate_Minutes(5);    // Set the DMP output rate in Minute
  mpu.CalibrateMPU();                      // Calibrates the MPU.
  mpu.load_DMP_Image();                    // Loads the DMP image into the MPU and finish configuration.
  mpu.on_FIFO(imu_data);               // Set callback function that is triggered when FIFO Data is retrieved
}


void loop() {
  currentMillis = millis(); 
  if (currentMillis - previousMillis >= 10) {  // start timed loop for everything else
         previousMillis = currentMillis;
         mpu.dmp_read_fifo(false);
         Serial.print(encoder0Pos);
         Serial.print(",");
         Serial.println(encoder1Pos);
  }
analogWrite(10,0);
digitalWrite(7,HIGH);
analogWrite(9,0);
digitalWrite(6,LOW);

}

// ************** encoders interrupts **************

// ************** encoder 1 *********************


void doEncoderA(){  

  
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
 
}

void doEncoderB(){  

  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  

}

// ************** encoder 2 *********************

void doEncoderC(){  


  if (digitalRead(encoder1PinA) == HIGH) { 
    
    if (digitalRead(encoder1PinB) == LOW) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  else                                        
  { 
   
    if (digitalRead(encoder1PinB) == HIGH) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
 
}

void doEncoderD(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder1PinB) == HIGH) {   
   
    if (digitalRead(encoder1PinA) == HIGH) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }

  else { 
    
    if (digitalRead(encoder1PinA) == LOW) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
  

}
