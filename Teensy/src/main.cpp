// /dev/ttyACM0

#include <Arduino.h>
#include "AutoPID.h"
#include <Encoder.h>
#include "ros.h"
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <string>

// #define DEBUG
#define TLED 23

#define MOTOR_R_EN 31
#define MOTOR_R_FR 30
#define MOTOR_R_ENC_A 24
#define MOTOR_R_ENC_B 12

#define MOTOR_L_EN 29
#define MOTOR_L_FR 28
#define MOTOR_L_ENC_A 10
#define MOTOR_L_ENC_B 11

double MOTOR_R_BIAS = 175;
double MOTOR_L_BIAS = 175;

double speed_act_L = 0;
double speed_act_R = 0;

long encoderLeftPos = 0.0;
long encoderRightPos = 0.0;
long encoderLeftPrev = 0.0;
long encoderRightPrev = 0.0;

double encoderLeftDiff = 0.0;
double encoderRightDiff = 0.0;
double encoderLeftError = 0.0;
double encoderRightError = 0.0;

Encoder encoderRight(MOTOR_R_ENC_A, MOTOR_R_ENC_B);
Encoder encoderLeft(MOTOR_L_ENC_A, MOTOR_L_ENC_B);

double demandL = 0.0; 
double demandR = 0.0; 

unsigned long currentMillis;
unsigned long previousMillis;
unsigned long previousMillis_Serial;

unsigned int loopTimeMs = 10;
float PPT = 87;
float LBASE = 0.368; // in meters

double kP = 15;
double kI = 23;
double kD = 20;

double PkL = kP;
double IkL = kI;
double DkL = kD;
double SetpointL, InputL, OutputL;
AutoPID PIDL(&InputL, &SetpointL, &OutputL, -1000.0, 1000.0, PkL, IkL, DkL);

double PkR = kP;
double IkR = kI;
double DkR = kD;
double SetpointR, InputR, OutputR;
AutoPID PIDR(&InputR, &SetpointR, &OutputR, -1000.0, 1000.0, PkR, IkR, DkR);

double prev_OutputL = 0;
double prev_OutputR = 0;

//ROS
double demandX; // go forward
double demandZ; // rotate

void velCallback(const geometry_msgs::Twist& vel);
void actionsCallback( const std_msgs::String& act_msg);

ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);
tf::TransformBroadcaster broadcaster;
ros::Subscriber<geometry_msgs::Twist> sub_cmdvel("cmd_vel", velCallback);
ros::Subscriber<std_msgs::String> sub_actions("actions", actionsCallback);


// tf variables to be broadcast
double x = 0;
double y = 0;
double theta = 0;

char base_link[] = "/base_link";
char odom[] = "/odom";

float Vavr;
float pos_total;


void setup() {
  pinMode(MOTOR_L_EN, OUTPUT);
  pinMode(MOTOR_L_FR, OUTPUT);

  pinMode(MOTOR_R_EN, OUTPUT);
  pinMode(MOTOR_R_FR, OUTPUT);
 
  digitalWrite(MOTOR_L_EN, LOW);
  digitalWrite(MOTOR_L_FR, HIGH);

  digitalWrite(MOTOR_R_EN, LOW);
  digitalWrite(MOTOR_R_FR, LOW);

  digitalWrite(TLED, OUTPUT);

  analogWriteDAC1(0);
  analogWriteDAC0(0);

  PIDL.setBangBang(30000);
  PIDL.setTimeStep(loopTimeMs-1);

  PIDR.setBangBang(30000);
  PIDR.setTimeStep(loopTimeMs-1);

  Serial3.begin(9600);

  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(sub_cmdvel);
  nh.subscribe(sub_actions);

  nh.advertise(odom_pub);
  broadcaster.init(nh);
}

void loop() {   
  
  nh.spinOnce();

  currentMillis = millis();

  if (currentMillis - previousMillis >= loopTimeMs) {
    previousMillis = currentMillis;     

    demandL = demandX - (demandZ * LBASE/2);
    SetpointL = demandL * PPT;
    demandR = demandX + (demandZ * LBASE/2);
    SetpointR = demandR * PPT;

    encoderLeftPos = -encoderLeft.read();  
    speed_act_L = encoderLeftPos/PPT;
    encoderLeft.write(0);
    encoderLeftError = SetpointL - encoderLeftPos;

    encoderRightPos = encoderRight.read();  
    speed_act_R = encoderRightPos/PPT;
    encoderRight.write(0);
    encoderRightError = SetpointR - encoderRightPos;

    InputL = encoderLeftPos;
    PIDL.run(); 
    InputR = encoderRightPos;
    PIDR.run(); 

    speed_act_L = speed_act_L /100;
    speed_act_R = speed_act_R /100;

    Vavr = (speed_act_L + speed_act_R) / 2;
    
    pos_total += Vavr;

    float phi = ((speed_act_R - speed_act_L) / LBASE);
    theta = theta + phi;

    if (theta >= TWO_PI){
      theta = theta - TWO_PI;
    }
    if (theta <= (-TWO_PI)){
      theta = theta + TWO_PI;
    }

    y += Vavr * sin(theta);
    x += Vavr * cos(theta);

    // broadcast odom->base_link transformation with tf
    geometry_msgs::TransformStamped t;
    t.header.frame_id = odom;
    t.child_frame_id = base_link;

    t.transform.translation.x = x; // in meters
    t.transform.translation.y = y; // in meters
    t.transform.translation.z = 0.0;

    t.transform.rotation = tf::createQuaternionFromYaw(theta);
    t.header.stamp = nh.now();

    broadcaster.sendTransform(t);

    //broadcast odom message
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = odom;
    odom_msg.child_frame_id = base_link;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);

    odom_msg.twist.twist.linear.x = Vavr;
    odom_msg.twist.twist.linear.y = phi;
    odom_pub.publish(&odom_msg);



    if (prev_OutputL > 0 && OutputL < 0){
      PIDL.reset();
    }
    else if (prev_OutputL < 0 && OutputL > 0){
      PIDL.reset();
    }

    if (prev_OutputR > 0 && OutputR < 0){
      PIDR.reset();
    }
    else if (prev_OutputR < 0 && OutputR > 0){
      PIDR.reset();
    }

    prev_OutputL = OutputL;
    prev_OutputR = OutputR; 

    #if defined (DEBUG)
      if (currentMillis - previousMillis_Serial >= 1000) {
        previousMillis_Serial = currentMillis; 
          Serial3.print("x: ");
          Serial3.print(x);
          Serial3.print(", y: ");
          Serial3.print(y);
          Serial3.print(", Vavr: ");
          Serial3.print(Vavr);
          Serial3.print(", phi: ");
          Serial3.print(phi);
          Serial3.print(" |  P: ");
          Serial3.print(PkL);
          Serial3.print(", I: ");
          Serial3.print(IkL);
          Serial3.print(", D: ");
          Serial3.print(DkL);
          Serial3.print(",  ||  SpL: ");
          Serial3.print(SetpointL);
          Serial3.print(", EL: ");
          Serial3.print(encoderLeftError);
          Serial3.print(", OUTL: ");
          Serial3.print(OutputL);
          Serial3.print(",   |  SpR: ");
          Serial3.print(SetpointR);
          Serial3.print(", ER: ");
          Serial3.print(encoderRightError);
          Serial3.print(", OUTR: ");
          Serial3.println(OutputR);
      }
    #endif

    //Left wheel control
    if (OutputL > 0){
      digitalWrite(MOTOR_L_FR, HIGH);
      analogWriteDAC1(MOTOR_L_BIAS+abs(OutputL));
      
    }
    else if (OutputL < 0){
      digitalWrite(MOTOR_L_FR, LOW);
      analogWriteDAC1(MOTOR_L_BIAS+abs(OutputL));
    }
    else{
      analogWriteDAC1(0);
    }

    
    //Right wheel control
    if (OutputR > 0){
      digitalWrite(MOTOR_R_FR, LOW);
      analogWriteDAC0(MOTOR_R_BIAS+abs(OutputR));
    }
    else if (OutputR < 0){
      digitalWrite(MOTOR_R_FR, HIGH);
      analogWriteDAC0(MOTOR_R_BIAS+abs(OutputR));
    }
    else{
      analogWriteDAC0(0);
    }

    if (demandX==0 && demandZ==0){
      PIDL.reset();
      PIDR.reset();
      analogWriteDAC0(0);
      analogWriteDAC1(0);
    }
  } // end if elapsed time
} // end loop


void velCallback(const geometry_msgs::Twist& vel){
  demandX = vel.linear.x;
  demandZ = vel.angular.z;

  demandX = constrain(demandX, -0.5, 0.5);
  demandZ = constrain(demandZ, -1.6, 1.6);
}

void actionsCallback( const std_msgs::String& act_msg){
  String str = String(act_msg.data);
  Serial3.println(str);
  if (str == String("toogle")){
    digitalWrite(TLED, !digitalRead(TLED));   // blink the led
  }
  else if (str == String("reset_odom")){
    x = 0.0;
    y = 0.0;
    theta = 0.0;
  }

}