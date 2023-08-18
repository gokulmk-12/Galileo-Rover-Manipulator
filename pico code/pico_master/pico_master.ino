#include<ros.h>
#include<std_msgs/Float32.h>

#define LED 25
#define egr 25
#define ENCA 2 // connect the follwing pins to digital input of pico
#define ENCB 3 //ENCA and ENCB are used to get encoder outputs
#define PPR 9881 //change this constant according to ppr mentioned in datasheet , this one ppr is for one of the motors of our arm
#define PWM 5
#define IN1 7 
#define IN2 6

using namespace ros;
using namespace std_msgs;


void elbow_cb(const Float32& msg){
  float joint_angle;
  joint_angle = msg.data;
  float setpoint = (egr*PPR)*joint_angle/(2*3.14);
  elbow_controller(setpoint);  
}

// initialising node and a subscriber 
NodeHandle nh;
Subscriber<Float32> sub("/elbow_joint_cmd", &elbow_cb);


// initialsing a few variables
int pos  = 0; //used to look after rotated angle from encoder feedback
int dir = 0;
int pwmVal= 0;

// function to write the output to motors
void setMotor(int dir,int pwmVal){
  analogWrite(PWM,pwmVal);
  if (dir>0){
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
  }else if(dir<0){  
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
  }else{
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
  }  
}

void elbow_controller(float setpoint){
  while(abs(setpoint-pos)>2){
    int e = setpoint - pos;
    if(e>0){
      dir = 1;
    }else if(e<0){
      dir = -1;
    }else{
      dir = 0;
    }
    setMotor(dir,e);            
  }
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b==0){
    pos++;
  }else{
    pos--;
  }
}

void setup() {
  
  Serial.begin(9600);
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.subscribe(sub);
  
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING); 

  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  setMotor(dir,pwmVal);
  
}

void loop() {
  int a = digitalRead(ENCA);
  int b = digitalRead(ENCB);
  nh.spinOnce();
  Serial.println(pos);  
}
