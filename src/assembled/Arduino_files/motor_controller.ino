#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

// stepper motor 
#include <AccelStepper.h>
#include <AFMotor.h>
// definitions of const 
#define MSTEPS 200 //step/second 
#define MAXSpeed 500
int analogBumperRight = A5;
int analogBumperLeft = A0;
unsigned long current_millies, rprevious_millies, lprevious_millies, timeout_millies;
// two stepper motors one on each port
AF_Stepper motor1(MSTEPS, 1);
AF_Stepper motor2(MSTEPS, 2);
// Motor shield has two motor ports, now we'll wrap them in an AccelStepper object
// options: SINGLE to DOUBLE or INTERLEAVE or MICROSTEP!
void forwardstep1() { motor1.onestep(FORWARD, SINGLE); }
void backwardstep1() {motor1.onestep(BACKWARD, SINGLE); }
// wrappers for the second motor!
void forwardstep2() {motor2.onestep(FORWARD, SINGLE);}
void backwardstep2() {motor2.onestep(BACKWARD, SINGLE);}

// STEPPER OBJECTS
AccelStepper stepper1(forwardstep1, backwardstep1);
AccelStepper stepper2(forwardstep2, backwardstep2);
float v_r, v_l;

ros::NodeHandle nh;
// ros::Publisher chatter("chatter", &msg);

void rw_callback(std_msgs::Float64 &vtarget){
  v_r = vtarget.data;
  rprevious_millies = current_millies;
}
void lw_callback(std_msgs::Float64 &vtarget){
  v_l = vtarget.data;
  lprevious_millies = current_millies;
}

ros::Subscriber<std_msgs::Float64> rw_sub("/rwheel_vtarget", &rw_callback );
ros::Subscriber<std_msgs::Float64> lw_sub("/lwheel_vtarget", &lw_callback );
std_msgs::Bool msgR, msgL;
ros::Publisher lBumper("/assembled/bumper_left", &msgL);
ros::Publisher rBumper("/assembled/bumper_right", &msgR);
int baud_rate{57600};
void setup()
{  
  // start serial connection, default is 9600
  Serial.begin(baud_rate);           // set up Serial library at ros baud rate bps
  nh.initNode();
  nh.subscribe(rw_sub);
  nh.subscribe(lw_sub);
  nh.advertise(lBumper);
  nh.advertise(rBumper);
  stepper1.setMaxSpeed(MAXSpeed);     
  stepper2.setMaxSpeed(MAXSpeed);
  // start time
  current_millies = rprevious_millies = lprevious_millies =  millis();
  timeout_millies = 500;
}

void loop()
{ 
  nh.spinOnce();
  current_millies =  millis();
  // setting speed
  stepper1.setSpeed(v_r);
  stepper2.setSpeed(v_l);

  // sense bumpers and send them over the wire
  // resolution of analog is 1024
  msgR.data = (analogRead(analogBumperRight) < 512)? false: true;
  msgL.data = (analogRead(analogBumperLeft) < 512)? false: true;
  rBumper.publish(&msgR);
  lBumper.publish(&msgL);
  const bool timout = ((current_millies - lprevious_millies) > timeout_millies
                    || (current_millies - rprevious_millies) > timeout_millies);
  if ((v_l != 0.0 || v_r != 0) && !timout){
    stepper1.runSpeed();
    stepper2.runSpeed();  

  } else {
    // timeout, stop motors
    motor1.release();
    motor2.release();
  }

  // we only want this to work at 10 Hz, we sleep for 99 ms
  delay(99);
}
