#include <ros.h>
#include <std_msgs/Float64.h>

#include "ax12.h"
#include <Servo.h>

ros::NodeHandle nh;
std_msgs::Float64 state;
std_msgs::Float64 cmd;

int port_servo_cmd = 150;
int stbd_servo_cmd = 150;
int state[] = {150, 150};

void callback(const std_msgs::Float64 cmd);
{
  port_servo_cmd = cmd[0];
  stbd_servo_cmd = cmd[1];
  SetPosition(PORT_SERVO, port_servo_cmd * ANGLE_CONVERSION);
  SetPosition(STBD_SERVO, stbd_servo_cmd * ANGLE_CONVERSION);
  // set -- port, stbd, aft -- thrusters
}

ros::Publisher state_pub("state", &state);
ros::Subscriber<std_msgs::Float64> cmd_sub("command", &callback);

const int PORT_SERVO = 18;
const int STBD_SERVO = 15;
const int AFT_THRUSTER = 13;
const int PORT_THRUSTER = 12;
const int STBD_THRUSTER = 14;

const int MIN_THRUST = 1000;
const int ZERO_THRUST = 1500;
const int MAX_THRUST = 2000;

Servo aft_thruster;
Servo port_thruster;
Servo stbd_thruster;

const float ANGLE_CONVERSION = 4096.0/360.0;

void setup()
{
  nh.initNode();
  nh.advertise(state_pub);

  ax12Init(1000000);
  aft_thruster.attach(AFT_THRUSTER);
  port_thruster.attach(PORT_THRUSTER);
  stbd_thruster.attach(STBD_THRUSTER);

  SetPosition(PORT_SERVO, 150 * ANGLE_CONVERSION);
  SetPosition(STBD_SERVO, 150 * ANGLE_CONVERSION);
  aft_thruster.writeMicroseconds(ZERO_THRUST);
  port_thruster.writeMicroseconds(ZERO_THRUST);
  stbd_thruster.writeMicroseconds(ZERO_THRUST);
}

void loop()
{
  nh.spinOnce();
  port_servo_pos = GetPosition(PORT_SERVO);
  stbd_servo_pos = GetPosition(STBD_SERVO);
  if(port_servo_pos == port_servo_cmd && stbd_servo_pos == stbd_servo_cmd)
  {
    port_thruster.writeMicroseconds(port_thrust);
    stbd_thruster.writeMicroseconds(stbd_thrust);
    aft_thruster.writeMicroseconds(aft_thrust);
  }
  else // Thrusters not in position yet:
  {
    port_thruster.writeMicroseconds(ZERO_THRUST);
    stbd_thruster.writeMicroseconds(ZERO_THRUST);
    aft_thruster.writeMicroseconds(ZERO_THRUST);
  }
  state[0] = port_servo_pos;
  state[1] = stbd_servo_pos;
  state_pub.publish(&state);
  delay(33);
}
