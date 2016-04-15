#include <ros.h>
#include <std_msgs/Float64.h>

#include "ax12.h"
#include <Servo.h>

ros::NodeHandle nh;
std_msgs::Float64 state;
std_msgs::Float64 cmd;

void callback(const std_msgs::Float64 cmd);
{
  SetPosition(PORT_SERVO, cmd[0] * ANGLE_CONVERSION);
  SetPosition(STBD_SERVO, cmd[1] * ANGLE_CONVERSION);
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
  state = 0;
  if(state != 0)
  {
    port_thruster.writeMicroseconds(port_thrust);
    stbd_thruster.writeMicroseconds(stbd_thrust);
    aft_thruster.writeMicroseconds(aft_thrust);
  }
  else // state == 0;
  {
    port_thruster.writeMicroseconds(ZERO_THRUST);
    stbd_thruster.writeMicroseconds(ZERO_THRUST);
    aft_thruster.writeMicroseconds(ZERO_THRUST);
  }
  state_pub.publish(&state);
  delay(33);
}
