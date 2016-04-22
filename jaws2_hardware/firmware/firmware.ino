// ROS
#include <ros.h>
#include <jaws2_msgs/ServoCommand.h>
#include <jaws2_msgs/ServoState.h>
// Dynamixel
#include <ax12.h>
// PWM
#include <Servo.h>

// Servo Velocity/Effort Functions
#define GetSpeed(id) (ax12GetRegister(id, AX_PRESENT_SPEED_L, 2))
#define GetLoad(id) (ax12GetRegister(id, AX_PRESENT_LOAD_L, 2))


// Dynamixel ID's
const int PORT_SERVO = 18;
const int STBD_SERVO = 15;

// PWM Limits
const int MIN_THRUST = 1000;
const int ZERO_THRUST = 1500;
const int MAX_THRUST = 2000;

// PWM Pins
const int AFT_THRUSTER = 13;
const int PORT_THRUSTER = 12;
const int STBD_THRUSTER = 14;

// Dynamixel Encoder/Angle
const float ANGLE_CONVERSION = 4096.0/360.0;

// Startup Servo Angles
float port_servo_cmd = 150.0 * ANGLE_CONVERSION;
float stbd_servo_cmd = 150.0 * ANGLE_CONVERSION;
float port_servo_pos;
float stbd_servo_pos;

// Startup Thruser Forces
int port_thrust = ZERO_THRUST;
int stbd_thrust = ZERO_THRUST;
int aft_thrust = ZERO_THRUST;

// Thruster PWM Interfaces
Servo aft_thruster;
Servo port_thruster;
Servo stbd_thruster;

ros::NodeHandle nh;

void callback( const jaws2_msgs::ServoCommand& servos){
  
  port_servo_cmd = servos.port_cmd * ANGLE_CONVERSION;
  stbd_servo_cmd = servos.stbd_cmd * ANGLE_CONVERSION;
  
  digitalWrite(13, HIGH-digitalRead(13));
}

jaws2_msgs::ServoState state;
ros::Subscriber<jaws2_msgs::ServoCommand> command("servo/command", &callback);
ros::Publisher servos("servo/state", &state);

void setup()
{
  ax12Init(1000000);

  SetPosition(PORT_SERVO, port_servo_cmd);
  SetPosition(STBD_SERVO, stbd_servo_cmd);
    
  port_thruster.attach(PORT_THRUSTER);
  stbd_thruster.attach(STBD_THRUSTER);
  aft_thruster.attach(AFT_THRUSTER);

  port_thruster.writeMicroseconds(port_thrust);
  stbd_thruster.writeMicroseconds(stbd_thrust);
  aft_thruster.writeMicroseconds(aft_thrust);
    
  nh.initNode();
  nh.subscribe(command);
  nh.advertise(servos);
  
  pinMode(13, OUTPUT);
}

void loop()
{
  port_servo_pos = GetPosition(PORT_SERVO) / ANGLE_CONVERSION;
  stbd_servo_pos = GetPosition(STBD_SERVO) / ANGLE_CONVERSION;
  
  state.port_pos = port_servo_pos;
  state.stbd_pos = stbd_servo_pos;
  // Conversions?
  state.port_vel = GetSpeed(PORT_SERVO);
  state.stbd_vel = GetSpeed(STBD_SERVO);
  state.port_eff = GetLoad(PORT_SERVO);
  state.stbd_eff = GetLoad(STBD_SERVO);
  
  servos.publish( &state );

  if((abs(port_servo_pos - port_servo_cmd) < 48 ) && (abs(stbd_servo_pos - stbd_servo_cmd) < 48))
  {
    port_thruster.writeMicroseconds(port_thrust);
    stbd_thruster.writeMicroseconds(stbd_thrust);
    aft_thruster.writeMicroseconds(aft_thrust);
  }
  else
  {
    port_thruster.writeMicroseconds(ZERO_THRUST);
    stbd_thruster.writeMicroseconds(ZERO_THRUST);
    aft_thruster.writeMicroseconds(ZERO_THRUST);
  }

  nh.spinOnce();
  delay(33);
}

