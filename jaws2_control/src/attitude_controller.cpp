#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "imu_3dm_gx4/FilterOutput.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

ros::NodeHandle nh;

geometry_msgs::Vector3 angular_accel;

ros::Subscriber get_target;
ros::Subscriber orientation;
ros::Subscriber velocity;
ros::Publisher command;

struct Vector3
{
  double x;
  double y;
  double z;
};

Vector3 target;
Vector3 target_dot;
Vector3 state;
Vector3 state_dot;

void set_target(const geometry_msgs::Vector3::ConstPtr& vector)
{
  target.x = vector->x;
  target.y = vector->y;
  target.z = vector->z;
}

void set_orientation(const imu_3dm_gx4::FilterOutput::ConstPtr& filter)
{
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(filter->orientation, quaternion);
  tf::Matrix3x3 angles = tf::Matrix3x3(quaternion);
  angles.getRPY(state.x, state.y, state.z);
}

void set_velocity(const sensor_msgs::Imu::ConstPtr& imu)
{
  state_dot.x = imu->angular_velocity.x;
  state_dot.y = imu->angular_velocity.y;
  state_dot.z = imu->angular_velocity.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "attitude_controller");

  target_dot.x = 0;
  target_dot.y = 0;
  target_dot.z = 0;

  control_toolbox::Pid roll;
  control_toolbox::Pid pitch;
  control_toolbox::Pid yaw;

  roll.initPid(6.0, 1.0, 2.0, 0.3, -0.3);
  pitch.initPid(6.0, 1.0, 2.0, 0.3, -0.3);
  yaw.initPid(6.0, 1.0, 2.0, 0.3, -0.3);

  get_target = nh.subscribe<geometry_msgs::Vector3>("target_attitude", 1, set_target);
  orientation = nh.subscribe<imu_3dm_gx4::FilterOutput>("state/filter", 1, set_orientation);
  velocity = nh.subscribe<sensor_msgs::Imu>("state/imu", 1, set_velocity);
  command = nh.advertise<geometry_msgs::Vector3>("command_attitude", 1);

  ros::Rate rate(100);

  ros::Time this_time;
  ros::Time last_time = ros::Time::now();

  while(ros::ok())
  {
    this_time = ros::Time::now();

    ros::Duration dt = this_time - last_time;

    angular_accel.x = roll.computeCommand(state.x - target.x, state_dot.x - target_dot.x, dt);
    angular_accel.y = pitch.computeCommand(state.y - target.y, state_dot.y - target_dot.y, dt);
    angular_accel.z = yaw.computeCommand(state.z - target.z, state_dot.z - target_dot.z, dt);

    command.publish(angular_accel);

    last_time = this_time;

    rate.sleep();
  }

  ros::spin();
}
