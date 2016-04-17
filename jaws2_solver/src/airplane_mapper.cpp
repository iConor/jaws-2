#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "sensor_msgs/JointState.h"
#include "jaws2_msgs/ThrustStamped.h"

#include <math.h>

class Solver
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber accels;
    ros::Publisher joints;
    ros::Publisher forces;
    sensor_msgs::JointState angles;
    jaws2_msgs::ThrustStamped thrust;

  public:
    Solver(char** argv);
    void callback(const geometry_msgs::Accel::ConstPtr& a);
    void loop();
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "airplane_mapper");
  Solver solver(argv);
  solver.loop();
}

Solver::Solver(char** argv)
{
  accels = nh.subscribe<geometry_msgs::Accel>("accel", 1, &Solver::callback, this);
  joints = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  forces = nh.advertise<jaws2_msgs::ThrustStamped>("thrust", 1);

  angles.name.resize(2);
  angles.name[0] = "port-base";
  angles.name[1] = "stbd-base";
  angles.position.resize(2);
}

void Solver::callback(const geometry_msgs::Accel::ConstPtr& a)
{
  double lin_x = a->linear.x;
  double lin_z = a->linear.z;

  double ang_x = a->angular.x;
  double ang_y = a->angular.y;
  double ang_z = a->angular.z;

  ros::Time time = ros::Time::now();
  angles.header.stamp = time;
  angles.position[0] = ang_x*90 + 90.0;
  angles.position[1] = ang_x*90 + 90.0;
  joints.publish(angles);

  thrust.header.stamp = time;
  thrust.thrust.aft = ang_y;
  thrust.thrust.stbd = lin_x;
  thrust.thrust.port = lin_x;
  forces.publish(thrust);

  std::cout << "callback" << std::endl;
}

void Solver::loop()
{
  ros::spin();
}
