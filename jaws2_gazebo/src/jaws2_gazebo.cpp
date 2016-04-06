#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "jaws2_msgs/ThrustStamped.h"

class Sim
{
  private:
    ros::NodeHandle nh;
    ros::Publisher port_wrench;
    ros::Publisher stbd_wrench;
    ros::Publisher aft_wrench;
    ros::Subscriber thrust_sub;
    geometry_msgs::Wrench wrench_p;
    geometry_msgs::Wrench wrench_s;
    geometry_msgs::Wrench wrench_a;

  public:
    Sim();
    void sim_callback(const jaws2_msgs::ThrustStamped::ConstPtr& wrench);
    void loop();
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim");
  Sim sim;
  sim.loop();
}

Sim::Sim()
{
  thrust_sub = nh.subscribe<jaws2_msgs::ThrustStamped>("solver/thrust", 1, &Sim::sim_callback, this);
  port_wrench = nh.advertise<geometry_msgs::Wrench>("port_thrust", 1);
  stbd_wrench = nh.advertise<geometry_msgs::Wrench>("stbd_thrust", 1);
  aft_wrench = nh.advertise<geometry_msgs::Wrench>("aft_thrust", 1);
}

void Sim::sim_callback(const jaws2_msgs::ThrustStamped::ConstPtr& force)
{
  wrench_p.force.x = force->thrust.port;
  wrench_s.force.x = force->thrust.stbd;
  wrench_a.force.z = force->thrust.aft;

  port_wrench.publish(wrench_p);
  stbd_wrench.publish(wrench_s);
  aft_wrench.publish(wrench_a);
}

void Sim::loop()
{
  ros::Rate rate(30);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
