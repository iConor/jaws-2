#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Accel.h"

class Accel
{
  private:
    ros::NodeHandle nh;
    ros::Publisher accels;
    ros::Subscriber js;
    geometry_msgs::Accel a;

  public:
    Accel();
    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
    void loop();
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "airplane");
  Accel accel;
  accel.loop();
}

Accel::Accel()
{
  js = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Accel::joy_callback, this);
  accels = nh.advertise<geometry_msgs::Accel>("accel", 1);
}

void Accel::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  a.linear.x = joy->axes[1];
  a.linear.y = 0;
  a.linear.z = (joy->axes[14] - joy->axes[15]);

  a.angular.x = joy->axes[2] * -1;
  a.angular.y = joy->axes[3];
  a.angular.z = joy->axes[13] - joy->axes[12];

  accels.publish(a);
}

void Accel::loop()
{
  ros::Rate rate(30);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
