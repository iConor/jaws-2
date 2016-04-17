#include "ros/ros.h"
#include "boost/asio.hpp"

#include "sensor_msgs/JointState.h"
#include "jaws2_msgs/PwmStamped.h"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

boost::asio::io_service i_o;
boost::asio::serial_port s_p(i_o);

void callback(const sensor_msgs::JointState::ConstPtr& angle, const jaws2_msgs::PwmStamped::ConstPtr& force)
{
  const int SIZE = 11;
  unsigned char packet[SIZE];

  int ang0 = 180 - int(angle->position[0] * 180 / M_PI);
  int ang1 = 180 + int(angle->position[1] * 180 / M_PI);

  packet[0]  = '-';

  packet[1]  = int(ang0) >> 8;
  packet[2]  = int(ang0);

  packet[3]  = int(ang1) >> 8;
  packet[4]  = int(ang1);

  packet[5]  = force->pwm.aft >> 8;
  packet[6]  = force->pwm.aft;

  packet[7]  = force->pwm.stbd >> 8;
  packet[8]  = force->pwm.stbd;

  packet[9]  = force->pwm.port >> 8;
  packet[10] = force->pwm.port;

  s_p.write_some(boost::asio::buffer(packet, SIZE));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_port");

  std::string port_name;
  int baud_rate;

  ros::NodeHandle nh;

  nh.param<std::string>("serial_port/name", port_name, "/dev/ttyUSB0");
  nh.param<int>("serial_port/rate", baud_rate, 9600);

  s_p.open(port_name);
  ROS_INFO("Serial port name: %s", port_name.c_str());
  s_p.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  ROS_INFO("Serial port rate: %i", baud_rate);

  message_filters::Subscriber<sensor_msgs::JointState> joint_sub(nh, "joint_states", 1);
  message_filters::Subscriber<jaws2_msgs::PwmStamped> thrust_sub(nh, "thrust_cal/pwm", 1);
  message_filters::TimeSynchronizer<sensor_msgs::JointState, jaws2_msgs::PwmStamped> sync(joint_sub, thrust_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
}
