#include "ros/ros.h"
#include "jaws2_msgs/ThrustStamped.h"
#include "jaws2_msgs/PwmStamped.h"

class ThrustCal
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber ts;
    ros::Publisher pwm;
    jaws2_msgs::PwmStamped duration;
    double max_force;
    double max_pwm;
    double aft_fwd;
    double aft_rev;
    double stbd_fwd;
    double stbd_rev;
    double port_fwd;
    double port_rev;
    int aft(double raw_force);
    int stbd(double raw_force);
    int port(double raw_force);

  public:
    ThrustCal();
    void callback(const jaws2_msgs::ThrustStamped::ConstPtr& force);
    void loop();
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "airplane_cal");
  ThrustCal thrust_cal;
  thrust_cal.loop();
}

ThrustCal::ThrustCal() : nh()
{
  ts = nh.subscribe<jaws2_msgs::ThrustStamped>("thrust", 1, &ThrustCal::callback, this);
  pwm = nh.advertise<jaws2_msgs::PwmStamped>("pwm", 1);
}

void ThrustCal::callback(const jaws2_msgs::ThrustStamped::ConstPtr& force)
{
  duration.header.stamp = force->header.stamp;

  duration.pwm.aft = aft(force->thrust.aft);
  duration.pwm.stbd = stbd(force->thrust.stbd);
  duration.pwm.port = port(force->thrust.port);

  pwm.publish(duration);
}

void ThrustCal::loop()
{
  ros::spin();
}

int ThrustCal::aft(double raw_force)
{
  int pwm = 1500;
  if(raw_force < -0.75)
  {
    pwm = 1360;
  }
  else if(raw_force > 0.75)
  {
    pwm = 1690;
  }
  return pwm;
}

int ThrustCal::stbd(double raw_force)
{
  int pwm = 1500;
  if(raw_force < -0.25)
  {
    pwm = int(1390.0 + raw_force * 15.0);
  }
  else if(raw_force > 0.25)
  {
    pwm = int(1550 + raw_force * 40.0);
  }
  return pwm;
}

int ThrustCal::port(double raw_force)
{
  int pwm = 1500;
  if(raw_force < -0.25)
  {
    pwm = int(1390.0 + raw_force * 20.0);
  }
  else if(raw_force > 0.25)
  {
    pwm = int(1520.0 + raw_force * 30.0);
  }
  return pwm;
}

