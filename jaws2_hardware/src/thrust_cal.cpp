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
    int calibrate(double raw_force, double fwd_cal, double rev_cal);

  public:
    ThrustCal();
    void callback(const jaws2_msgs::ThrustStamped::ConstPtr& force);
    void loop();
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thrust_cal");
  ThrustCal thrust_cal;
  thrust_cal.loop();
}

ThrustCal::ThrustCal() : nh()
{
  ts = nh.subscribe<jaws2_msgs::ThrustStamped>("solver/thrust", 1, &ThrustCal::callback, this);
  pwm = nh.advertise<jaws2_msgs::PwmStamped>("thrust_cal/pwm", 1);

  nh.param<double>("nominal_max_force", max_force, 25.0);
  nh.param<double>("nominal_max_pwm", max_pwm, 100.0);

  nh.param<double>("thrust_cal/aft_fwd", aft_fwd, 1.0);
  nh.param<double>("thrust_cal/aft_rev", aft_rev, 1.0);
  nh.param<double>("thrust_cal/stbd_fwd", stbd_fwd, 1.0);
  nh.param<double>("thrust_cal/stbd_rev", stbd_rev, 1.0);
  nh.param<double>("thrust_cal/port_fwd", port_fwd, 1.0);
  nh.param<double>("thrust_cal/port_rev", port_rev, 1.0);
}

void ThrustCal::callback(const jaws2_msgs::ThrustStamped::ConstPtr& force)
{
  duration.header.stamp = force->header.stamp;

  duration.pwm.aft = calibrate(force->thrust.aft, aft_fwd, aft_rev);
  duration.pwm.stbd = calibrate(force->thrust.stbd, stbd_fwd, stbd_rev);
  duration.pwm.port = calibrate(force->thrust.port, port_fwd, port_rev);

  pwm.publish(duration);
}

void ThrustCal::loop()
{
  ros::spin();
}

int ThrustCal::calibrate(double raw_force, double fwd_cal, double rev_cal)
{
  if(raw_force < 0.0)
  {
    raw_force /= rev_cal;
  }
  else
  {
    raw_force /= fwd_cal;
  }
  return 1500 + int(raw_force * max_pwm / max_force);
}
