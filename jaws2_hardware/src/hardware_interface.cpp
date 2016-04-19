#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

class Jaws2 : public hardware_interface::RobotHW
{
  private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    double cmd[2];
    double pos[2];
    double vel[2];
    double eff[2];
    ros::Time previous_time;

  public:
    Jaws2();
    void read();
    void write();
    const ros::Time get_time();
    const ros::Duration get_period();
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hardware_interface");
  Jaws2 jaws2;

  controller_manager::ControllerManager ctrl_mngr(&jaws2/*, const ros::NodeHandle* nh*/);

  ros::Rate rate(100);
  while(ros::ok())
  {
    // The control manager needs to be moved into
    // some sort of callback to arrange pubs/subs.
    jaws2.read();
    ctrl_mngr.update(jaws2.get_time(), jaws2.get_period()/*, bool reset_controllers*/);
    jaws2.write();

    rate.sleep();
  }
}

Jaws2::Jaws2()
{
  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_port_servo("port_servo", &pos[0], &vel[0], &eff[0]);
  jnt_state_interface.registerHandle(state_handle_port_servo);

  hardware_interface::JointStateHandle state_handle_stbd_servo("stbd_servo", &pos[1], &vel[1], &eff[1]);
  jnt_state_interface.registerHandle(state_handle_stbd_servo);

  registerInterface(&jnt_state_interface);

  // connect and register the joint position interface
  hardware_interface::JointHandle pos_handle_port_servo(jnt_state_interface.getHandle("port_servo"), &cmd[0]);
  jnt_pos_interface.registerHandle(pos_handle_port_servo);

  hardware_interface::JointHandle pos_handle_stbd_servo(jnt_state_interface.getHandle("stbd_servo"), &cmd[1]);
  jnt_pos_interface.registerHandle(pos_handle_stbd_servo);

  registerInterface(&jnt_pos_interface);
}

void Jaws2::read()
{
  // serial_read
  // double pos[2];
  // double vel[2];
  // double eff[2];
}

void Jaws2::write()
{
  // serial_write
  // double cmd[2];
}

const ros::Time Jaws2::get_time()
{
  return ros::Time::now();
}

const ros::Duration Jaws2::get_period()
{
  ros::Time current_time = ros::Time::now();
  const ros::Duration period = current_time - previous_time;
  previous_time = current_time;
  return period;
}
