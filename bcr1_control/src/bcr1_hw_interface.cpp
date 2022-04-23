/*********************************************************************

 *********************************************************************/

/* Author: Brock Cooper

*/

#include <bcr1_control/bcr1_hw_interface.h>

namespace bcr1_ns
{
  bcr1HWInterface::bcr1HWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
      : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
  {
    telemetry_sub = nh.subscribe("/arduino/bcr1Telemetry", 1, &bcr1HWInterface::telemetryCallback, this);

    cmd_pub = nh.advertise<bcr1_control::armCmd>("arduino/armCmd", 1);

    ROS_INFO("bcr1HWInterface constructed");
  }

  void bcr1HWInterface::telemetryCallback(const bcr1_control::bcr1Telemetry::ConstPtr &msg)
  {

    /*
    float32[6] angle # degrees

    States
      std::vector<double> joint_position_;
      std::vector<double> joint_velocity_;
      std::vector<double> joint_effort_;
    */
    for (int i = 0; i < num_joints_; i++)
    {
      joint_position_[i] = msg->angle[i] * DEG_TO_RAD;
    }
  }

  void bcr1HWInterface::init()
  {
    // Call parent class version of this function
    GenericHWInterface::init();

    ROS_INFO("bcr1HWInterface Ready.");
  }

  void bcr1HWInterface::read(ros::Duration &elapsed_time)
  {
    // No need to read since our write() command populates our state for us
    ros::spinOnce();
  }

  void bcr1HWInterface::write(ros::Duration &elapsed_time)
  {
    // Safety
    // enforceLimits(elapsed_time);

    /*
    float32[6] effort # 0-255 PWM forArduino
    float32[6] angle # deg
    uint32 msg_ctr # count sent msgs to detect missed messages

    Commands
      std::vector<double> joint_position_command_;
      std::vector<double> joint_velocity_command_;
      std::vector<double> joint_effort_command_;
    */

    eff_jnt_sat_interface_.enforceLimits(elapsed_time);

    // joint_effort_limits_[0]=10;
    static bcr1_control::armCmd arm_cmd;
    for (int i = 0; i < num_joints_; i++)
    {
      arm_cmd.effort[i] = joint_effort_command_[i];
      //  arm_cmd.effort[i] = joint_position_[i]; // testing reading robot angles into effort message
      arm_cmd.angle[i] = joint_position_command_[i] * RAD_TO_DEG;
    }

    cmd_pub.publish(arm_cmd);
  }

  void bcr1HWInterface::enforceLimits(ros::Duration &period)
  {
    // Enforces position and velocity
    // pos_jnt_sat_interface_.enforceLimits(period);
    // eff_jnt_sat_interface_.enforceLimits(period);
    // use_rosparam_joint_limits_=true;
    // use_soft_limits_if_available_=true;
  }
} // namespace bcr1_ns
