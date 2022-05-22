/*********************************************************************

 *********************************************************************/

/* Author: Brock Cooper

*/

#ifndef BCR1_HW_INTERFACE_H
#define BCR1_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>

#include <bcr1_control/armCmd.h>
#include <bcr1_control/bcr1Telemetry.h>

// #include <effort_controllers/joint_position_controller.h>

#define DEG_TO_RAD 0.01745329251
#define RAD_TO_DEG 57.2957795131


namespace bcr1_ns
{
/** \brief Hardware interface for a robot */
class bcr1HWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  bcr1HWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Initialize the robot hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);

protected:
ros::Subscriber telemetry_sub;
void telemetryCallback(const bcr1_control::bcr1Telemetry::ConstPtr &msg);

ros::Publisher cmd_pub;
};  // class

}  // namespace ros_control_boilerplate

#endif
