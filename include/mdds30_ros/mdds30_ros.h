/*
* Created by Sam
* Written as an interface to the MDDS30 DC motor driver
*
*
*/

#ifndef MDDS30_ROS_H
#define MDDS30_ROS_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

// Serial system includes
// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
#include <mdds30_ros/mdds30_rosConfig.h>

// Other ROS includes
#include <geometry_msgs/TwistStamped.h>

class MDDS30
{
public:
  //! Constructor.
  MDDS30(ros::NodeHandle private_nh, ros::NodeHandle public_nh);

  //! Destructor.
  ~MDDS30();

  //! Callback function for dynamic reconfigure server.
  void configCallback(mdds30_ros::mdds30_rosConfig &config, uint32_t level);

  //! Message subscriber.
  void twistCallback(const geometry_msgs::TwistStamped &msg);

private:

  ros::Subscriber twist_sub_; /* ROS twist subsriber */

  int serial_fd_; /* File descriptor that connects to serial device.*/

};

#endif // MDDS30_ROS_H
