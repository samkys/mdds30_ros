/*
* Created by Sam
* Written as an interface to the MDDS30 DC motor driver
*
*
*/
#include "mdds30_ros/mdds30_ros.h"

// Constructor
MDDS30::MDDS30(ros::NodeHandle private_nh, ros::NodeHandle public_nh)
{
  std::string serial_port_name;
  private_nh.param<std::string>("/serial_port",serial_port_name,"/dev/ttyUSB0");

  twist_sub_ = public_nh.subscribe("/mdds30/cmd/twist", 1, &MDDS30::twistCallback, this);
  serial_fd_ = open(serial_port_name.c_str(), O_RDWR);

  // Check for errors
  if (serial_fd_ < 0)
  {
    ROS_ERROR("Error %i from open: %s\n", errno, strerror(errno));
    ROS_WARN("Is your user part of dialout?\n");
    ROS_WARN("Try running: sudo adduser $USER dialout");
    exit(0);
  }

  // Set up the serial write correctly
  struct termios tty;

  // Read in existing settings, and handle any error
  // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
  // must have been initialized with a call to tcgetattr() overwise behaviour
  // is undefined
  if(tcgetattr(serial_fd_, &tty) != 0)
  {
    ROS_ERROR("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  /*
  struct termios {
  tcflag_t c_iflag;    // input mode flags
  tcflag_t c_oflag;    // output mode flags
  tcflag_t c_cflag;    // control mode flags
  tcflag_t c_lflag;    // local mode flags
  cc_t c_line;         // line discipline
  cc_t c_cc[NCCS];     // control characters
  };
  */

  tty.c_cflag &= ~PARENB;        // Clear parity bit
  tty.c_cflag &= ~CSTOPB;        // Set single stop bit
  tty.c_cflag &= ~CSIZE;         // Clear all the size bits
  tty.c_cflag |= CS8;            // 8 bits per byte
  tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  tty.c_lflag &= ~ICANON;        // Disable cononical mode (read on newline)
  tty.c_lflag &= ~ECHO;          // Disable echo
  tty.c_lflag &= ~ECHOE;         // Disable erasure
  tty.c_lflag &= ~ECHONL;        // Disable new-line echo
  tty.c_lflag &= ~ISIG;          // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)
  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;
  // Set baud rate to 115200 baud
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    ROS_ERROR("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
}

// Destructor
MDDS30::~MDDS30()
{
} // end ~NodeExample()


// Message callback
void MDDS30::twistCallback(const geometry_msgs::TwistStamped &msg)
{
  unsigned char serial_msg[] = { 'H' };
  write(serial_fd_, serial_msg, sizeof(serial_msg));
}

/*--------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *------------------------------------------------------------------*/

void MDDS30::configCallback(mdds30_ros::mdds30_rosConfig &config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
  //message = config.message.c_str();
  //a = config.a;
  //b = config.b;
} // end configCallback()


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "mdds30");
  ros::NodeHandle public_nh;
  ros::NodeHandle private_nh("~");

  MDDS30 mddriver(public_nh,private_nh);

  // Main loop.
  ros::spin();
  return 0;
} // end main()
