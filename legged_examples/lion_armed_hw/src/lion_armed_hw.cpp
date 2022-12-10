#include <legged_hw/control_loop.h>
#include "lion_armed_hw_interface.h"

int main(int argc, char** argv)
{
  std::cout<<"Starting ros node ..."<<std::endl;
  ros::init(argc, argv, "lion_armed_hw");
  ros::NodeHandle nh;
  ros::NodeHandle robot_hw_nh("~");
  
  // Run the hardware interface node
  // -------------------------------

  // We run the ROS loop in a separate thread as external calls, such
  // as service callbacks loading controllers, can block the (main) control loop
  std::cout<<"Starting spinner ..."<<std::endl;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  try
  {
    // Create the hardware interface specific to your robot
    std::shared_ptr<legged::LionArmedHW> lion_armed_hw = std::make_shared<legged::LionArmedHW>();
    // Initialise the hardware interface:
    // 1. retrieve configuration from rosparam
    // 2. initialize the hardware and interface it with ros_control
    lion_armed_hw->init(nh, robot_hw_nh);

    // Start the control loop
    std::cout<<"Starting control loop ..."<<std::endl;
    legged::LeggedHWLoop control_loop(nh, lion_armed_hw);

    // Wait until shutdown signal received
    ros::waitForShutdown();
  }
  catch (const ros::Exception& e)
  {
    ROS_FATAL_STREAM("Error in the hardware interface:\n"
                     << "\t" << e.what());
    return 1;
  }

  return 0;
}
