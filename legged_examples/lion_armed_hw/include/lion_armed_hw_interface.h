
#pragma once

#include <legged_hw/hardware_interface.h>
#include "sdk/comm.h"
#include "sdk/quadruped.h"
#include "sdk/actuatorcontroller.h"
#include <signal.h>
#include <thread>
#include <chrono>


namespace legged
{

  void paramFeedback(ActuatorController::UnifiedID uID,uint8_t paramType,double paramValue);
const std::vector<std::string> CONTACT_SENSOR_NAMES = { "RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT" };

struct LionMotorData
{
  double pos_, vel_, tau_;                   // state
  double pos_des_, vel_des_, kp_, kd_, ff_;  // command
};

struct LionImuData
{
  double ori[4];
  double ori_cov[9];
  double angular_vel[3];
  double angular_vel_cov[9];
  double linear_acc[3];
  double linear_acc_cov[9];
};

class LionArmedHW : public LeggedHW
{
public:
  LionArmedHW() = default;
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * Call @ref UNITREE_LEGGED_SDK::UDP::Recv() to get robot's state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref UNITREE_LEGGED_SDK::UDP::Recv(). Publish actuator
   * current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

  ~LionArmedHW();

private:
  bool setupJoints();

  bool setupImu();

  bool setupContactSensor(ros::NodeHandle& nh);

  void enableArmMotors();

  static void processSignal(int sign);



//   std::shared_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
//   std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safety_;
  UNITREE_LEGGED_SDK::LowState low_state_{};
  UNITREE_LEGGED_SDK::LowCmd low_cmd_{};

  LionMotorData joint_data_[18]{};
  LionImuData imu_data_{};
  bool contact_state_[4]{};

  int power_limit_{};
  int contact_threshold_{};

  ActuatorController * pController_;
  std::vector<ActuatorController::UnifiedID> arm_uID_array_;
  static bool bExit;
  ActuatorMode mode_;

};
  extern int count,cur1_count,cur2_count,cur3_count,cur4_count,cur5_count,cur6_count;
  extern int vel1_count,vel2_count,vel3_count,vel4_count,vel5_count,vel6_count;
  extern int pos1_count,pos2_count,pos3_count,pos4_count,pos5_count,pos6_count;
extern double cur[6],vel[6],pos[6];

}  // namespace legged


