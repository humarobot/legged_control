
#include "lion_armed_hw_interface.h"


namespace legged
{

  int count,cur1_count,cur2_count,cur3_count,cur4_count,cur5_count,cur6_count;
  int vel1_count,vel2_count,vel3_count,vel4_count,vel5_count,vel6_count;
  int pos1_count,pos2_count,pos3_count,pos4_count,pos5_count,pos6_count;
  double cur[6],vel[6],pos[6];
  

void paramFeedback(ActuatorController::UnifiedID uID,uint8_t paramType,double paramValue){
  // cout<<"uID: "<<(int)uID.actuatorID<<endl;
  switch ((int)uID.actuatorID){
    case 1:
      switch (paramType) {
        case Actuator::ACTUAL_CURRENT:
            cur1_count++;
            cur[0]=paramValue;
            // cout<<"c: "<<cur_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " current is " << paramValue << "A"<<endl;
            break;
        case Actuator::ACTUAL_POSITION:
            // cout<<"id1: "<<cur1_index<<" "<<pos1_index<<" "<<vel1_index<<endl;
            pos1_count++;
            pos[0]=paramValue;
            // cout<<"p: "<<pos_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " position is " << paramValue << "R"<<endl;
            break;
        case Actuator::ACTUAL_VELOCITY:
            vel1_count++;
            vel[0]=paramValue;
            // cout<<"v: "<<vel_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " velocity is " << paramValue << "RPM"<<endl;
            break;
        default:
            break;
        }
      break;
    case 2:
      switch (paramType) {
        case Actuator::ACTUAL_CURRENT:
            cur2_count++;
            cur[1]=paramValue;
            // cout<<"c: "<<cur_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " current is " << paramValue << "A"<<endl;
            break;
        case Actuator::ACTUAL_POSITION:
            // cout<<"id2: "<<cur2_index<<" "<<pos2_index<<" "<<vel2_index<<endl;
            pos2_count++;
            pos[1]=paramValue;
            // cout<<"p: "<<pos_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " position is " << paramValue << "R"<<endl;
            break;
        case Actuator::ACTUAL_VELOCITY:
            vel2_count++;
            vel[1]=paramValue;
            // cout<<"v: "<<vel_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " velocity is " << paramValue << "RPM"<<endl;
            break;
        default:
            break;
        }
      break;
    case 3:
      switch (paramType) {
        case Actuator::ACTUAL_CURRENT:
            cur3_count++;
            cur[2]=paramValue;
            // cout<<"c: "<<cur_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " current is " << paramValue << "A"<<endl;
            break;
        case Actuator::ACTUAL_POSITION:
            // cout<<"id3: "<<cur3_index<<" "<<pos3_index<<" "<<vel3_index<<endl;
            pos3_count++;
            pos[2]=paramValue;
            // cout<<"p: "<<pos_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " position is " << paramValue << "R"<<endl;
            break;
        case Actuator::ACTUAL_VELOCITY:
            vel3_count++;
            vel[2]=paramValue;
            // cout<<"v: "<<vel_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " velocity is " << paramValue << "RPM"<<endl;
            break;
        default:
            break;
        }
      break;
    case 4:
      switch (paramType) {
        case Actuator::ACTUAL_CURRENT:
            cur4_count++;
            cur[3]=paramValue;
            // cout<<"c: "<<cur_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " current is " << paramValue << "A"<<endl;
            break;
        case Actuator::ACTUAL_POSITION:
            // cout<<"id1: "<<cur1_index<<" "<<pos1_index<<" "<<vel1_index<<endl;
            pos4_count++;
            pos[3]=paramValue;
            // cout<<"p: "<<pos_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " position is " << paramValue << "R"<<endl;
            break;
        case Actuator::ACTUAL_VELOCITY:
            vel4_count++;
            vel[3]=paramValue;
            // cout<<"v: "<<vel_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " velocity is " << paramValue << "RPM"<<endl;
            break;
        default:
            break;
        }
      break;
    case 5:
      switch (paramType) {
        case Actuator::ACTUAL_CURRENT:
            cur5_count++;
            cur[4]=paramValue;
            // cout<<"c: "<<cur_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " current is " << paramValue << "A"<<endl;
            break;
        case Actuator::ACTUAL_POSITION:
            // cout<<"id2: "<<cur2_index<<" "<<pos2_index<<" "<<vel2_index<<endl;
            pos5_count++;
            pos[4]=paramValue;
            // cout<<"p: "<<pos_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " position is " << paramValue << "R"<<endl;
            break;
        case Actuator::ACTUAL_VELOCITY:
            vel5_count++;
            vel[4]=paramValue;
            // cout<<"v: "<<vel_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " velocity is " << paramValue << "RPM"<<endl;
            break;
        default:
            break;
        }
      break;
    case 6:
      switch (paramType) {
        case Actuator::ACTUAL_CURRENT:
            cur6_count++;
            cur[5]=paramValue;
            // cout<<"c: "<<cur_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " current is " << paramValue << "A"<<endl;
            break;
        case Actuator::ACTUAL_POSITION:
            // cout<<"id3: "<<cur3_index<<" "<<pos3_index<<" "<<vel3_index<<endl;
            pos6_count++;
            pos[5]=paramValue;
            // cout<<"p: "<<pos_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " position is " << paramValue << "R"<<endl;
            break;
        case Actuator::ACTUAL_VELOCITY:
            vel6_count++;
            vel[5]=paramValue;
            // cout<<"v: "<<vel_index<<endl;
            // cout << "Actuator " << (int)uID.actuatorID << " velocity is " << paramValue << "RPM"<<endl;
            break;
        default:
            break;
        }
      break;
    default:
      break;
  }
}

bool LionArmedHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  if (!LeggedHW::init(root_nh, robot_hw_nh))
    return false;

  robot_hw_nh.getParam("power_limit", power_limit_);

  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);

  count = 0;
  //IMU 
  // TODO : IMU
  auto imu_callback = [this](const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_data_.ori[0] = msg->orientation.x;
    imu_data_.ori[1] = msg->orientation.y;
    imu_data_.ori[2] = msg->orientation.z;
    imu_data_.ori[3] = msg->orientation.w;
    imu_data_.angular_vel[0] = msg->angular_velocity.x;
    imu_data_.angular_vel[1] = msg->angular_velocity.y;
    imu_data_.angular_vel[2] = msg->angular_velocity.z;
    imu_data_.linear_acc[0] = msg->linear_acceleration.x;
    imu_data_.linear_acc[1] = msg->linear_acceleration.y;
    imu_data_.linear_acc[2] = msg->linear_acceleration.z;
    };
  sub_ = root_nh.subscribe<sensor_msgs::Imu>("lion_imu",1,imu_callback);
  return true;
}


void LionArmedHW::read(const ros::Time& time, const ros::Duration& period)
{
}

void LionArmedHW::write(const ros::Time& time, const ros::Duration& period)
{
}

bool LionArmedHW::setupJoints()
{
  //Quadruped
  for (const auto& joint : urdf_model_->joints_)
  {
    int leg_index, joint_index;
    if (joint.first.find("RF") != std::string::npos)
      leg_index = UNITREE_LEGGED_SDK::FR_;
    else if (joint.first.find("LF") != std::string::npos)
      leg_index = UNITREE_LEGGED_SDK::FL_;
    else if (joint.first.find("RH") != std::string::npos)
      leg_index = UNITREE_LEGGED_SDK::RR_;
    else if (joint.first.find("LH") != std::string::npos)
      leg_index = UNITREE_LEGGED_SDK::RL_;
    else
      continue;
    if (joint.first.find("HAA") != std::string::npos)
      joint_index = 0;
    else if (joint.first.find("HFE") != std::string::npos)
      joint_index = 1;
    else if (joint.first.find("KFE") != std::string::npos)
      joint_index = 2;
    else
      continue;

    int index = leg_index * 3 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &joint_data_[index].pos_, &joint_data_[index].vel_,
                                                      &joint_data_[index].tau_);
    joint_state_interface_.registerHandle(state_handle);
    hybrid_joint_interface_.registerHandle(HybridJointHandle(state_handle, &joint_data_[index].pos_des_,
                                                             &joint_data_[index].vel_des_, &joint_data_[index].kp_,
                                                             &joint_data_[index].kd_, &joint_data_[index].ff_));
  }
  //ARM 
  for(int i=0;i<6;i++){
    std::string num = std::to_string(i+1);
    std::string name = "joint"+num;
    int index = 12 + i;
    hardware_interface::JointStateHandle state_handle(name, &joint_data_[index].pos_, &joint_data_[index].vel_,
                                                      &joint_data_[index].tau_);
    joint_state_interface_.registerHandle(state_handle);
    hybrid_joint_interface_.registerHandle(HybridJointHandle(state_handle, &joint_data_[index].pos_des_,
                                                              &joint_data_[index].vel_des_, &joint_data_[index].kp_,
                                                              &joint_data_[index].kd_, &joint_data_[index].ff_));
  }
  return true;
}

bool LionArmedHW::setupImu()
{
  imu_sensor_interface_.registerHandle(hardware_interface::ImuSensorHandle(
      "unitree_imu", "unitree_imu", imu_data_.ori, imu_data_.ori_cov, imu_data_.angular_vel, imu_data_.angular_vel_cov,
      imu_data_.linear_acc, imu_data_.linear_acc_cov));
  imu_data_.ori_cov[0] = 0.0012;
  imu_data_.ori_cov[4] = 0.0012;
  imu_data_.ori_cov[8] = 0.0012;

  imu_data_.angular_vel_cov[0] = 0.0004;
  imu_data_.angular_vel_cov[4] = 0.0004;
  imu_data_.angular_vel_cov[8] = 0.0004;

  return true;
}

bool LionArmedHW::setupContactSensor(ros::NodeHandle& nh)
{
  nh.getParam("contact_threshold", contact_threshold_);
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i)
    contact_sensor_interface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contact_state_[i]));
  return true;
}

LionArmedHW::~LionArmedHW(){
  // cout << "MintaSCA:: Disable motors ... "<<endl;
  // for(auto actuator: arm_uID_array_){
  //   pController_->setCurrent(actuator.actuatorID,0.0);
  // }
  
  // this_thread::sleep_for(std::chrono::seconds(3));
  // ActuatorController::getInstance()->disableAllActuators();
  // this_thread::sleep_for(std::chrono::milliseconds(200));

}


}  // namespace legged
