
#include "lion_armed_hw_interface.h"


namespace legged
{

  int count,cur1_count,cur2_count,cur3_count,cur4_count,cur5_count,cur6_count;
  int vel1_count,vel2_count,vel3_count,vel4_count,vel5_count,vel6_count;
  int pos1_count,pos2_count,pos3_count,pos4_count,pos5_count,pos6_count;
  double cur[6],vel[6],pos[6];
  
void LionArmedHW::processSignal(int sign)
{

    // ActuatorController::getInstance()->disableAllActuators();
    // this_thread::sleep_for(std::chrono::milliseconds(200));
    bExit = true;
}

bool LionArmedHW::bExit = false;

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

  //Arm motor enable
  enableArmMotors();
  count = 0;
//   udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
//   udp_->InitCmdData(low_cmd_);

  // std::string robot_type;
  // root_nh.getParam("robot_type", robot_type);
  // if (robot_type == "a1")
  //   safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::A1);
  // else if (robot_type == "aliengo")
  //   safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Aliengo);
  // else
  // {
  //   ROS_FATAL("Unknown robot type: %s", robot_type.c_str());
  //   return false;
  // }
  return true;
}

void LionArmedHW::enableArmMotors(){
  // mode_= Actuator::Mode_Cur; 
  mode_ = Actuator::Mode_Profile_Pos;
  //Associate program interrupt signals and call processSignal when you end the program with ctrl-c
  signal(SIGINT,processSignal);
  //Initialize the controller
  ActuatorController * pController_ = ActuatorController::initController();
  //ec Define an error type, ec==0x00 means no error, ec will be passed to pcontroller-> lookupActuators(ec) by reference,
  //when the error occurs, ec value will be modified by SDK to the corresponding error code
  Actuator::ErrorsDefine ec;
  //Find the connected actuators.
  arm_uID_array_ = pController_->lookupActuators(ec);
  //If the size of the idArray is greater than zero, the connected actuators have been found
  if(arm_uID_array_.size() > 0)
  {
    std::cout<<"MintaSCA:: Number of motors: "<<arm_uID_array_.size()<<std::endl;
    cout << "MintaSCA:: Enable motors ... " <<endl;
    pController_->enableAllActuators();
    pController_->addParaRequestCallback(paramFeedback);
    //设置电机模式
    for(auto actuator:arm_uID_array_)
      pController_->activateActuatorMode(actuator.actuatorID,mode_);
    
  }
  else
  {
      //ec=0x803 Communication with ECB(ECU) failed
      //ec=0x802 Communication with actuator failed
      cout << "MintaSCA:: Connected error code:" << hex << ec << endl;
  }

}

void LionArmedHW::read(const ros::Time& time, const ros::Duration& period)
{
  if(!bExit){
    //Event polling, polling callback events, event triggering calls to the corresponding callback function
    ActuatorController::processEvents();
    //Asynchronous request executor current, velocity, poistion, and when the request returns,
    //the callback function is triggered by a polling callback event. This function does not block.
    for(auto actuator: arm_uID_array_){
        pController_->requestCVPValue(actuator.actuatorID);
        // pController->setCurrent(actuator.actuatorID,1.0);
    }
    //安全起见设置电机默认指令
    std::vector<std::string> names = hybrid_joint_interface_.getNames();
    for (const auto& name : names)
    {
      HybridJointHandle handle = hybrid_joint_interface_.getHandle(name);
      handle.setFeedforward(0.);
      handle.setVelocityDesired(0.);
      handle.setKd(0.);
      handle.setKp(0.);
    }
    //设置默认关节角度为当前角度
    if(mode_==Actuator::Mode_Profile_Pos){
      for(int i=0;i<6;i++){
        joint_data_[12+i].pos_des_=joint_data_[12+i].pos_;
      }  
    }

    count++;
    if(count==500){ //打印数据完整性
      cout<<"Motor1 current: "<<(float)cur1_count/500.0<<" vel: "<<(float)vel1_count/500.0<<" pos: "<<(float)pos1_count/500.0<<endl;
      cout<<"Motor2 current: "<<(float)cur2_count/500.0<<" vel: "<<(float)vel2_count/500.0<<" pos: "<<(float)pos2_count/500.0<<endl;
      cout<<"Motor3 current: "<<(float)cur3_count/500.0<<" vel: "<<(float)vel3_count/500.0<<" pos: "<<(float)pos3_count/500.0<<endl;
      cout<<"Motor4 current: "<<(float)cur4_count/500.0<<" vel: "<<(float)vel4_count/500.0<<" pos: "<<(float)pos4_count/500.0<<endl;
      cout<<"Motor5 current: "<<(float)cur5_count/500.0<<" vel: "<<(float)vel5_count/500.0<<" pos: "<<(float)pos5_count/500.0<<endl;
      cout<<"Motor6 current: "<<(float)cur6_count/500.0<<" vel: "<<(float)vel6_count/500.0<<" pos: "<<(float)pos6_count/500.0<<endl;
      count=0;
      cur1_count=cur2_count=cur3_count=cur4_count=cur5_count=cur6_count=0;
      vel1_count=vel2_count=vel3_count=vel4_count=vel5_count=vel6_count=0;
      pos1_count=pos2_count=pos3_count=pos4_count=pos5_count=pos6_count=0;
    }
  }
  else{ //退出程序时失能电机
    cout << "MintaSCA:: Disable motors ... "<<endl;
    for(auto actuator: arm_uID_array_){
      pController_->setCurrent(actuator.actuatorID,0.0);
    }
    
    this_thread::sleep_for(std::chrono::seconds(3));
    ActuatorController::getInstance()->disableAllActuators();
    this_thread::sleep_for(std::chrono::milliseconds(200));
    bExit=false;
  }
  //转存关节信息
  //读取信息时，直接把电机数据转成关节数据,转换成弧度
  for(int i=0;i<6;i++){
    joint_data_[i+12].pos_ = pos[i]/18.0*3.1415927;
    joint_data_[i+12].vel_ = vel[i]/18.0*3.1415927;
    joint_data_[i+12].tau_ = cur[i];
  }
  joint_data_[14].pos_ = (pos[1]+pos[2])/18.0*3.1415927;
  joint_data_[14].vel_ = (vel[1]+vel[2])/18.0*3.1415927;
  // if(count==0)
  //   cout<<joint_data_[17].pos_<<" "<<joint_data_[17].vel_<<" "<<joint_data_[17].tau_<<endl;
  // for (int i = 0; i < 12; ++i)
  // {
  //   joint_data_[i].pos_ = low_state_.motorState[i].q;
  //   joint_data_[i].vel_ = low_state_.motorState[i].dq;
  //   joint_data_[i].tau_ = low_state_.motorState[i].tauEst;
  // }
  // std::cout<<"read"<<std::endl;

  // imu_data_.ori[0] = low_state_.imu.quaternion[1];
  // imu_data_.ori[1] = low_state_.imu.quaternion[2];
  // imu_data_.ori[2] = low_state_.imu.quaternion[3];
  // imu_data_.ori[3] = low_state_.imu.quaternion[0];
  // imu_data_.angular_vel[0] = low_state_.imu.gyroscope[0];
  // imu_data_.angular_vel[1] = low_state_.imu.gyroscope[1];
  // imu_data_.angular_vel[2] = low_state_.imu.gyroscope[2];
  // imu_data_.linear_acc[0] = low_state_.imu.accelerometer[0];
  // imu_data_.linear_acc[1] = low_state_.imu.accelerometer[1];
  // imu_data_.linear_acc[2] = low_state_.imu.accelerometer[2];

  // for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i)
  //   contact_state_[i] = low_state_.footForce[i] > contact_threshold_;

  // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
  // std::vector<std::string> names = hybrid_joint_interface_.getNames();
  // for (const auto& name : names)
  // {
  //   HybridJointHandle handle = hybrid_joint_interface_.getHandle(name);
  //   handle.setFeedforward(0.);
  //   handle.setVelocityDesired(0.);
  //   handle.setKd(3.);
  // }
  // std::cout<<"read"<<std::endl;
}

void LionArmedHW::write(const ros::Time& time, const ros::Duration& period)
{
  double joint_cur[18];
  //计算机械臂电流
  for(int i=0;i<6;i++){
    joint_cur[i+12] = joint_data_[i+12].ff_+ (joint_data_[i+12].pos_des_ - joint_data_[i+12].pos_)*joint_data_[i+12].kp_
                  +(joint_data_[i+12].vel_des_ - joint_data_[i+12].vel_) * joint_data_[i+12].kd_;
    //设置电流上下限
    if(joint_cur[i+12] > max_cur_){
      joint_cur[i+12] = max_cur_;
    }else if(joint_cur[i+12] < -max_cur_){
      joint_cur[i+12] = -max_cur_;
    }
  }
  //打印信息
  if(count%500==0) {
    cout<<"joint1:"<<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[12].pos_des_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[12].pos_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[12].tau_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[12].vel_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_cur[12]<<endl;
    cout<<"joint2:"<<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[13].pos_des_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[13].pos_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[13].tau_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[13].vel_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_cur[13]<<endl;
    cout<<"joint3:"<<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[14].pos_des_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[14].pos_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[14].tau_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[14].vel_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_cur[14]<<endl;
    cout<<"joint4:"<<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[15].pos_des_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[15].pos_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[15].tau_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[15].vel_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_cur[15]<<endl;
    cout<<"joint5:"<<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[16].pos_des_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[16].pos_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[16].tau_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[16].vel_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_cur[16]<<endl;
    cout<<"joint6:"<<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[17].pos_des_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[17].pos_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[17].tau_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_data_[17].vel_<<" "
    <<right<<setw(8)<<fixed<<setprecision(3)<<joint_cur[17]<<endl;
  }
  
  //设置关节角度上下限
  auto clip = [](double num, double up, double low){
    if(num > up) num = up;
    else if(num < low) num = low;
  };
  for(int i=0;i<6;i++){
    clip(joint_data_[12+i].pos_des_, joint_up_limit_[i], joint_low_limit_[i]);
  }
  
  //发送机械臂电机控制指令
  //3关节的运动和2 3两个电机都有关系，joint3=motor2+motor3, joint2=motor2，motor3=joint3-joint2
  for(auto actuator: arm_uID_array_){
    if(mode_==Actuator::Mode_Profile_Pos){
      // cout<<joint_data_[11+actuator.actuatorID].pos_des_<<" ";
      if(actuator.actuatorID!=3)
        pController_->setPosition(actuator.actuatorID,joint_data_[11+actuator.actuatorID].pos_des_/3.1415927*18.);
      else 
        pController_->setPosition(actuator.actuatorID,
        (joint_data_[11+actuator.actuatorID].pos_des_-joint_data_[10+actuator.actuatorID].pos_des_)/3.1415927*18.);
    }else if(mode_==Actuator::Mode_Cur){
      pController_->setCurrent(actuator.actuatorID,joint_cur[11+actuator.actuatorID]);
    }
  }

  // for (int i = 0; i < 12; ++i)
  // {
  //   low_cmd_.motorCmd[i].q = joint_data_[i].pos_des_;
  //   low_cmd_.motorCmd[i].dq = joint_data_[i].vel_des_;
  //   low_cmd_.motorCmd[i].Kp = joint_data_[i].kp_;
  //   low_cmd_.motorCmd[i].Kd = joint_data_[i].kd_;
  //   low_cmd_.motorCmd[i].tau = joint_data_[i].ff_;
  // }
  
  // std::cout<<"write"<<std::endl;
  // safety_->PositionLimit(low_cmd_);
  // safety_->PowerProtect(low_cmd_, low_state_, power_limit_);
}

bool LionArmedHW::setupJoints()
{
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
