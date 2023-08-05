#include "legged_hw/ArxMotorDriver.h"

void ArxMotorDriver::Init()
{
  std::cout << "ArxMotors: Can initializing ... " << std::endl;
  InitCan();
  can_size = sizeof(struct can_frame);
  // id [1 2 4 5 6 7]->joint_data_[12 13 14 15 16 17]
  // 定义id和joint_data_数组
  int arx_id[6] = { 1, 2, 4, 5, 6, 7 };
  int joint_index[6] = { 12, 13, 14, 15, 16, 17 };
  for (int i = 0; i < 6; i++)
  {
    id_to_index_[arx_id[i]] = joint_index[i];
  }
  last_time_ = high_resolution_clock::now();
}

void ArxMotorDriver::Update()
{
  const auto current_time = high_resolution_clock::now();
  // Compute desired duration rounded to clock decimation
  const duration<double> desired_duration(1.0 / loop_hz_);
  // Get change in time
  // duration<double> time_span = duration_cast<duration<double>>(current_time - last_time_);
  last_time_ = current_time;

  // Check Data Integrity
  index_++;
  if (index_ == loop_hz_)
  {
    index_ = 0;
    for (int i = 0; i < 7; i++)
      frame_num_[i] = 0;
  }
  StatisticPrinter(index_);

  // TransReceive
  CanSendRecOnce();

  // Sleep
  const auto sleep_till = current_time + duration_cast<high_resolution_clock::duration>(desired_duration);
  std::this_thread::sleep_until(sleep_till);
}

void ArxMotorDriver::InitCan()
{
  struct ifreq ifr2;
  int setflag = 0, ret2 = 0;
  std::string n2 = "can2";
  const char* name2 = n2.c_str();
  if ((_s2 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    perror("Error while opening socket2");
  }
  strcpy(ifr2.ifr_name, name2);
  ioctl(_s2, SIOCGIFINDEX, &ifr2);
  _addr_2.can_family = AF_CAN;
  _addr_2.can_ifindex = ifr2.ifr_ifindex;
  if (bind(_s2, (struct sockaddr*)&_addr_2, sizeof(_addr_2)) < 0)
  {
    perror("Error in socket1 bind");
  }
  setflag = setflag | O_NONBLOCK;
  ret2 = fcntl(_s2, F_SETFL, setflag);
  fcntl(_s2, F_GETFL, 0);
  can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF;
  ret2 = setsockopt(_s2, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
  if (ret2 != 0)
    printf("setsockopt2 fail\n");

  std::cout << "ArxMotors: CAN init done!!!\n" << std::endl;
}

void ArxMotorDriver::CanSendRecOnce()
{
  int nbytes;
  for (u_char i = 1; i < 8; i++)
  {
    if (i != 3)
    {
      MsgToRaw(i, _frame2_s);
      nbytes = write(_s2, &_frame2_s, can_size);
      nbytes = read(_s2, &_frame_buffer, can_size);
      if (nbytes == 16)
        memcpy(&_frame2_r, &_frame_buffer, sizeof(_frame_buffer));

      RawToMsg(_frame2_r.can_id, _frame2_r);
    }
  }
}

void ArxMotorDriver::StatisticPrinter(int index)
{
  if (index == 499)
  {
    // TODO print connectivity
    float conn[7] = { 0 };
    for (int i = 0; i < 7; i++)
    {
      conn[i] = (float)frame_num_[i] / 4.99;
      std::cout << "Arx " << i + 1 << " connectivity is " << conn[i] << "%" << std::endl;
    }
    std::cout << "----------------------------------------------" << std::endl;
  }
}

void ArxMotorDriver::MsgToRaw(int id, struct can_frame& frame)
{
  if (id >= 1 && id <= 7)
  {
    frame.can_dlc = 8;
    legged::LionMotorData joint_data = joint_data_[id_to_index_[id]];
    send_motor_ctrl_cmd(id, joint_data.kp_, joint_data.kd_, joint_data.pos_des_, joint_data.vel_des_, joint_data.ff_,
                        frame.data, &frame.can_id);
  }

}

void ArxMotorDriver::RawToMsg(int motor_id, struct can_frame& frame)
{
  if (motor_id >= 1 && motor_id <= 7)
  {
    RV_can_data_repack(frame.can_id, frame.data, frame.can_dlc, 0);
    joint_data_[id_to_index_[motor_id]].pos_ = rv_motor_msg[motor_id - 1].angle_actual_rad;
    joint_data_[id_to_index_[motor_id]].vel_ = rv_motor_msg[motor_id - 1].speed_actual_rad;
    joint_data_[id_to_index_[motor_id]].tau_ = rv_motor_msg[motor_id - 1].current_actual_float;
    // if(motor_id==6){
    // std::cout << "motor_id: " << rv_motor_msg[motor_id - 1].motor_id << std::endl;
    // std::cout << "motor_angle: " << rv_motor_msg[motor_id - 1].angle_actual_rad << std::endl;
    // std::cout << "motor_speed: " << rv_motor_msg[motor_id - 1].speed_actual_rad << std::endl;
    // std::cout << "motor_torque: " << rv_motor_msg[motor_id - 1].current_actual_float << std::endl;
    // }
    frame_num_[motor_id - 1]++;
  }
}

void ArxMotorDriver::CleanUp()
{
  for (u_char i = 1; i < 8; i++)
  {
    if (i != 3)
    {
      can_frame frame;
      frame.can_dlc = 8;
      send_motor_ctrl_cmd(i, 0, 0, 0, 0, 0, frame.data, &frame.can_id);
      int nbytes = write(_s2, &frame, can_size);
    }
  }
  std::cerr << "ArxMotorDriver: Clean Done!!!!" << std::endl;
}