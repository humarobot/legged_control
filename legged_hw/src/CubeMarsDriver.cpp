#include "legged_hw/CubeMarsDriver.h"

void CubeMarsDriver::Init()
{
	
	// CAN INIT
	InitCan();
	InitSpecialFrames();
	can_size = sizeof(struct can_frame);
	// printf("can size is: %d \n",(int)can_size);

	// Motor Enable
	EnableMotors();
	ZeroCheck();
	
}

void CubeMarsDriver::Run()
{
	while(ros::ok()){
		_index++;
		if(_index==500){
			_index=0;
			for(int i=0;i<12;i++) _frame_num[i]=0;
		} 
		StatisticPrinter(_index);

		CanSendRecOnce();
		ros::spinOnce();
		_loop_rate.sleep();
	}
	//关闭程序时
	CleanUp();
}

void CubeMarsDriver::InitCan()
{
	struct ifreq ifr0, ifr1;
	int setflag = 0, ret0, ret1 = 0;
	std::string n0 = "can0";
	std::string n1 = "can1";
	const char *name0 = n0.c_str();
	const char *name1 = n1.c_str();
	if ((_s0 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		perror("Error while opening socket0");
	}
	if ((_s1 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		perror("Error while opening socket1");
	}
	strcpy(ifr0.ifr_name, name0);
	strcpy(ifr1.ifr_name, name1);
	ioctl(_s0, SIOCGIFINDEX, &ifr0);
	ioctl(_s1, SIOCGIFINDEX, &ifr1);
	_addr_0.can_family = AF_CAN;
	_addr_0.can_ifindex = ifr0.ifr_ifindex;
	_addr_1.can_family = AF_CAN;
	_addr_1.can_ifindex = ifr1.ifr_ifindex;
	if (bind(_s0, (struct sockaddr *)&_addr_0, sizeof(_addr_0)) < 0)
	{
		perror("Error in socket0 bind");
	}
	if (bind(_s1, (struct sockaddr *)&_addr_1, sizeof(_addr_1)) < 0)
	{
		perror("Error in socket1 bind");
	}

	setflag = setflag | O_NONBLOCK;
	ret0 = fcntl(_s0, F_SETFL, setflag);
	ret1 = fcntl(_s1, F_SETFL, setflag);
	fcntl(_s0, F_GETFL, 0);
	fcntl(_s1, F_GETFL, 0);

	can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF;
	ret0 = setsockopt(_s0, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
	ret1 = setsockopt(_s1, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
	if (ret0 != 0)
		printf("setsockopt0 fail\n");
	if (ret1 != 0)
		printf("setsockopt1 fail\n");
	printf("CAN init done!!!\n");
	
}

void CubeMarsDriver::InitSpecialFrames(){
	//使能
	_frame_enable.can_id = 0x00;
	_frame_enable.can_dlc = 8;
	_frame_enable.data[0] = 0xFF;
	_frame_enable.data[1] = 0xFF;
	_frame_enable.data[2] = 0xFF;
	_frame_enable.data[3] = 0xFF;
	_frame_enable.data[4] = 0xFF;
	_frame_enable.data[5] = 0xFF;
	_frame_enable.data[6] = 0xFF;
	_frame_enable.data[7] = 0xFC;
	//失能
	_frame_disable.can_id = 0x00;
	_frame_disable.can_dlc = 8;
	_frame_disable.data[0] = 0xFF;
	_frame_disable.data[1] = 0xFF;
	_frame_disable.data[2] = 0xFF;
	_frame_disable.data[3] = 0xFF;
	_frame_disable.data[4] = 0xFF;
	_frame_disable.data[5] = 0xFF;
	_frame_disable.data[6] = 0xFF;
	_frame_disable.data[7] = 0xFD;
	//设置当前位置为0
	_frame_set0.can_id = 0x00;
	_frame_set0.can_dlc = 8;
	_frame_set0.data[0] = 0xFF;
	_frame_set0.data[1] = 0xFF;
	_frame_set0.data[2] = 0xFF;
	_frame_set0.data[3] = 0xFF;
	_frame_set0.data[4] = 0xFF;
	_frame_set0.data[5] = 0xFF;
	_frame_set0.data[6] = 0xFF;
	_frame_set0.data[7] = 0xFE;
}

/**
 * 给所有的电机发送指令，并且接收电机信息，总耗时大约3ms
*/
void CubeMarsDriver::CanSendRecOnce()
{
	int nbytes;
	// _frame0_s=_frame_enable;
	// _frame1_s=_frame_enable;
	// auto t1=std::chrono::high_resolution_clock::now();
	for(u_char i=1;i<7;i++){
		//两路can，一路6个电机
		//ros数据转成can数据
		MsgToRaw(i,_frame0_s);
		int j=i+6;
		MsgToRaw(j,_frame1_s);
		nbytes = write(_s0, &_frame0_s, can_size);
		nbytes = write(_s1, &_frame1_s, can_size);
		// std::this_thread::sleep_for(std::chrono::microseconds(550)); //200hz不能大于600
		nbytes=read(_s0, &_frame_buffer, can_size);
		if(nbytes==16)
			memcpy(&_frame0_r,&_frame_buffer,sizeof(_frame_buffer));
		nbytes=read(_s1, &_frame_buffer, can_size);
		if(nbytes==16)
			memcpy(&_frame1_r,&_frame_buffer,sizeof(_frame_buffer));
		// std::cout<<"s0 nbytes:"<<nbytes<<std::endl;

		// nbytes=read(_s0, &_frame0_r, can_size);
		{
			uint motor_id_0=_frame0_r.data[0];
				RawToMsg(motor_id_0,_frame0_r);
				_frame_num[motor_id_0-1]++;
			uint motor_id_1=_frame1_r.data[0];
				RawToMsg(motor_id_1,_frame1_r);
				_frame_num[motor_id_1-1]++;
		}
		
		
	}
	
	// auto t2=std::chrono::high_resolution_clock::now();
	// auto duration=std::chrono::duration_cast<std::chrono::microseconds>(t2-t1);
	// std::cout<<duration.count()<<"us \n";
	
}

/**
 * can0 can1相互收发，用于测试通信
 */
void CubeMarsDriver::CanLoopTest()
{
	// int nbytes;
	// size_t can_size = sizeof(struct can_frame);
	// _frame0_s.can_id  = 0x123;
	// _frame0_s.can_dlc = 8;
	// _frame0_s.data[0] = 0x1;
	// _frame0_s.data[1] = 0x2;
	// _frame0_s.data[2] = 0x3;
	// _frame0_s.data[3] = 0x4;
	// _frame0_s.data[4] = 0x5;
	// _frame0_s.data[5] = 0x6;
	// _frame0_s.data[6] = 0x7;
	// _frame0_s.data[7] = 0x8;
	// nbytes = write(_s0, &_frame0_s, can_size);
	// printf("using %s to write\n", "can0");
	// std::cout<<"nbytes:"<<nbytes<<std::endl;
	// ros::Duration(0.001).sleep(); //需要一定的延时，延时过短无法即使收到数据，过长数据自动消失
	// nbytes = read(_s1, _frame1_r, can_size*100);
	// printf("using %s to read\n", "can1");
	// std::cout<<"nbytes:"<<nbytes<<std::endl;
	// if (nbytes > 0)
	// {
	// 	if (_frame1_r[0].can_id & CAN_ERR_FLAG)
	// 		printf("error frame\n");
	// 	else
	// 	{
	// 		printf("\nID=0x%x DLC=%d \n", _frame1_r[0].can_id, _frame1_r[0].can_dlc);
	// 		for (int i = 0; i < _frame1_r[0].can_dlc; i++)
	// 		{
	// 			printf("data[%d]=0x%x \n", i, _frame1_r[0].data[i]);
	// 		}
	// 	}
	// }
}




void CubeMarsDriver::EnableMotors(){
	_frame0_s=_frame_enable;
	_frame1_s=_frame_enable;

	//TODO 需要发送两路
	for(uint i=1;i<7;i++){
		_frame0_s.can_id=i;
		_frame1_s.can_id=i+6;
		nbytes = write(_s0, &_frame0_s, can_size);
		nbytes = write(_s1, &_frame1_s, can_size);
		usleep(500);//延时500us
		while(1){
			nbytes=read(_s0, &_frame_buffer, can_size);
			// std::cout<<"s0 nbytes:"<<nbytes<<std::endl;
			if(nbytes==-1) break;
			memcpy(&_frame0_r,&_frame_buffer,sizeof(_frame_buffer));
			// printf("in loop\n");
		}
		{
			uint motor_id=_frame0_r.data[0];
			if(motor_id==i){
				RawToMsg(motor_id,_frame0_r);
			}
		}
		
		// nbytes=read(_s0, &_frame0_r, can_size);
		// std::cout<<"s0 nbytes:"<<nbytes<<std::endl;
		// nbytes=read(_s1, &_frame1_r, can_size);
		// std::cout<<"s1 nbytes:"<<nbytes<<std::endl;
		// nbytes=read(_s1, &_frame1_r, can_size);
		// std::cout<<"s1 nbytes:"<<nbytes<<std::endl;
		// while(read(_s0, &_frame_buffer, can_size)!=-1){
		// 	memcpy(&_frame0_r,&_frame_buffer,can_size);
		// }
		while(1){
			nbytes=read(_s1, &_frame_buffer, can_size);
			// std::cout<<"s1 nbytes:"<<nbytes<<std::endl;
			if(nbytes==-1) break;
			memcpy(&_frame1_r,&_frame_buffer,sizeof(_frame_buffer));
			// printf("in loop\n");
		}
		{
			uint motor_id=_frame1_r.data[0];
			if(motor_id==i+6){
				RawToMsg(motor_id,_frame1_r);
			}
		}
		
		
	}
}

void CubeMarsDriver::DisableMotors(){
	_frame0_s=_frame_disable;
	_frame1_s=_frame_disable;
	//TODO 需要发送两路
	for(uint i=1;i<7;i++){
		_frame0_s.can_id=i;
		_frame1_s.can_id=i+6;
		nbytes = write(_s0, &_frame0_s, can_size);
		nbytes = write(_s1, &_frame1_s, can_size);
		usleep(500);//延时500us
		
	}
}

void CubeMarsDriver::CleanMotorsBuffer(){
	// for(int i=0;i<12;i++){
	// 	_motors_control.position[i]=0;
	// 	_motors_control.velocity[i]=0;
	// 	_motors_control.kp[i]=0;
	// 	_motors_control.kd[i]=0;
	// 	_motors_control.current[i]=0;
	// }
	
	CanSendRecOnce();
}

void CubeMarsDriver::CleanUp(){
	std::cout<<"Entering Cleann Up"<<std::endl;
	CleanMotorsBuffer();
	DisableMotors();
	std::cout<<"Clean Done!!!!"<<std::endl;
}

void CubeMarsDriver::RawToMsg(int motor_id,struct can_frame& frame){
	motor_id-=1;
	uint p_int=(frame.data[1]<<8)|frame.data[2];
	uint v_int=(frame.data[3]<<4)|(frame.data[4]>>4);
	uint i_int=(frame.data[4]&0x0F)<<8|frame.data[5];
	// int temp_int=frame.data[6];
	joint_data_[motor_id].pos_ = CubeMarsDriver::uint_to_float(p_int,P_MIN,P_MAX,16)*_motors_direction[motor_id]-_motors_offset[motor_id];
	joint_data_[motor_id].vel_ = CubeMarsDriver::uint_to_float(v_int,V_MIN,V_MAX,12)*_motors_direction[motor_id];
	joint_data_[motor_id].tau_ = CubeMarsDriver::uint_to_float(i_int,I_MIN,I_MAX,12)*_motors_direction[motor_id];
	// _motors_return.temp[motor_id]= temp_int-40;
	// _motors_return.fault[motor_id]=frame.data[7];

	// std::cout<<"ID: "<<motor_id+1;
	// std::cout<<" pos:"<<_motors_return.position[motor_id]<<" vel:"<<_motors_return.velocity[motor_id];
	// std::cout<<" I:"<<_motors_return.current[motor_id]<<" temp:"<<(int)_motors_return.temp[motor_id];
	// std::cout<<" fault:"<<(int)_motors_return.fault[motor_id]<<std::endl;
}

void CubeMarsDriver::MsgToRaw(int i,struct can_frame& frame){
	uint p_int=float_to_uint((joint_data_[i-1].pos_des_ + _motors_offset[i-1])*_motors_direction[i-1],P_MIN,P_MAX,16);
	uint v_int=float_to_uint(joint_data_[i-1].pos_des_ * _motors_direction[i-1],V_MIN,V_MAX,12);
	// std::cout<<_motors_control.velocity[8]<<_motors_control.kd[8]<<std::endl;
	uint kp_int=float_to_uint(joint_data_[i-1].kp_,Kp_MIN,Kp_MAX,12);
	uint kd_int=float_to_uint(joint_data_[i-1].kd_,Kd_MIN,Kd_MAX,12);
	uint i_int=float_to_uint(joint_data_[i-1].ff_*_motors_direction[i-1],I_MIN,I_MAX,12);
	frame.can_id=i;
	frame.data[0] = p_int>>8;//位置高 8
	frame.data[1] = p_int&0xFF;//位置低 8
	frame.data[2] = v_int>>4;//速度高 8 位
	frame.data[3] = ((v_int&0xF)<<4)|(kp_int>>8);//速度低 4 位 KP 高 4 位
	frame.data[4] = kp_int&0xFF; //KP 低 8 位
	frame.data[5] = kd_int>>4;	//Kd 高 8 位
	frame.data[6] = ((kd_int&0xF)<<4)|(i_int>>8);//KP 低 4 位扭矩高 4 位
	frame.data[7] = i_int&0xff;//扭矩低 8 位
}

void CubeMarsDriver::StatisticPrinter(int index){
	if(index==499){
		//TODO print connectivity
		float conn[12]={0};
		for(int i=0;i<12;i++){
			conn[i]=(float)_frame_num[i]/4.99;
			std::cout<<"Motor "<<i+1<<" connectivity is "<<conn[i]<<"%"<<std::endl;
		}
		std::cout<<"----------------------------------------------"<<std::endl;

	}
}

void CubeMarsDriver::ZeroCheck(){
	for(int i=0;i<12;i++){
		std::cout<<joint_data_[i].pos_<<"  ";
		double delta = joint_data_[i].pos_-_init_joints_pos[i];
		if(delta<-0.5) _motors_offset[i]-=0.698131;
		else if(delta>0.5) _motors_offset[i]+=0.698131;
	}
	std::cout<<std::endl;


}