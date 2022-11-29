#ifndef CubeMarsDriver_H
#define CubeMarsDriver_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <fcntl.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include <unistd.h>
#include <chrono>
#include <thread>
#include "hardware_interface.h"


#define P_MIN -12.5
#define P_MAX 12.5
#define	V_MIN -30
#define V_MAX 30
#define I_MIN -18
#define I_MAX 18
#define Kp_MIN 0
#define Kp_MAX 500
#define Kd_MIN 0
#define Kd_MAX 5
#define PI 3.1415927

/** 
 * 调试问题记录：
 * 1 突然掉电重启 原因未知
 * 2 缓慢运动过程中电机抖动 可能是没有前馈，轨迹规划不合理
 */

namespace legged{

using namespace std::chrono;
using clock = high_resolution_clock;

class CubeMarsDriver {
public:
    CubeMarsDriver(legged::LionMotorData* joint_data,double loop_hz):
        loop_hz_(loop_hz),_loop_rate(loop_hz),joint_data_(joint_data){}
    void Init();
    void Run();
    void Update();
    ~CubeMarsDriver(){
        CleanUp();
    }
private:
    
    void InitCan();
    void CanSendRecOnce();
    void SendCan(char* name);
    void ReadCan(); 
    void CanLoopTest();
    void InitSpecialFrames();
    void CleanUp();
    void CleanMotorsBuffer();
    void EnableMotors();
    void DisableMotors();
    void RawToMsg(int,struct can_frame&);
    void MsgToRaw(int,struct can_frame&);
    void ZeroCheck();

    void StatisticPrinter(int);

    float uint_to_float(int x_int, float x_min, float x_max, int bits){
        /// converts unsigned int to float, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }
    int float_to_uint(float x, float x_min, float x_max, unsigned int bits){
        ///Converts a float to an unsigned int, given range and number of bits ///
        float span = x_max - x_min;
        if(x < x_min) x = x_min;
        else if(x > x_max) x = x_max;
        return (int) ((x- x_min)*((float)((1<<bits)/span)));
    }
    double loop_hz_;
    ros::Rate _loop_rate{500};


    double _motors_direction[12]={-1,1,1,-1,-1,-1,1,1,1,1,-1,-1};
    double _motors_offset[12]={0,0,PI,0,0,PI,0,0,PI,0,0,PI};
    double _init_joints_pos[12]={0.4,0.0,-3.14,-0.4,0.0,-3.14,0.4,0.0,-3.14,-0.4,0.0,-3.14};
    int _index=0;
    int _frame_num[12]={0};

    int _s0,_s1;
	int nbytes;
    size_t can_size;
	struct sockaddr_can _addr_0,_addr_1;
	struct can_frame    _frame_buffer,_frame0_r,_frame0_s,_frame1_r,_frame1_s,
                        _frame_enable,_frame_disable,_frame_set0;

    legged::LionMotorData* joint_data_;
    //Timing
    ros::Duration elapsed_time_;
    clock::time_point last_time_;
};

}

#endif
