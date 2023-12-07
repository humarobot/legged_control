#pragma once
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <vector>
#include "ros/ros.h"

class TrajectoryLoader {
 private:
  std::string stateFilePath_{std::string(CMAKE_DIR) + std::string("/result/state.csv")};
  std::string velocityFilePath_{std::string(CMAKE_DIR) + std::string("/result/vel.csv")};
  double timeStep_{0.05};
  std::vector<double> time;
  Eigen::MatrixXd stateTrajectory_;
  Eigen::MatrixXd velTrajectory_;

  int LoadMatrix(std::string fileName,Eigen::MatrixXd& matrix);

 public:
 double totalTime_{3.0};
  TrajectoryLoader() {
    if(LoadMatrix(stateFilePath_,stateTrajectory_)!=0)
      ROS_INFO("\033[1;31mCan't open %s\033[0m", stateFilePath_.c_str());
    if(LoadMatrix(velocityFilePath_,velTrajectory_)!=0)
      ROS_INFO("\033[1;31mCan't open %s\033[0m", velocityFilePath_.c_str());
    // ROS_INFO("stateTrajectory_:");
    // ROS_INFO_STREAM(stateTrajectory_.transpose());
    // ROS_INFO("velTrajectory_:");
    // ROS_INFO_STREAM(velTrajectory_.transpose());
  }
  // Constructor with default values for total time and time step
  TrajectoryLoader(std::string stateFilePath, std::string velocityFilePath, double totalTime = 3.0,
                   double timeStep = 0.05)
      : totalTime_(totalTime), timeStep_(timeStep) {
    if(LoadMatrix(stateFilePath,stateTrajectory_)!=0)
      ROS_INFO("\033[1;31mCan't open %s\033[0m", stateFilePath_.c_str());
    if(LoadMatrix(velocityFilePath,velTrajectory_)!=0)
      ROS_INFO("\033[1;31mCan't open %s\033[0m", velocityFilePath_.c_str());
  }

  void UpdateTrajectory(std::string stateFilePath, std::string velocityFilePath, double totalTime = 3.0,
                        double timeStep = 0.05) {
    totalTime_ = totalTime;
    timeStep_ = timeStep;
    if(LoadMatrix(stateFilePath,stateTrajectory_)!=0)
      ROS_INFO("\033[1;31mCan't open %s\033[0m", stateFilePath_.c_str());
    if(LoadMatrix(velocityFilePath,velTrajectory_)!=0)
      ROS_INFO("\033[1;31mCan't open %s\033[0m", velocityFilePath_.c_str());
  }

  // Getters
  Eigen::MatrixXd GetBaseStateTrajectory() const;
  Eigen::MatrixXd GetBaseVelTrajectory() const;

  Eigen::Matrix<double, 6, 1> GetArmStateAtTime(double time) const;
  Eigen::Matrix<double, 6, 1> GetArmVelAtTime(double time) const;
};
