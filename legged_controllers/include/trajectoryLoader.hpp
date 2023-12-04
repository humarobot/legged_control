#pragma once
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <vector>


class TrajectoryLoader {
 private:
  std::string stateFilePath_{std::string(CMAKE_DIR) + std::string("/result/state.csv")};
  std::string velocityFilePath_{std::string(CMAKE_DIR) + std::string("/result/vel.csv")};
  double timeStep_{0.05};
  std::vector<double> time;
  Eigen::MatrixXd stateTrajectory_;
  Eigen::MatrixXd velTrajectory_;

  Eigen::MatrixXd LoadMatrix(std::string fileName);

 public:
 double totalTime_{3.0};
  TrajectoryLoader() {
    stateTrajectory_ = LoadMatrix(stateFilePath_);
    velTrajectory_ = LoadMatrix(velocityFilePath_);
    // std::cout<<"stateTrajectory_:"<<std::endl;
    // std::cout<<stateTrajectory_.transpose()<<std::endl;
    // std::cout<<"velTrajectory_:"<<std::endl;
    // std::cout<<velTrajectory_.transpose()<<std::endl;
  }
  // Constructor with default values for total time and time step
  TrajectoryLoader(std::string stateFilePath, std::string velocityFilePath, double totalTime = 3.0,
                   double timeStep = 0.05)
      : totalTime_(totalTime), timeStep_(timeStep) {
    stateTrajectory_ = LoadMatrix(stateFilePath);
    velTrajectory_ = LoadMatrix(velocityFilePath);
  }

  // Getters
  Eigen::MatrixXd GetBaseStateTrajectory() const;
  Eigen::MatrixXd GetBaseVelTrajectory() const;

  Eigen::Matrix<double, 6, 1> GetArmStateAtTime(double time) const;
  Eigen::Matrix<double, 6, 1> GetArmVelAtTime(double time) const;
};
