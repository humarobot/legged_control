//
// Created by qiayuan on 2022/7/24.
//

#include "legged_controllers/target_trajectories_publisher.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include "angles/angles.h"

using namespace legged;

vector_t current_pose_ = vector_t::Zero(6);



scalar_t estimateTimeToTarget(const vector_t& desired_base_displacement)
{
  const scalar_t& dx = desired_base_displacement(0);
  const scalar_t& dy = desired_base_displacement(1);
  const scalar_t& dyaw = desired_base_displacement(3);
  const scalar_t rotation_time = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacement_time = displacement / TARGET_DISPLACEMENT_VELOCITY;
  return std::max(rotation_time, displacement_time);
}

TargetTrajectories targetPoseToTargetTrajectories(const vector_t& target_pose, const SystemObservation& observation,
                                                  const scalar_t& target_reaching_time)
{
  // desired time trajectory
  const scalar_array_t time_trajectory{ observation.time, target_reaching_time };

  // desired state trajectory
  vector_t current_pose = observation.state.segment<6>(6);
  current_pose(2) = COM_HEIGHT;
  current_pose(4) = 0;
  current_pose(5) = 0;
  vector_array_t state_trajectory(2, vector_t::Zero(observation.state.size()));
  state_trajectory[0] << vector_t::Zero(6), current_pose, DEFAULT_JOINT_STATE;
  state_trajectory[1] << vector_t::Zero(6), target_pose, DEFAULT_JOINT_STATE;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t input_trajectory(2, vector_t::Zero(observation.input.size()));

  return { time_trajectory, state_trajectory, input_trajectory };
}

TargetTrajectories goalToTargetTrajectories(const vector_t& goal, const SystemObservation& observation)
{
  const vector_t current_pose = observation.state.segment<6>(6);
  const vector_t target_pose = [&]() {
    vector_t target(6);
    target(0) = goal(0);
    target(1) = goal(1);
    target(2) = goal(2);
    target(3) = current_pose(3) + angles::shortest_angular_distance(current_pose(3), goal(3));
    target(4) = goal(4);
    target(5) = goal(5);
    return target;
  }();
  const scalar_t target_reaching_time = observation.time + estimateTimeToTarget(target_pose - current_pose);
  return targetPoseToTargetTrajectories(target_pose, observation, target_reaching_time);
}

TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmd_vel, const SystemObservation& observation)
{
  // if cmd_vel(0),cmd_vel(1),cmd_vel(3) is not near zero, update current_pose_
  if (std::abs(cmd_vel(0)) > 1e-3 || std::abs(cmd_vel(1)) > 1e-3 || std::abs(cmd_vel(3)) > 1e-3)
  {
    current_pose_ = observation.state.segment<6>(6);
  }
  const Eigen::Matrix<scalar_t, 3, 1> zyx = current_pose_.tail(3);
  vector_t cmd_vel_rot = getRotationMatrixFromZyxEulerAngles(zyx) * cmd_vel.head(3);

  const scalar_t time_to_target = TIME_TO_TARGET;
  const vector_t target_pose = [&]() {
    vector_t target(6);
    target(0) = current_pose_(0) + cmd_vel_rot(0) * time_to_target;
    target(1) = current_pose_(1) + cmd_vel_rot(1) * time_to_target;
    target(2) = COM_HEIGHT;
    target(3) = current_pose_(3) + cmd_vel(3) * time_to_target;
    target(4) = 0;
    target(5) = 0;
    return target;
  }();

  // target reaching duration
  const scalar_t target_reaching_time = observation.time + time_to_target;
  return targetPoseToTargetTrajectories(target_pose, observation, target_reaching_time);
}

TargetTrajectories splineToTargetTrajectories(const vector_t& cmd, const SystemObservation& observation)
{
  // Define a spline
  std::vector<KnotPoint> knots_fw(4);
  knots_fw[0].position << 0, 0, 0.5;
  knots_fw[0].velocity << 0, 0, 0;
  knots_fw[1].position << 1.5, 1, 0.5;
  knots_fw[1].velocity << 0.5, 0, 0;
  knots_fw[2].position << 3.5, -1, 0.5;
  knots_fw[2].velocity << 0.5, 0, 0;
  knots_fw[3].position << 5., 0., 0.5;
  knots_fw[3].velocity << 0, 0, 0;
  double t_max{ 20.0 };
  int num_sample{ 201 };
  HermiteSpline hermite_spline_fw{ knots_fw, t_max };

  // desired time trajectory
  // const scalar_array_t time_trajectory{ latest_observation_.time, target_reaching_time };
  scalar_array_t time_trajectory;
  for (int i = 0; i < num_sample; ++i)
  {
    time_trajectory.push_back(i * t_max / (num_sample-1) + observation.time);
    std::cout <<"time: "<< time_trajectory[i] << std::endl;
  }

  // desired state trajectory

  vector_array_t state_trajectory(num_sample, vector_t::Zero(observation.state.size()));
  for (int i = 0; i < num_sample; ++i)
  {
    Vector6d target_pose;
    target_pose << hermite_spline_fw.getPosition(time_trajectory[i]-observation.time), 0, 0, 0;
    std::cout <<"target_pose: "<< target_pose.transpose() << std::endl;
    state_trajectory[i] << vector_t::Zero(6), target_pose, DEFAULT_JOINT_STATE;
  }

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t input_trajectory(num_sample, vector_t::Zero(observation.input.size()));

  return { time_trajectory, state_trajectory, input_trajectory };
}

int main(int argc, char* argv[])
{
  const std::string robot_name = "legged_robot";

  // Initialize ros node
  ros::init(argc, argv, robot_name + "_target");
  ros::NodeHandle node_handle;
  // Get node parameters
  std::string reference_file, task_file;
  node_handle.getParam("/reference_file", reference_file);
  node_handle.getParam("/task_file", task_file);

  loadData::loadCppDataType(reference_file, "comHeight", COM_HEIGHT);
  loadData::loadEigenMatrix(reference_file, "defaultJointState", DEFAULT_JOINT_STATE);
  loadData::loadCppDataType(reference_file, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(reference_file, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(task_file, "mpc.timeHorizon", TIME_TO_TARGET);

  TargetTrajectoriesPublisher target_pose_command(node_handle, robot_name, &goalToTargetTrajectories,
                                                  &cmdVelToTargetTrajectories, &splineToTargetTrajectories);

  ros::spin();
  // Successful exit
  return 0;
}
