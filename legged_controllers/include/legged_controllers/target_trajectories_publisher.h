//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include <mutex>
#include <ros/subscriber.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include "HermiteSpline.hpp"
#include "lion_msg/baseTraj.h"

namespace legged
{

using namespace ocs2;
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;
vector_t DEFAULT_JOINT_STATE(12);
scalar_t TIME_TO_TARGET;

class TargetTrajectoriesPublisher final
{
public:
  using CmdToTargetTrajectories =
      std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;

  TargetTrajectoriesPublisher(::ros::NodeHandle& nh, const std::string& topic_prefix,
                              CmdToTargetTrajectories goal_to_target_trajectories,
                              CmdToTargetTrajectories cmd_vel_to_target_trajectories,
                              CmdToTargetTrajectories spline_to_target_trajectories)
    : goal_to_target_trajectories_(std::move(goal_to_target_trajectories))
    , cmd_vel_to_target_trajectories_(std::move(cmd_vel_to_target_trajectories))
    , spline_to_target_trajectories_(std::move(spline_to_target_trajectories))
    , tf2_(buffer_)
  {
    // Trajectories publisher
    target_trajectories_publisher_.reset(new TargetTrajectoriesRosPublisher(nh, topic_prefix));

    // observation subscriber
    auto observation_callback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(latest_observation_mutex_);
      latest_observation_ = ros_msg_conversions::readObservationMsg(*msg);
    };
    observation_sub_ =
        nh.subscribe<ocs2_msgs::mpc_observation>(topic_prefix + "_mpc_observation", 1, observation_callback);

    // goal subscriber
    auto goal_callback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
      if (latest_observation_.time == 0.0)
        return;
      geometry_msgs::PoseStamped pose = *msg;
      try
      {
        buffer_.transform(pose, pose, "odom", ros::Duration(0.2));
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("Failure %s\n", ex.what());
        return;
      }

      vector_t cmd_goal = vector_t::Zero(6);
      cmd_goal[0] = pose.pose.position.x;
      cmd_goal[1] = pose.pose.position.y;
      cmd_goal[2] = pose.pose.position.z;
      Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y,
                                    pose.pose.orientation.z);
      // quaternion to rqy euler angle
      cmd_goal[3] = std::atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
      cmd_goal[4] = std::asin(2 * (q.w() * q.y() - q.z() * q.x()));
      cmd_goal[5] = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

      const auto trajectories = goal_to_target_trajectories_(cmd_goal, latest_observation_);
      target_trajectories_publisher_->publishTargetTrajectories(trajectories);
    };

    // cmd_vel subscriber
    auto cmd_vel_callback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
      if (latest_observation_.time == 0.0)
        return;

      vector_t cmd_vel = vector_t::Zero(4);
      cmd_vel[0] = msg->linear.x;
      cmd_vel[1] = msg->linear.y;
      cmd_vel[2] = msg->linear.z;
      cmd_vel[3] = msg->angular.z;

      const auto trajectories = cmd_vel_to_target_trajectories_(cmd_vel, latest_observation_);
      target_trajectories_publisher_->publishTargetTrajectories(trajectories);
    };

    auto spline_callback = [this](const std_msgs::Int32ConstPtr& msg) {
      if (latest_observation_.time == 0.0)
        return;
      if (msg->data == 1)
      {
        const auto trajectories = spline_to_target_trajectories_(vector_t::Zero(4), latest_observation_);
        target_trajectories_publisher_->publishTargetTrajectories(trajectories);
      }
      else if (msg->data == 2)
      {
        // target_trajectories_publisher_->publishTargetTrajectories(trajectories);
      }
    };

    auto base_traj_callback = [this](const lion_msg::baseTrajConstPtr& msg) {
      double T = msg->totalTime;
      int numKnots = msg->numKnots;
      double dt = T / (numKnots - 1);
      // * desired time trajectory
      scalar_array_t time_trajectory;
      for (int i = 0; i < numKnots; ++i)
      {
        time_trajectory.push_back(i * dt + latest_observation_.time);
        // std::cout << "time: " << time_trajectory[i] << std::endl;
      }
      // * desired state trajectory
      vector_array_t state_trajectory(numKnots, vector_t::Zero(latest_observation_.state.size()));
      for (int i = 0; i < numKnots; ++i)
      {
        Vector6d target_pose;
        Eigen::Quaterniond q;
        q.w() = msg->poses[i].orientation.w;
        q.x() = msg->poses[i].orientation.x;
        q.y() = msg->poses[i].orientation.y;
        q.z() = msg->poses[i].orientation.z;
        target_pose[0] = msg->poses[i].position.x;
        target_pose[1] = msg->poses[i].position.y;
        target_pose[2] = msg->poses[i].position.z;
        target_pose[3] = std::atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
        target_pose[4] = std::asin(2 * (q.w() * q.y() - q.z() * q.x()));
        target_pose[5] = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

        // std::cout << "target_pose: " << target_pose.transpose() << std::endl;
        state_trajectory[i] << vector_t::Zero(6), target_pose, DEFAULT_JOINT_STATE;
      }

      // * desired input trajectory (just right dimensions, they are not used)
      const vector_array_t input_trajectory(numKnots, vector_t::Zero(latest_observation_.input.size()));

      TargetTrajectories trajectories{ time_trajectory, state_trajectory, input_trajectory };
      target_trajectories_publisher_->publishTargetTrajectories(trajectories);
    };

    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goal_callback);
    cmd_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmd_vel_callback);
    spline_sub_ = nh.subscribe<std_msgs::Int32>("/spline", 1, spline_callback);
    base_traj_sub_ = nh.subscribe<lion_msg::baseTraj>("/base_trajectory_topic", 1, base_traj_callback);
  }

private:
  CmdToTargetTrajectories goal_to_target_trajectories_, cmd_vel_to_target_trajectories_, spline_to_target_trajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> target_trajectories_publisher_;

  ::ros::Subscriber observation_sub_, goal_sub_, cmd_vel_sub_, spline_sub_, base_traj_sub_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  mutable std::mutex latest_observation_mutex_;
  SystemObservation latest_observation_;
};

}  // namespace legged
