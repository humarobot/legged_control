//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/jacobian.hpp>

#include "legged_controllers/legged_controller.h"

#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_sqp/MultipleShootingMpc.h>

#include <legged_estimation/from_topice_estimate.h>
#include <legged_estimation/linear_kalman_filter.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>


namespace legged
{
bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  // Initialize OCS2
  std::string task_file, urdf_file, reference_file;
  controller_nh.getParam("/task_file", task_file);
  controller_nh.getParam("/urdf_file", urdf_file);
  controller_nh.getParam("/reference_file", reference_file);
  bool verbose;
  loadData::loadCppDataType(task_file, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(task_file, urdf_file, reference_file, verbose);
  setupMpc();
  setupMrt();
  setupArmController();

  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
  PinocchioEndEffectorKinematics ee_kinematics(legged_interface_->getPinocchioInterface(), pinocchio_mapping,
                                               legged_interface_->modelSettings().contactNames3DoF);
  visualizer_ = std::make_shared<LeggedRobotVisualizer>(legged_interface_->getPinocchioInterface(),
                                                        legged_interface_->getCentroidalModelInfo(), ee_kinematics, nh);

  // Hardware interface
  HybridJointInterface* hybrid_joint_interface = robot_hw->get<HybridJointInterface>();
  //Legs handles
  std::vector<std::string> joint_names{ "LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                        "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  for (const auto& joint_name : joint_names)
    hybrid_joint_handles_.push_back(hybrid_joint_interface->getHandle(joint_name));
  //Arm handles
  // std::vector<std::string> arm_joint_names{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  // for (const auto& joint_name : arm_joint_names)
  //   arm_joint_handles_.push_back(hybrid_joint_interface->getHandle(joint_name));

  ContactSensorInterface* contact_interface = robot_hw->get<ContactSensorInterface>();
  std::vector<ContactSensorHandle> contact_handles;
  std::vector<std::string> foot_names{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
  for (auto& name : foot_names)
    contact_handles.push_back(contact_interface->getHandle(name));

  // State estimation
  setupStateEstimate(*legged_interface_, hybrid_joint_handles_, contact_handles,
                     robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu"),&current_observation_);

  // Whole body control
  wbc_ = std::make_shared<Wbc>(task_file, *legged_interface_, ee_kinematics, verbose);

  // Safety Checker
  safety_checker_ = std::make_shared<SafetyChecker>(legged_interface_->getCentroidalModelInfo());


  // ros::NodeHandle nh3;
  // counter_ = 0;
  // auto basePoseCallback = [this](const nav_msgs::OdometryConstPtr& msg) {
  //   counter_++;
  //   // std::cout<<"odom callback"<<std::endl;
  //   double dx = msg->pose.pose.position.x;
  //   double dy = msg->pose.pose.position.y;
  //   double dz = msg->pose.pose.position.z;
  //   Eigen::Vector3d basePos(dx,dy,dz);
  //   // std::cout<<basePos.transpose()<<std::endl;
  //   if(counter_ == 20){
  //     inverseKineWBC(basePos);
  //     counter_ = 0; 
  //   }
                                             
  // };
  // odomSubscriber_ = nh3.subscribe<nav_msgs::Odometry>("odom",1,basePoseCallback);

  return true;
}

void LeggedController::starting(const ros::Time& time)
{
  // Initial state
  current_observation_.mode = ModeNumber::STANCE;
  current_observation_.state =
      rbd_conversions_->computeCentroidalStateFromRbdModel(state_estimate_->update(time, ros::Duration(0.005)));
  current_observation_.input.setZero(legged_interface_->getCentroidalModelInfo().inputDim);

  TargetTrajectories target_trajectories({ current_observation_.time }, { current_observation_.state },
                                         { current_observation_.input });

  // Set the first observation and command and wait for optimization to finish
  mpc_mrt_interface_->setCurrentObservation(current_observation_);
  mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpc_mrt_interface_->initialPolicyReceived() && ros::ok())
  {
    mpc_mrt_interface_->advanceMpc();
    ros::WallRate(legged_interface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpc_running_ = true;
  
  //Arm inverse kinematics
  inverseKine();

  ros::NodeHandle nh2;
  auto armReferenceCallback = [this](const geometry_msgs::PoseStampedConstPtr& msg) {
    // std::cout<<"callback"<<std::endl;
    // std::cout<<"test"<<std::endl;
    inverseKine(msg);
  };
  armRefSubscriber_ = nh2.subscribe<geometry_msgs::PoseStamped>("legged_robot_EE_pose", 1, armReferenceCallback);
  
}

void LeggedController::update(const ros::Time& time, const ros::Duration& period)
{
  // std::cout<<period.toSec()<<std::endl; //周期没问题，根据插件controlPeriod参数的设置相应变化，问题出在WBC求解
  // State Estimate
  current_observation_.time += period.toSec();

  vector_t measured_rbd_state = state_estimate_->update(time, period);
  scalar_t yaw_last = current_observation_.state(9);
  current_observation_.state = rbd_conversions_->computeCentroidalStateFromRbdModel(measured_rbd_state);
  current_observation_.state(9) = yaw_last + angles::shortest_angular_distance(yaw_last, current_observation_.state(9));
  current_observation_.mode = state_estimate_->getMode();

  // Update the current state of the system
  mpc_mrt_interface_->setCurrentObservation(current_observation_);

  // Load the latest MPC policy
  mpc_mrt_interface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimized_state, optimized_input;
  size_t planned_mode;  // The mode that is active at the time the policy is evaluated at.
  mpc_mrt_interface_->evaluatePolicy(current_observation_.time, current_observation_.state, optimized_state,
                                     optimized_input, planned_mode);

  // Whole body control
  current_observation_.input = optimized_input;

  vector_t x = wbc_->update(optimized_state, optimized_input, measured_rbd_state, planned_mode);

  
  vector_t torque = x.tail(12);

  vector_t pos_des = centroidal_model::getJointAngles(optimized_state, legged_interface_->getCentroidalModelInfo());
  vector_t vel_des = centroidal_model::getJointVelocities(optimized_input, legged_interface_->getCentroidalModelInfo());

  // Safety check, if failed, stop the controller
  if (!safety_checker_->check(current_observation_, optimized_state, optimized_input))
  {
    ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
    stopRequest(time);
  }

  
  for (size_t j = 0; j < legged_interface_->getCentroidalModelInfo().actuatedDofNum; ++j){
    hybrid_joint_handles_[j].setCommand(pos_des(j), vel_des(j), 4, 2.5, 0.2*torque(j));

  }
  // std::cout<<std::endl;
    
  // hybrid_joint_handles_[1].setCommand(0., 0., 0, 0, 1.);

  // ARM control
  // impedanceControl();
  
  // arm_joint_handles_[0].setCommand(arm_q_[0],0,500,3,0.0);
  // arm_joint_handles_[1].setCommand(arm_q_[1],0,500,3,0.0);
  // arm_joint_handles_[2].setCommand(arm_q_[2],0,500,3,0.0);
  // arm_joint_handles_[3].setCommand(arm_q_[3],0,120,0,0.0);
  // arm_joint_handles_[4].setCommand(arm_q_[4],0,120,0,0.0);
  // arm_joint_handles_[5].setCommand(arm_q_[5],0,120,0,0.0);
  // std::cout<<arm_q_.transpose()<<std::endl;

  // Visualization
  // visualizer_->update(current_observation_, mpc_mrt_interface_->getPolicy(), mpc_mrt_interface_->getCommand());

  // Publish the observation. Only needed for the command interface
  observation_publisher_.publish(ros_msg_conversions::createObservationMsg(current_observation_));
}

LeggedController::~LeggedController()
{
  controller_running_ = false;
  if (mpc_thread_.joinable())
    mpc_thread_.join();
}

void LeggedController::setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                                            const std::string& reference_file, bool verbose)
{
  legged_interface_ = std::make_shared<LeggedInterface>(task_file, urdf_file, reference_file, verbose);
  legged_interface_->setupOptimalControlProblem(task_file, urdf_file, reference_file, verbose);
}

void LeggedController::setupMpc()
{
  //  mpc_ = std::make_shared<GaussNewtonDDP_MPC>(legged_interface_->mpcSettings(), legged_interface_->ddpSettings(),
  //                                              legged_interface_->getRollout(),
  //                                              legged_interface_->getOptimalControlProblem(),
  //                                              legged_interface_->getInitializer());
  mpc_ = std::make_shared<MultipleShootingMpc>(legged_interface_->mpcSettings(), legged_interface_->sqpSettings(),
                                               legged_interface_->getOptimalControlProblem(),
                                               legged_interface_->getInitializer());
  rbd_conversions_ = std::make_shared<CentroidalModelRbdConversions>(legged_interface_->getPinocchioInterface(),
                                                                     legged_interface_->getCentroidalModelInfo());

  const std::string robot_name = "legged_robot";
  ros::NodeHandle nh;
  // Gait receiver
  auto gait_receiver_ptr = std::make_shared<GaitReceiver>(
      nh, legged_interface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robot_name);
  // ROS ReferenceManager
  auto ros_reference_manager_ptr =
      std::make_shared<RosReferenceManager>(robot_name, legged_interface_->getReferenceManagerPtr());
  ros_reference_manager_ptr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gait_receiver_ptr);
  mpc_->getSolverPtr()->setReferenceManager(ros_reference_manager_ptr);
  observation_publisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robot_name + "_mpc_observation", 1);

  // auto arm_ref_ptr = std::make_shared<ArmReference>(robot_name);
  // arm_ref_ptr->subscribe(nh);
  // this->arm_ref_ = std::move(arm_ref_ptr);
}

void LeggedController::setupMrt()
{
  mpc_mrt_interface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());

  controller_running_ = true;
  mpc_thread_ = std::thread([&]() {
    while (controller_running_)
    {
      try
      {
        executeAndSleep(
            [&]() {
              if (mpc_running_)
                mpc_mrt_interface_->advanceMpc();
            },
            legged_interface_->mpcSettings().mpcDesiredFrequency_);
      }
      catch (const std::exception& e)
      {
        controller_running_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
      }
    }
  });
  setThreadPriority(legged_interface_->sqpSettings().threadPriority, mpc_thread_);
}

void LeggedController::setupStateEstimate(LeggedInterface& legged_interface,
                                          const std::vector<HybridJointHandle>& hybrid_joint_handles,
                                          const std::vector<ContactSensorHandle>& contact_sensor_handles,
                                          const hardware_interface::ImuSensorHandle& imu_sensor_handle,
                                          SystemObservation* current_observation)
{
  state_estimate_ = std::make_shared<KalmanFilterEstimate>(*legged_interface_, hybrid_joint_handles_,
                                                           contact_sensor_handles, imu_sensor_handle,
                                                           current_observation);
  current_observation_.time = 0;
}

void LeggedCheaterController::setupStateEstimate(LeggedInterface& legged_interface,
                                                 const std::vector<HybridJointHandle>& hybrid_joint_handles,
                                                 const std::vector<ContactSensorHandle>& contact_sensor_handles,
                                                 const hardware_interface::ImuSensorHandle& imu_sensor_handle,
                                                 SystemObservation* current_observation)
{
  state_estimate_ = std::make_shared<FromTopicStateEstimate>(*legged_interface_, hybrid_joint_handles_,
                                                             contact_sensor_handles, imu_sensor_handle);
}

void LeggedController::setupArmController(){
  const std::string urdf_filename = "/home/quad/ocs2_ws/src/legged_control/inverse_kinematics_pinocchio/arm.urdf";
  pinocchio::urdf::buildModel(urdf_filename,arm_model_);
  arm_data_ = pinocchio::Data(arm_model_);
  std::cout<<"Arm controller's model name:"<<arm_model_.name<<std::endl;

}

void LeggedController::inverseKine(const geometry_msgs::PoseStampedConstPtr& msg){
  const int JOINT_ID = 6;
  //Get desired pos
  Eigen::Vector3d pos(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
  //Get desired quaternion and trans to rotational matrix
  Eigen::Quaterniond quat;
  quat.x() = msg->pose.orientation.x;
  quat.y() = msg->pose.orientation.y;
  quat.z() = msg->pose.orientation.z;
  quat.w() = msg->pose.orientation.w;
  Eigen::Matrix3d R = quat.normalized().toRotationMatrix();
  const pinocchio::SE3 oMdes(R, pos);

  Eigen::VectorXd q = pinocchio::neutral(arm_model_);
  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-6;

  pinocchio::Data::Matrix6x J(6, arm_model_.nv);
  J.setZero();

  bool success = false;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d err;
  Eigen::VectorXd v(arm_model_.nv);
  for (int i = 0;; i++){
    pinocchio::forwardKinematics(arm_model_, arm_data_, q);
    const pinocchio::SE3 dMi = oMdes.actInv(arm_data_.oMi[JOINT_ID]);
    err = pinocchio::log6(dMi).toVector();
    if (err.norm() < eps)
    {
    success = true;
    break;
    }
    if (i >= IT_MAX)
    {
    success = false;
    break;
    }
    pinocchio::computeJointJacobian(arm_model_, arm_data_, q, JOINT_ID, J);
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(arm_model_, q, v * DT);
    // if (!(i % 10))
    // std::cout << i << ": error = " << err.transpose() << std::endl;
  }
  if (success){
      // std::cout << "Convergence achieved!" << std::endl;
      arm_q_=q;
  }
  else{
      std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
  }
  std::cout << "\nresult: " << q.transpose() << std::endl;
  // std::cout << "\nfinal error: " << err.transpose() << std::endl;
  // std::cout<<arm_q_[0]<<" "<<arm_q_[1]<<" "<<arm_q_[2]<<" "<<arm_q_[3]<<" "<<arm_q_[4]<<" "<<arm_q_[5]<<" "<<std::endl;
}

void LeggedController::inverseKine(){
  const int JOINT_ID = 6;
  const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.2, 0., 0.4));

  Eigen::VectorXd q = pinocchio::neutral(arm_model_);
  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-6;

  pinocchio::Data::Matrix6x J(6, arm_model_.nv);
  J.setZero();

  bool success = false;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d err;
  Eigen::VectorXd v(arm_model_.nv);
  for (int i = 0;; i++){
    pinocchio::forwardKinematics(arm_model_, arm_data_, q);
    const pinocchio::SE3 dMi = oMdes.actInv(arm_data_.oMi[JOINT_ID]);
    err = pinocchio::log6(dMi).toVector();
    if (err.norm() < eps)
    {
    success = true;
    break;
    }
    if (i >= IT_MAX)
    {
    success = false;
    break;
    }
    pinocchio::computeJointJacobian(arm_model_, arm_data_, q, JOINT_ID, J);
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(arm_model_, q, v * DT);
    // if (!(i % 10))
    // std::cout << i << ": error = " << err.transpose() << std::endl;
  }
  if (success){
      std::cout << "Convergence achieved!" << std::endl;
      arm_q_=q;
  }
  else{
      std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
  }
  std::cout << "\nresult: " << q.transpose() << std::endl;
  // std::cout << "\nfinal error: " << err.transpose() << std::endl;
  std::cout<<arm_q_[0]<<" "<<arm_q_[1]<<" "<<arm_q_[2]<<" "<<arm_q_[3]<<" "<<arm_q_[4]<<" "<<arm_q_[5]<<" "<<std::endl;
}

void LeggedController::inverseKineWBC(const Eigen::Vector3d& base_pos){
  const int JOINT_ID = 6;
  Eigen::Vector3d pos_with_base(0.5,0.,0.4);
  const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), pos_with_base - base_pos);

  Eigen::VectorXd q = pinocchio::neutral(arm_model_);
  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-6;

  pinocchio::Data::Matrix6x J(6, arm_model_.nv);
  J.setZero();

  bool success = false;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d err;
  Eigen::VectorXd v(arm_model_.nv);
  for (int i = 0;; i++){
    pinocchio::forwardKinematics(arm_model_, arm_data_, q);
    const pinocchio::SE3 dMi = oMdes.actInv(arm_data_.oMi[JOINT_ID]);
    err = pinocchio::log6(dMi).toVector();
    if (err.norm() < eps)
    {
    success = true;
    break;
    }
    if (i >= IT_MAX)
    {
    success = false;
    break;
    }
    pinocchio::computeJointJacobian(arm_model_, arm_data_, q, JOINT_ID, J);
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(arm_model_, q, v * DT);
    // if (!(i % 10))
    // std::cout << i << ": error = " << err.transpose() << std::endl;
  }
  if (success){
      // std::cout << "Convergence achieved!" << std::endl;
      arm_q_=q;
  }
  else{
      std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
  }
  // std::cout << "\nresult: " << q.transpose() << std::endl;
  // // std::cout << "\nfinal error: " << err.transpose() << std::endl;
  // std::cout<<arm_q_[0]<<" "<<arm_q_[1]<<" "<<arm_q_[2]<<" "<<arm_q_[3]<<" "<<arm_q_[4]<<" "<<arm_q_[5]<<" "<<std::endl;  
}

void LeggedController::updateArmState(){
  // std::cout<<"0"<<std::endl;
  for(int i=0;i<6;i++){
    arm_measured_q_(i,0) = arm_joint_handles_[i].getPosition();
    arm_measured_v_(i,0) = arm_joint_handles_[i].getVelocity();
  }  
  std::cout<<"arm q:"<<arm_measured_q_.transpose()<<std::endl;
  std::cout<<"arm v:"<<arm_measured_v_.transpose()<<std::endl;
  pinocchio::forwardKinematics(arm_model_, arm_data_, arm_measured_q_, arm_measured_v_);
  pinocchio::computeJointJacobians(arm_model_, arm_data_);
  pinocchio::updateFramePlacements(arm_model_, arm_data_);
  pinocchio::crba(arm_model_, arm_data_, arm_measured_q_);
  arm_data_.M.triangularView<Eigen::StrictlyLower>() = arm_data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(arm_model_, arm_data_, arm_measured_q_, arm_measured_v_);
}

void LeggedController::impedanceControl(){
  updateArmState();
  const int JOINT_ID = 6;
  pinocchio::Data::Matrix6x J(6, 6),J_inv(6,6),A_mass(6,6),J_dot(6,6),B(6,6),K(6,6);
  pinocchio::Data::Matrix6x ita(6,1);
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d err,tau,ones;
  ones<<1,1,1,1,1,1;
  J.setZero();
  //计算操作空间动力学
  pinocchio::getJointJacobian(arm_model_,arm_data_,JOINT_ID,pinocchio::ReferenceFrame::LOCAL,J);
  pinocchio::getJointJacobianTimeVariation(arm_model_,arm_data_,JOINT_ID,pinocchio::ReferenceFrame::LOCAL,J_dot);
  J_inv=J.inverse();
  // J_inv = J.pseudoInverse();
  // std::cout<<"1"<<std::endl;
  // J_inv = J.ldlt().solve(ones);
  // std::cout<<"2"<<std::endl;
  A_mass = J_inv.transpose()*arm_data_.M*J_inv;
  ita = J_inv.transpose()*arm_data_.nle-A_mass*J_dot*arm_measured_v_;
  std::cout<<"J inverse transpose:"<<std::endl;
  std::cout<<J_inv.transpose()<<std::endl;
  std::cout<<"mass matrix M:"<<std::endl;
  std::cout<<arm_data_.M<<std::endl;
  std::cout<<"J derivative:"<<std::endl;
  std::cout<<J_dot<<std::endl;
  std::cout<<"nonlinear term:"<<std::endl;
  std::cout<<arm_data_.nle.transpose()<<std::endl;
  // std::cout<<"3"<<std::endl;
  //阻抗控制
  B = Eigen::Matrix<double,6,6>::Identity()*5.0;
  B(3,3) = 0.0;
  B(4,4) = 0.0;
  B(5,5) = 0.0;
  K = Eigen::Matrix<double,6,6>::Identity()*5.0;
  K(0,0) = 50.0;
  K(1,1) = 50.0;
  K(2,2) = 50.0;
  // K(3,3) = 0.0;
  // K(4,4) = 0.0;
  // K(5,5) = 0.0;
  // std::cout<<"4"<<std::endl;
  const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 0.4));
  // std::cout<<"5"<<std::endl;
  const pinocchio::SE3 iMd = arm_data_.oMi[JOINT_ID].actInv(oMdes);
  // std::cout<<"6"<<std::endl;
  err = pinocchio::log6(iMd).toVector();
  // std::cout<<"7"<<std::endl;
  // tau = J.transpose()*(ita-B*J*arm_measured_v_-K*err);
  // tau = J.transpose()*(-B*J*arm_measured_v_-K*err);
  tau = J.transpose()*(K*err-B*J*arm_measured_v_)+arm_data_.nle;


  arm_joint_handles_[0].setCommand(0,0,0,0,tau[0]);
  arm_joint_handles_[1].setCommand(0,0,0,0,tau[1]);
  arm_joint_handles_[2].setCommand(0,0,0,0,tau[2]);
  arm_joint_handles_[3].setCommand(0,0,0,0,tau[3]);
  arm_joint_handles_[4].setCommand(0,0,0,0,tau[4]);
  arm_joint_handles_[5].setCommand(0,0,0,0,tau[5]);

}


}  // namespace legged


PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)
