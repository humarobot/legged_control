//
// Created by qiayuan on 2022/7/16.
//

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "legged_interface/legged_interface.h"
#include "ocs2_legged_robot/constraint/ArmZeroVelocityConstraint.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <ocs2_legged_robot/LeggedRobotPreComputation.h>
#include <ocs2_legged_robot/constraint/FrictionConeConstraint.h>
#include <ocs2_legged_robot/constraint/NormalVelocityConstraintCppAd.h>
#include <ocs2_legged_robot/constraint/ZeroForceConstraint.h>
#include <ocs2_legged_robot/constraint/ZeroWrenchConstraint.h>
#include <ocs2_legged_robot/constraint/FixPositionConstraint.h>
#include <ocs2_legged_robot/constraint/ZeroVelocityConstraintCppAd.h>
#include "constraint/ArmEePositionConstraint.h"
#include <ocs2_legged_robot/constraint/BaseConstraint.h>
#include <ocs2_legged_robot/cost/LeggedRobotQuadraticTrackingCost.h>
#include <ocs2_legged_robot/dynamics/LeggedRobotDynamicsAD.h>
#include <ocs2_legged_robot/initialization/LeggedRobotInitializer.h>

namespace legged
{
LeggedInterface::LeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                 const std::string& referenceFile, bool verbose)
{
  // check that task file exists
  boost::filesystem::path task_file_path(taskFile);
  if (boost::filesystem::exists(task_file_path))
    std::cerr << "[LeggedInterface] Loading task file: " << task_file_path << std::endl;
  else
    throw std::invalid_argument("[LeggedInterface] Task file not found: " + task_file_path.string());

  // check that urdf file exists
  boost::filesystem::path urdf_file_path(urdfFile);
  if (boost::filesystem::exists(urdf_file_path))
    std::cerr << "[LeggedInterface] Loading Pinocchio model from: " << urdf_file_path << std::endl;
  else
    throw std::invalid_argument("[LeggedInterface] URDF file not found: " + urdf_file_path.string());

  // check that targetCommand file exists
  boost::filesystem::path reference_file_path(referenceFile);
  if (boost::filesystem::exists(reference_file_path))
    std::cerr << "[LeggedInterface] Loading target command settings from: " << reference_file_path << std::endl;
  else
    throw std::invalid_argument("[LeggedInterface] targetCommand file not found: " + reference_file_path.string());

  // load setting from loading file
  modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
  rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
  sqpSettings_ = multiple_shooting::loadSettings(taskFile, "multiple_shooting", verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                 const std::string& referenceFile, bool verbose)
{
  setupModel(taskFile, urdfFile, referenceFile, verbose);

  // Initial state
  initialState_.setZero(centroidalModelInfo_.stateDim);
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

  // Swing trajectory planner
  std::unique_ptr<SwingTrajectoryPlanner> swing_trajectory_planner(
      new SwingTrajectoryPlanner(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose),
                                 centroidalModelInfo_.numThreeDofContacts));

  // Mode schedule manager
  referenceManagerPtr_ = std::make_shared<LionArmedReferenceManager>(loadGaitSchedule(referenceFile, verbose),
                                                                         std::move(swing_trajectory_planner));

  // Optimal control problem
  problemPtr_.reset(new OptimalControlProblem);

  // Dynamics
  std::unique_ptr<SystemDynamicsBase> dynamics_ptr;
  dynamics_ptr.reset(
      new LeggedRobotDynamicsAD(*pinocchioInterfacePtr_, centroidalModelInfo_, "dynamics", modelSettings_));
  problemPtr_->dynamicsPtr = std::move(dynamics_ptr);

  // Cost terms
  problemPtr_->costPtr->add("baseTrackingCost", getBaseTrackingCost(taskFile, centroidalModelInfo_, verbose));

  // Constraint terms
  // friction cone settings
  scalar_t friction_coefficient = 0.5;
  RelaxedBarrierPenalty::Config barrier_penalty_config;
  std::tie(friction_coefficient, barrier_penalty_config) = loadFrictionConeSettings(taskFile, verbose);

  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
  {
    const std::string& foot_name = modelSettings_.contactNames3DoF[i];

    std::unique_ptr<EndEffectorKinematics<scalar_t>> ee_kinematics_ptr;

    const auto info_cpp_ad = centroidalModelInfo_.toCppAd();
    const CentroidalModelPinocchioMappingCppAd pinocchio_mapping_cpp_ad(info_cpp_ad);
    auto velocity_update_callback = [&info_cpp_ad](const ad_vector_t& state,
                                                   PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
      const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, info_cpp_ad);
      updateCentroidalDynamics(pinocchioInterfaceAd, info_cpp_ad, q);
    };
    ee_kinematics_ptr.reset(new PinocchioEndEffectorKinematicsCppAd(
        *pinocchioInterfacePtr_, pinocchio_mapping_cpp_ad, { foot_name }, centroidalModelInfo_.stateDim,
        centroidalModelInfo_.inputDim, velocity_update_callback, foot_name, modelSettings_.modelFolderCppAd,
        modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));

    problemPtr_->softConstraintPtr->add(foot_name + "_frictionCone",
                                        getFrictionConeConstraint(i, friction_coefficient, barrier_penalty_config));
    problemPtr_->equalityConstraintPtr->add(foot_name + "_zeroForce", getZeroForceConstraint(i));
    problemPtr_->equalityConstraintPtr->add(foot_name + "_zeroVelocity",
                                            getZeroVelocityConstraint(*ee_kinematics_ptr, i));
    problemPtr_->equalityConstraintPtr->add(foot_name + "_normalVelocity",
                                            getNormalVelocityConstraint(*ee_kinematics_ptr, i));
    // problemPtr_->equalityConstraintPtr->add(foot_name + "_footsTrack",
    // getFootsTrackConstraint(*ee_kinematics_ptr, i));
  }
  // Arm constraint term
  {
    int i = 4;
    const std::string& hand_name = "lower_arm_link";
    std::unique_ptr<EndEffectorKinematics<scalar_t>> ee_kinematics_ptr;
    const auto info_cpp_ad = centroidalModelInfo_.toCppAd();
    const CentroidalModelPinocchioMappingCppAd pinocchio_mapping_cpp_ad(info_cpp_ad);
    CentroidalModelPinocchioMapping pinocchio_mapping(centroidalModelInfo_);
    auto velocity_update_callback = [&info_cpp_ad](const ad_vector_t& state,
                                                   PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
      const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, info_cpp_ad);
      updateCentroidalDynamics(pinocchioInterfaceAd, info_cpp_ad, q);
    };
    ee_kinematics_ptr.reset(new PinocchioEndEffectorKinematicsCppAd(
        *pinocchioInterfacePtr_, pinocchio_mapping_cpp_ad, { hand_name }, centroidalModelInfo_.stateDim,
        centroidalModelInfo_.inputDim, velocity_update_callback, hand_name, modelSettings_.modelFolderCppAd,
        modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));
    // problemPtr_->stateSoftConstraintPtr->add(hand_name + "_fixPosition", getFixPositionConstraint());
    problemPtr_->stateSoftConstraintPtr->add(hand_name + "_armEndEffector",
                                             getArmEndEffectorConstraint(*ee_kinematics_ptr, taskFile, "endEffector",
                                                                         verbose));
    problemPtr_->finalSoftConstraintPtr->add(
        "finalEndEffector", getArmEndEffectorConstraint(*ee_kinematics_ptr, taskFile, "finalEndEffector", verbose));
    // problemPtr_->softConstraintPtr->add("armJointLimits",
                                        // getJointLimitSoftConstraint(*pinocchioInterfacePtr_, taskFile, verbose));
    // problemPtr_->softConstraintPtr->add(hand_name + "_zeroVelocity", getArmZeroVelocityConstraint());
    // problemPtr_->equalityConstraintPtr->add(hand_name + "_zeroWrench", getZeroWrenchConstraint(i));
  }

  // Pre-computation
  problemPtr_->preComputationPtr.reset(new LeggedRobotPreComputation(*pinocchioInterfacePtr_, centroidalModelInfo_,
                                                                     *referenceManagerPtr_->getSwingTrajectoryPlanner(),
                                                                     modelSettings_));

  // Rollout
  rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

  // Initialization
  constexpr bool extend_normalized_momentum = true;
  initializerPtr_.reset(
      new LeggedRobotInitializer(centroidalModelInfo_, *referenceManagerPtr_, extend_normalized_momentum));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedInterface::setupModel(const std::string& taskFile, const std::string& urdfFile,
                                 const std::string& referenceFile, bool verbose)
{
  // PinocchioInterface
  pinocchioInterfacePtr_.reset(
      new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames)));

  // CentroidalModelInfo
  centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
      *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
      centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile),
      modelSettings_.contactNames3DoF, modelSettings_.contactNames6DoF);
  // print centroidalModelInfo_
  std::cerr << "centroidalModelInfo_.generalizedCoordinatesNum:" << centroidalModelInfo_.generalizedCoordinatesNum
            << std::endl;
  std::cerr << "centroidalModelInfo_.actuatedDofNum: " << centroidalModelInfo_.actuatedDofNum << std::endl;
  std::cerr << "centroidalModelInfo_.stateDim: " << centroidalModelInfo_.stateDim << std::endl;
  std::cerr << "centroidalModelInfo_.inputDim: " << centroidalModelInfo_.inputDim << std::endl;
  std::cerr << "centroidalModelInfo_.numThreeDofContacts: " << centroidalModelInfo_.numThreeDofContacts << std::endl;
  std::cerr << "centroidalModelInfo_.numSixDofContacts: " << centroidalModelInfo_.numSixDofContacts << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<GaitSchedule> LeggedInterface::loadGaitSchedule(const std::string& file, bool verbose) const
{
  const auto init_mode_schedule = loadModeSchedule(file, "initialModeSchedule", false);
  const auto default_mode_sequence_template = loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);

  const auto default_gait = [default_mode_sequence_template] {
    Gait gait{};
    gait.duration = default_mode_sequence_template.switchingTimes.back();
    // Events: from time -> phase
    std::for_each(default_mode_sequence_template.switchingTimes.begin() + 1,
                  default_mode_sequence_template.switchingTimes.end() - 1,
                  [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = default_mode_sequence_template.modeSequence;
    return gait;
  }();

  // display
  if (verbose)
  {
    std::cerr << "\n#### Modes Schedule: ";
    std::cerr << "\n#### =============================================================================\n";
    std::cerr << "Initial Modes Schedule: \n" << init_mode_schedule;
    std::cerr << "Default Modes Sequence Template: \n" << default_mode_sequence_template;
    std::cerr << "#### =============================================================================\n";
  }

  return std::make_shared<GaitSchedule>(init_mode_schedule, default_mode_sequence_template,
                                        modelSettings_.phaseTransitionStanceTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t LeggedInterface::initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info)
{
  const size_t foot_contact_dim = 3 * info.numThreeDofContacts;
  const size_t total_contact_dim = 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;

  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  const auto q = centroidal_model::getGeneralizedCoordinates(initialState_, centroidalModelInfo_);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  matrix_t base2feet_jac(foot_contact_dim, 12);
  for (size_t i = 0; i < info.numThreeDofContacts; i++)
  {
    matrix_t jac = matrix_t::Zero(6, info.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, model.getBodyId(modelSettings_.contactNames3DoF[i]),
                                pinocchio::LOCAL_WORLD_ALIGNED, jac);
    base2feet_jac.block(3 * i, 0, 3, 12) = jac.block(0, 6, 3, 12);
  }

  matrix_t r_taskspace(info.inputDim, info.inputDim);
  loadData::loadEigenMatrix(taskFile, "R", r_taskspace);
  matrix_t r = r_taskspace;
  // Joint velocities
  r.block(total_contact_dim, total_contact_dim, 12, 12) =
      base2feet_jac.transpose() * r_taskspace.block(total_contact_dim, total_contact_dim, 12, 12) * base2feet_jac;
  return r;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedInterface::getBaseTrackingCost(const std::string& taskFile,
                                                                     const CentroidalModelInfo& info, bool verbose)
{
  matrix_t q(info.stateDim, info.stateDim);
  loadData::loadEigenMatrix(taskFile, "Q", q);
  matrix_t r = initializeInputCostWeight(taskFile, info);

  if (verbose)
  {
    std::cerr << "\n #### Base Tracking Cost Coefficients: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "Q:\n" << q << "\n";
    std::cerr << "R:\n" << r << "\n";
    std::cerr << " #### =============================================================================\n";
  }
  // print size of q
  std::cerr << "q size: " << q.rows() << " " << q.cols() << std::endl;
  std::cerr << "r size: " << r.rows() << " " << r.cols() << std::endl;

  return std::unique_ptr<StateInputCost>(
      new LeggedRobotStateInputQuadraticCost(std::move(q), std::move(r), info, *referenceManagerPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<scalar_t, RelaxedBarrierPenalty::Config>
LeggedInterface::loadFrictionConeSettings(const std::string& taskFile, bool verbose) const
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "frictionConeSoftConstraint.";

  scalar_t friction_coefficient = 1.0;
  RelaxedBarrierPenalty::Config barrier_penalty_config;
  if (verbose)
  {
    std::cerr << "\n #### Friction Cone Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, friction_coefficient, prefix + "frictionCoefficient", verbose);
  loadData::loadPtreeValue(pt, barrier_penalty_config.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrier_penalty_config.delta, prefix + "delta", verbose);
  if (verbose)
  {
    std::cerr << " #### =============================================================================\n";
  }

  return { friction_coefficient, barrier_penalty_config };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost>
LeggedInterface::getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                           const RelaxedBarrierPenalty::Config& barrierPenaltyConfig)
{
  FrictionConeConstraint::Config friction_cone_con_config(frictionCoefficient);
  std::unique_ptr<FrictionConeConstraint> friction_cone_constraint_ptr(new FrictionConeConstraint(
      *referenceManagerPtr_, friction_cone_con_config, contactPointIndex, centroidalModelInfo_));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(
      new StateInputSoftConstraint(std::move(friction_cone_constraint_ptr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint>
LeggedInterface::getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics, size_t contactPointIndex)
{
  auto ee_zero_vel_con_config = [](scalar_t positionErrorGain) {
    EndEffectorLinearConstraint::Config config;
    config.b.setZero(3);
    config.Av.setIdentity(3, 3);
    if (!numerics::almost_eq(positionErrorGain, 0.0))
    {
      config.Ax.setZero(3, 3);
      config.Ax(2, 2) = positionErrorGain;
    }
    return config;
  };
  return std::unique_ptr<StateInputConstraint>(
      new ZeroVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex,
                                      ee_zero_vel_con_config(modelSettings_.positionErrorGain)));
}

std::unique_ptr<StateInputConstraint>
LeggedInterface::getNormalVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                             size_t contactPointIndex)
{
  return std::unique_ptr<StateInputConstraint>(
      new NormalVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex));
}
std::unique_ptr<StateInputConstraint> LeggedInterface::getZeroForceConstraint(size_t contactPointIndex)
{
  return std::unique_ptr<StateInputConstraint>(
      new ZeroForceConstraint(*referenceManagerPtr_, contactPointIndex, centroidalModelInfo_));
}

std::unique_ptr<StateInputConstraint> LeggedInterface::getZeroWrenchConstraint(size_t contactPointIndex)
{
  return std::unique_ptr<StateInputConstraint>(new ZeroWrenchConstraint(contactPointIndex, centroidalModelInfo_));
}

std::unique_ptr<StateCost> LeggedInterface::getFixPositionConstraint()
{
  std::unique_ptr<StateConstraint> constraint;
  constraint.reset(new FixPositionConstraint(*referenceManagerPtr_));
  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 6, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(1000)); });

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

std::unique_ptr<StateCost> LeggedInterface::getBaseConstraint()
{
  std::unique_ptr<StateConstraint> constraint;
  constraint.reset(new BaseConstraint(*referenceManagerPtr_));
  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 6, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(1000)); });

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

std::unique_ptr<StateCost>
LeggedInterface::getArmEndEffectorConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                             const std::string& taskFile, const std::string& prefix, bool verbose)
{
  scalar_t muPosition = 1.0;
  // scalar_t muOrientation = 1.0;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);

  loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", verbose);
  // loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", verbose);

  if (referenceManagerPtr_ == nullptr)
  {
    throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr should be set first!");
  }

  std::unique_ptr<StateConstraint> constraint;
  constraint.reset(new ArmEePositionConstraint(eeKinematics, *referenceManagerPtr_));
  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(3);
  std::generate_n(penaltyArray.begin(), 3,
                  [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muPosition)); });
  // std::generate_n(penaltyArray.begin() + 3, 3,
  //                 [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muOrientation)); });

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

std::unique_ptr<StateInputCost>
LeggedInterface::getJointLimitSoftConstraint(const ocs2::PinocchioInterface& pinocchioInterface,
                                             const std::string& taskFile, bool verbose)
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);

  const auto& model = pinocchioInterface.getModel();

  const int armStateDim = 3;
  const int armInputDim = 3;
  const int armStateIdx = centroidalModelInfo_.stateDim - 3;
  const int armInputIdx = centroidalModelInfo_.inputDim - 3;

  // Load arm position limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
  {
    scalar_t muPositionLimits = 1e-2;
    scalar_t deltaPositionLimits = 1e-3;

    // arm joint DOF limits from the parsed URDF
    const vector_t lowerBoundArm = model.lowerPositionLimit.tail(armStateDim);
    const vector_t upperBoundArm = model.upperPositionLimit.tail(armStateDim);

    loadData::loadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", verbose);
    loadData::loadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", verbose);
    if (verbose)
    {
      std::cerr << "\n #### JointPositionLimits Settings: ";
      std::cerr << "\n #### =============================================================================\n";
      std::cerr << " #### lowerBound: " << lowerBoundArm.transpose() << '\n';
      std::cerr << " #### upperBound: " << upperBoundArm.transpose() << '\n';
      std::cerr << " #### =============================================================================\n";
    }

    stateLimits.reserve(armStateDim);
    for (int i = 0; i < armStateDim; ++i)
    {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = armStateIdx + i;
      boxConstraint.lowerBound = lowerBoundArm(i);
      boxConstraint.upperBound = upperBoundArm(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({ muPositionLimits, deltaPositionLimits }));
      stateLimits.push_back(std::move(boxConstraint));
    }
  }

  // load arm velocity limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
  {
    scalar_t muVelocityLimits = 1e-2;
    scalar_t deltaVelocityLimits = 1e-3;

    loadData::loadPtreeValue(pt, muVelocityLimits, "jointVelocityLimits.mu", verbose);
    loadData::loadPtreeValue(pt, deltaVelocityLimits, "jointVelocityLimits.delta", verbose);

    // arm joint DOFs velocity limits
    vector_t lowerBoundArm = vector_t::Zero(armInputDim);
    vector_t upperBoundArm = vector_t::Zero(armInputDim);
    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound.arm", lowerBoundArm);
    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound.arm", upperBoundArm);

    inputLimits.reserve(armInputDim);
    for (int i = 0; i < armInputDim; ++i)
    {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = armInputIdx + i;
      boxConstraint.lowerBound = lowerBoundArm(i);
      boxConstraint.upperBound = upperBoundArm(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({ muVelocityLimits, deltaVelocityLimits }));
      inputLimits.push_back(std::move(boxConstraint));
    }

    if (verbose)
    {
      std::cerr << "\n #### JointVelocityLimits Settings: ";
      std::cerr << "\n #### =============================================================================\n";
      std::cerr << " #### 'lowerBound':  " << lowerBoundArm.transpose() << std::endl;
      std::cerr << " #### 'upperBound':  " << upperBoundArm.transpose() << std::endl;
      std::cerr << " #### =============================================================================\n";
    }
  }

  auto boxConstraints =
      std::unique_ptr<StateInputSoftBoxConstraint>(new StateInputSoftBoxConstraint(stateLimits, inputLimits));
  boxConstraints->initializeOffset(0.0, vector_t::Zero(centroidalModelInfo_.stateDim),
                                   vector_t::Zero(centroidalModelInfo_.inputDim));
  return boxConstraints;
}

// std::unique_ptr<StateConstraint>
// LeggedInterface::getArmEndEffectorConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics)
// {
//   return std::unique_ptr<StateConstraint>(
//       new ArmEndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
// }

std::unique_ptr<StateInputCost> LeggedInterface::getArmZeroVelocityConstraint()
{
  std::unique_ptr<StateInputConstraint> constraint;
  constraint.reset(new ArmZeroVelocityConstraint());
  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(3);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(100)); });

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

// std::unique_ptr<StateInputConstraint>
// LeggedInterface::getFootsTrackConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
//                                              size_t contactPointIndex)
// {
//     return std::unique_ptr<StateInputConstraint>(
//         new FootsTrackConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex));
// }

}  // namespace legged
