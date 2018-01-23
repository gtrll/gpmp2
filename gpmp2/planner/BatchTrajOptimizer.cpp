/**
 *  @file  BatchTrajOptimizer.cpp
 *  @brief batch trajectory optimizer
 *  @author Jing Dong, Mustafa Mukadam
 *  @date  May 10, 2015
 **/

#include <gpmp2/planner/BatchTrajOptimizer.h>
#include <gpmp2/kinematics/JointLimitFactorVector.h>
#include <gpmp2/kinematics/JointLimitFactorPose2Vector.h>
#include <gpmp2/kinematics/VelocityLimitFactorVector.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorArm.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPArm.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileBase.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileBase.h>
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileBase.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileBase.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileArm.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorPose2Mobile2Arms.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2Mobile2Arms.h>
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileVetLinArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileVetLinArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileVetLin2Arms.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileVetLin2Arms.h>
#include <gpmp2/gp/GaussianProcessPriorLinear.h>
#include <gpmp2/gp/GaussianProcessPriorPose2Vector.h>

using namespace std;
using namespace gtsam;


namespace gpmp2 {

/* ************************************************************************** */
gtsam::Values BatchTrajOptimize2DArm(
    const ArmModel& arm, const PlanarSDF& sdf,
    const gtsam::Vector& start_conf, const gtsam::Vector& start_vel,
    const gtsam::Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting) {

  return internal::BatchTrajOptimize<ArmModel, GaussianProcessPriorLinear,
      PlanarSDF, ObstaclePlanarSDFFactorArm, ObstaclePlanarSDFFactorGPArm, 
      JointLimitFactorVector, VelocityLimitFactorVector>(
          arm, sdf, start_conf, start_vel, end_conf, end_vel, init_values, setting);
}

/* ************************************************************************** */
gtsam::Values BatchTrajOptimize3DArm(
    const ArmModel& arm, const SignedDistanceField& sdf,
    const gtsam::Vector& start_conf, const gtsam::Vector& start_vel,
    const gtsam::Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting) {

  return internal::BatchTrajOptimize<ArmModel, GaussianProcessPriorLinear,
      SignedDistanceField, ObstacleSDFFactorArm, ObstacleSDFFactorGPArm, 
      JointLimitFactorVector, VelocityLimitFactorVector>(
          arm, sdf, start_conf, start_vel, end_conf, end_vel, init_values, setting);
}

/* ************************************************************************** */
gtsam::Values BatchTrajOptimizePose2MobileArm2D(
    const Pose2MobileArmModel& marm, const PlanarSDF& sdf,
    const Pose2Vector& start_conf, const gtsam::Vector& start_vel,
    const Pose2Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting) {

  return internal::BatchTrajOptimize<Pose2MobileArmModel, GaussianProcessPriorPose2Vector,
      PlanarSDF, ObstaclePlanarSDFFactorPose2MobileArm, ObstaclePlanarSDFFactorGPPose2MobileArm, 
      JointLimitFactorPose2Vector, VelocityLimitFactorVector>(
          marm, sdf, start_conf, start_vel, end_conf, end_vel, init_values, setting);
}

/* ************************************************************************** */
gtsam::Values BatchTrajOptimizePose2MobileArm(
    const Pose2MobileArmModel& marm, const SignedDistanceField& sdf,
    const Pose2Vector& start_conf, const gtsam::Vector& start_vel,
    const Pose2Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting) {

  return internal::BatchTrajOptimize<Pose2MobileArmModel, GaussianProcessPriorPose2Vector,
      SignedDistanceField, ObstacleSDFFactorPose2MobileArm, ObstacleSDFFactorGPPose2MobileArm, 
      JointLimitFactorPose2Vector, VelocityLimitFactorVector>(
          marm, sdf, start_conf, start_vel, end_conf, end_vel, init_values, setting);
}

/* ************************************************************************** */
gtsam::Values BatchTrajOptimizePose2Mobile2Arms(
    const Pose2Mobile2ArmsModel& marm, const SignedDistanceField& sdf,
    const Pose2Vector& start_conf, const gtsam::Vector& start_vel,
    const Pose2Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting) {

  return internal::BatchTrajOptimize<Pose2Mobile2ArmsModel, GaussianProcessPriorPose2Vector,
      SignedDistanceField, ObstacleSDFFactorPose2Mobile2Arms, ObstacleSDFFactorGPPose2Mobile2Arms, 
      JointLimitFactorPose2Vector, VelocityLimitFactorVector>(
          marm, sdf, start_conf, start_vel, end_conf, end_vel, init_values, setting);
}

/* ************************************************************************** */
gtsam::Values BatchTrajOptimizePose2MobileVetLinArm(
    const Pose2MobileVetLinArmModel& marm, const SignedDistanceField& sdf,
    const Pose2Vector& start_conf, const gtsam::Vector& start_vel,
    const Pose2Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting) {

  return internal::BatchTrajOptimize<Pose2MobileVetLinArmModel, GaussianProcessPriorPose2Vector,
      SignedDistanceField, ObstacleSDFFactorPose2MobileVetLinArm, ObstacleSDFFactorGPPose2MobileVetLinArm, 
      JointLimitFactorPose2Vector, VelocityLimitFactorVector>(
          marm, sdf, start_conf, start_vel, end_conf, end_vel, init_values, setting);
}

/* ************************************************************************** */
gtsam::Values BatchTrajOptimizePose2MobileVetLin2Arms(
    const Pose2MobileVetLin2ArmsModel& marm, const SignedDistanceField& sdf,
    const Pose2Vector& start_conf, const gtsam::Vector& start_vel,
    const Pose2Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting) {

  return internal::BatchTrajOptimize<Pose2MobileVetLin2ArmsModel, GaussianProcessPriorPose2Vector,
      SignedDistanceField, ObstacleSDFFactorPose2MobileVetLin2Arms, ObstacleSDFFactorGPPose2MobileVetLin2Arms, 
      JointLimitFactorPose2Vector, VelocityLimitFactorVector>(
          marm, sdf, start_conf, start_vel, end_conf, end_vel, init_values, setting);
}

/* ************************************************************************** */
double CollisionCost2DArm(
    const ArmModel& arm, const PlanarSDF& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting) {

  return internal::CollisionCost<ArmModel, PlanarSDF, ObstaclePlanarSDFFactorArm>(
    arm, sdf, result, setting);
}

/* ************************************************************************** */
double CollisionCost3DArm(
    const ArmModel& arm, const SignedDistanceField& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting) {

  return internal::CollisionCost<ArmModel, SignedDistanceField, ObstacleSDFFactorArm>(
    arm, sdf, result, setting);
}

/* ************************************************************************** */
double CollisionCostPose2MobileArm2D(
    const Pose2MobileArmModel& marm, const PlanarSDF& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting) {

  return internal::CollisionCost<Pose2MobileArmModel, PlanarSDF, 
    ObstaclePlanarSDFFactorPose2MobileArm>(marm, sdf, result, setting);
}

/* ************************************************************************** */
double CollisionCostPose2MobileBase2D(
    const Pose2MobileBaseModel& robot, const PlanarSDF& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting) {

  return internal::CollisionCost<Pose2MobileBaseModel, PlanarSDF, 
    ObstaclePlanarSDFFactorPose2MobileBase>(robot, sdf, result, setting);
}

/* ************************************************************************** */
double CollisionCostPose2MobileBase(
    const Pose2MobileBaseModel& robot, const SignedDistanceField& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting) {

  return internal::CollisionCost<Pose2MobileBaseModel, SignedDistanceField, 
    ObstacleSDFFactorPose2MobileBase>(robot, sdf, result, setting);
}

/* ************************************************************************** */
double CollisionCostPose2MobileArm(
    const Pose2MobileArmModel& marm, const SignedDistanceField& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting) {

  return internal::CollisionCost<Pose2MobileArmModel, SignedDistanceField, 
    ObstacleSDFFactorPose2MobileArm>(marm, sdf, result, setting);
}

/* ************************************************************************** */
double CollisionCostPose2Mobile2Arms(
    const Pose2Mobile2ArmsModel& marm, const SignedDistanceField& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting) {

  return internal::CollisionCost<Pose2Mobile2ArmsModel, SignedDistanceField, 
    ObstacleSDFFactorPose2Mobile2Arms>(marm, sdf, result, setting);
}

/* ************************************************************************** */
double CollisionCostPose2MobileVetLinArm(
    const Pose2MobileVetLinArmModel& marm, const SignedDistanceField& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting) {

  return internal::CollisionCost<Pose2MobileVetLinArmModel, SignedDistanceField, 
    ObstacleSDFFactorPose2MobileVetLinArm>(marm, sdf, result, setting);
}

/* ************************************************************************** */
double CollisionCostPose2MobileVetLin2Arms(
    const Pose2MobileVetLin2ArmsModel& marm, const SignedDistanceField& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting) {

  return internal::CollisionCost<Pose2MobileVetLin2ArmsModel, SignedDistanceField, 
    ObstacleSDFFactorPose2MobileVetLin2Arms>(marm, sdf, result, setting);
}

/* ************************************************************************** */
gtsam::Values optimize(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& init_values,
    const TrajOptimizerSetting& setting, bool iter_no_increase) {

  std::shared_ptr<gtsam::NonlinearOptimizer> opt;
  std::shared_ptr<gtsam::NonlinearOptimizerParams> params;

  // init the params/opt and type specific settings
  if (setting.opt_type == TrajOptimizerSetting::Dogleg) {
    params = std::shared_ptr<gtsam::NonlinearOptimizerParams>(new DoglegParams());
    // trust region ranage, 0.2 rad or meter, no whitenning, not sure make sense or not 
    dynamic_cast<DoglegParams*>(params.get())->setDeltaInitial(0.2);
  
  } else if (setting.opt_type == TrajOptimizerSetting::LM) {
    params = std::shared_ptr<gtsam::NonlinearOptimizerParams>(new LevenbergMarquardtParams());
    dynamic_cast<LevenbergMarquardtParams*>(params.get())->setlambdaInitial(100.0);
  
  } else if (setting.opt_type == TrajOptimizerSetting::GaussNewton) {
    params = std::shared_ptr<gtsam::NonlinearOptimizerParams>(new GaussNewtonParams());
  }

  // common settings
  params->setMaxIterations(setting.max_iter);
  params->setRelativeErrorTol(setting.rel_thresh);
  if (setting.opt_verbosity >= TrajOptimizerSetting::Error)
    params->setVerbosity("ERROR");

  // optimizer
  if (setting.opt_type == TrajOptimizerSetting::Dogleg) {
    opt = std::shared_ptr<gtsam::NonlinearOptimizer>(new DoglegOptimizer(
      graph, init_values, *(dynamic_cast<DoglegParams*>(params.get()))));
  } else if (setting.opt_type == TrajOptimizerSetting::LM) {
    opt = std::shared_ptr<gtsam::NonlinearOptimizer>(new LevenbergMarquardtOptimizer(
      graph, init_values, *(dynamic_cast<LevenbergMarquardtParams*>(params.get()))));
  } else if (setting.opt_type == TrajOptimizerSetting::GaussNewton) {
    opt = std::shared_ptr<gtsam::NonlinearOptimizer>(new GaussNewtonOptimizer(
      graph, init_values, *(dynamic_cast<GaussNewtonParams*>(params.get()))));
  }

  double currentError = opt->error();
  
  // check if we're already close enough
  if (currentError <= params->errorTol) {
    if (params->verbosity >= NonlinearOptimizerParams::ERROR)
      cout << "Exiting, as error = " << currentError << " < " << params->errorTol << endl;
    return opt->values();
  }

  // Maybe show output
  if (params->verbosity >= NonlinearOptimizerParams::ERROR)
    cout << "Initial error: " << currentError << endl;

  // Return if we already have too many iterations
  if (opt->iterations() >= params->maxIterations) {
    if (params->verbosity >= NonlinearOptimizerParams::TERMINATION)
      cout << "iterations: " << opt->iterations() << " > " << params->maxIterations << endl;
    return opt->values();
  }

  Values last_values;

  // Iterative loop
  do {
    // iteration
    currentError = opt->error();
    // copy last values in case error increase
    if (iter_no_increase)
      last_values = opt->values();
    opt->iterate();
    // Maybe show output
    if (params->verbosity >= NonlinearOptimizerParams::ERROR)
      cout << "newError: " << opt->error() << endl;

  } while (opt->iterations() < params->maxIterations &&
      !checkConvergence(params->relativeErrorTol, params->absoluteErrorTol, params->errorTol,
          currentError, opt->error(), params->verbosity));

  // Printing if verbose
  if (params->verbosity >= NonlinearOptimizerParams::TERMINATION) {
    cout << "iterations: " << opt->iterations() << " > " << params->maxIterations << endl;
    if (opt->iterations() >= params->maxIterations)
      cout << "Terminating because reached maximum iterations" << endl;
  }

  // check whether values increase
  // if increase use last copied values
  if (opt->error() > currentError) {
    if (iter_no_increase) {
      if (params->verbosity >= NonlinearOptimizerParams::ERROR)
        cout << "Error increase, use last copied values" << endl;
      return last_values;
    } else {
      return opt->values();
    }
  } else {
    return opt->values();
  }
}

} // namespace gpmp2
