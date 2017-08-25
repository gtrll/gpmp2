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
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileArm.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileArm.h>
#include <gpmp2/gp/GaussianProcessPriorLinear.h>
#include <gpmp2/gp/GaussianProcessPriorPose2Vector.h>


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
double CollisionCostPose2MobileArm(
    const Pose2MobileArmModel& marm, const SignedDistanceField& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting) {

  return internal::CollisionCost<Pose2MobileArmModel, SignedDistanceField, 
    ObstacleSDFFactorPose2MobileArm>(marm, sdf, result, setting);
}

}
