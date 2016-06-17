/**
 *  @file  BatchTrajOptimizer.cpp
 *  @brief batch trajectory optimizer
 *  @author Jing Dong
 *  @date  May 10, 2015
 **/

#include <gpmp2/planner/BatchTrajOptimizer.h>

#include <gpmp2/obstacle/ObstaclePlanarSDFFactorArm.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPArm.h>
#include <gpmp2/gp/GaussianProcessPriorLinear.h>


using namespace gtsam;

namespace gpmp2 {

/* ************************************************************************** */
gtsam::Values BatchTrajOptimize3DArm(
    const ArmModel& arm, const SignedDistanceField& sdf,
    const gtsam::Vector& start_conf, const gtsam::Vector& start_vel,
    const gtsam::Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting) {

  return internal::BatchTrajOptimize<ArmModel, GaussianProcessPriorLinear,
      SignedDistanceField, ObstacleSDFFactorArm, ObstacleSDFFactorGPArm>(
          arm, sdf, start_conf, start_vel, end_conf, end_vel, init_values, setting);
}

/* ************************************************************************** */
gtsam::Values BatchTrajOptimize2DArm(
    const ArmModel& arm, const PlanarSDF& sdf,
    const gtsam::Vector& start_conf, const gtsam::Vector& start_vel,
    const gtsam::Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting) {

  return internal::BatchTrajOptimize<ArmModel, GaussianProcessPriorLinear,
      PlanarSDF, ObstaclePlanarSDFFactorArm, ObstaclePlanarSDFFactorGPArm>(
          arm, sdf, start_conf, start_vel, end_conf, end_vel, init_values, setting);
}

}

