/**
 *  @file  ISAM2TrajOptimizer.h
 *  @brief incremental planner using iSAM2
 *  @author Jing Dong
 *  @date  Dec 1, 2015
 **/

#pragma once

#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/obstacle/PlanarSDF.h>
#include <gpmp2/obstacle/ObstacleSDFFactorArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPArm.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorArm.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPArm.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/gp/GaussianProcessPriorLinear.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>


namespace gpmp2 {
namespace internal {

/**
 * iSAM2 3D goal planner, give the goal configuration,
 * can re-plan given end goal changes.
 * generic version implementation, use templated types
 *
 * @tparam ROBOT robot body model type
 * @tparam POSE system pose state type
 * @tparam VEL system velocity state type
 * @tparam GP  GP prior factor type
 * @tparam SDF signed distance field type
 * @tparam OBS_FACTOR obstacle cost factor type
 * @tparam OBS_FACTOR_GP GP interpolated obstacle cost factor type
 */
template <class ROBOT, class GP, class SDF, class OBS_FACTOR, class OBS_FACTOR_GP>
class ISAM2TrajOptimizer {

private:

  // typedefs
  typedef typename ROBOT::Pose Pose;
  typedef typename ROBOT::Velocity Velocity;

  // settings
  TrajOptimizerSetting setting_;

  // arm
  const ROBOT& arm_;

  // sdf
  const SDF& sdf_;

  // utils
  gtsam::ISAM2 isam_;

  // internal caches
  gtsam::NonlinearFactorGraph inc_graph_;
  gtsam::Values opt_values_, init_values_;
  std::vector<size_t> removed_factor_index_;    // factors want to be removed in next update
  size_t goal_conf_factor_idx_, goal_vel_factor_idx_;  // index of goal prior factor, use to remove

public:
  /// constructor
  ISAM2TrajOptimizer(const ROBOT& arm, const SDF& sdf,
      const TrajOptimizerSetting& setting);

  ~ISAM2TrajOptimizer() {}


  /// setup start and goal position, build initial factor graph and values
  void initFactorGraph(
      const Pose& start_conf, const Velocity& start_vel,
      const Pose& goal_conf, const Velocity& goal_vel);

  /// initial values of traj
  void initValues(const gtsam::Values& init_values);

  /// update isam by reoptimize
  void update();


  /**
   * Replanning interface
   */

  /// change goal configuration and velocity
  void changeGoalConfigAndVel(const Pose& goal_conf, const Velocity& goal_vel);

  /// fix a given conf and vel (after execution) at current isam2 values
  /// also need current time stamp (state index)
  void fixConfigAndVel(size_t state_idx, const Pose& conf_fix, const Velocity& vel_fix);


  /// accesses
  const gtsam::Values& values() const { return opt_values_; }
};
}   // namespace internal



/// 2D arm specialization
typedef internal::ISAM2TrajOptimizer<ArmModel, GaussianProcessPriorLinear,
    PlanarSDF, ObstaclePlanarSDFFactorArm, ObstaclePlanarSDFFactorGPArm>
ISAM2TrajOptimizer2DArm;


/// 3D arm specialization
typedef internal::ISAM2TrajOptimizer<ArmModel, GaussianProcessPriorLinear,
    SignedDistanceField, ObstacleSDFFactorArm, ObstacleSDFFactorGPArm>
ISAM2TrajOptimizer3DArm;


}   // namespace gpmp2

#include "ISAM2TrajOptimizer-inl.h"
