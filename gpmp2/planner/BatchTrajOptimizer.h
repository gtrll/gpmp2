/**
 *  @file  BatchTrajOptimizer.h
 *  @brief batch trajectory optimizer
 *  @author Jing Dong
 *  @date  May 10, 2015
 **/

#pragma once

#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/obstacle/PlanarSDF.h>
#include <gpmp2/obstacle/SignedDistanceField.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>


namespace gpmp2 {

/**
 * @brief 3D trajectory optimizer, given start/end conf/vel, use 3D SignedDistanceField as sdf
 * @param arm arm model
 * @param sdf 3d Signed Distance Field
 * @param start_conf start configuration
 * @param start_vel start velocity
 * @param end_conf end configuration
 * @param end_vel end velocity
 * @param init_values initial values, x(0) - x(setting.total_step), v(0) - v(setting.total_step)
 * @param setting trajectory optimization settings
 * @return optimized values, x(0) - x(setting.total_step), v(0) - v(setting.total_step)
 */
gtsam::Values BatchTrajOptimize3DArm(
    const ArmModel& arm, const SignedDistanceField& sdf,
    const gtsam::Vector& start_conf, const gtsam::Vector& start_vel,
    const gtsam::Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting);


/**
 * @brief 2D trajectory optimizer, given start/end conf/vel, use PlanarSDF as sdf
 * @param arm arm model
 * @param sdf 2d Signed Distance Field
 * @param start_conf start configuration
 * @param start_vel start velocity
 * @param end_conf end configuration
 * @param end_vel end velocity
 * @param init_values initial values, x(0) - x(setting.total_step), v(0) - v(setting.total_step)
 * @param setting trajectory optimization settings
 * @return optimized values, x(0) - x(setting.total_step), v(0) - v(setting.total_step)
 */
gtsam::Values BatchTrajOptimize2DArm(
    const ArmModel& arm, const PlanarSDF& sdf,
    const gtsam::Vector& start_conf, const gtsam::Vector& start_vel,
    const gtsam::Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting);


namespace internal {

/**
 * @brief generic version implementation of batch trajectory optimizer, use templated types
 * @tparam ROBOT robot body model type
 * @tparam POSE system pose state type
 * @tparam VEL system velocity state type
 * @tparam GP GP prior factor type
 * @tparam SDF signed distance field type
 * @tparam OBS_FACTOR obstacle cost factor type
 * @tparam OBS_FACTOR_GP GP interpolated obstacle cost factor type
 */
template <class ROBOT, class GP, class SDF, class OBS_FACTOR, class OBS_FACTOR_GP>
gtsam::Values BatchTrajOptimize(
    const ROBOT& arm, const SDF& sdf,
    const typename ROBOT::Pose& start_conf, const typename ROBOT::Velocity& start_vel,
    const typename ROBOT::Pose& end_conf, const typename ROBOT::Velocity& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting);

}   // namespace internal
}   // namespace gpmp2

#include "BatchTrajOptimizer-inl.h"

