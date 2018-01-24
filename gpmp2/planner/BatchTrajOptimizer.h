/**
 *  @file  BatchTrajOptimizer.h
 *  @brief batch trajectory optimizer
 *  @author Jing Dong, Mustafa Mukadam
 *  @date  May 10, 2015
 **/

#pragma once

#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/kinematics/Pose2MobileBaseModel.h>
#include <gpmp2/kinematics/Pose2MobileArmModel.h>
#include <gpmp2/kinematics/Pose2Mobile2ArmsModel.h>
#include <gpmp2/kinematics/Pose2MobileVetLinArmModel.h>
#include <gpmp2/kinematics/Pose2MobileVetLin2ArmsModel.h>
#include <gpmp2/obstacle/PlanarSDF.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/geometry/Pose2Vector.h>
#include <gpmp2/config.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>


namespace gpmp2 {

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
GPMP2_EXPORT gtsam::Values BatchTrajOptimize2DArm(
    const ArmModel& arm, const PlanarSDF& sdf,
    const gtsam::Vector& start_conf, const gtsam::Vector& start_vel,
    const gtsam::Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting);


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
GPMP2_EXPORT gtsam::Values BatchTrajOptimize3DArm(
    const ArmModel& arm, const SignedDistanceField& sdf,
    const gtsam::Vector& start_conf, const gtsam::Vector& start_vel,
    const gtsam::Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting);


/**
 * @brief 2D trajectory optimizer, given start/end conf/vel, use 2D SignedDistanceField as sdf
 * @param marm mobile arm model
 * @param sdf 2d Signed Distance Field
 * @param start_conf start configuration
 * @param start_vel start velocity
 * @param end_conf end configuration
 * @param end_vel end velocity
 * @param init_values initial values, x(0) - x(setting.total_step), v(0) - v(setting.total_step)
 * @param setting trajectory optimization settings
 * @return optimized values, x(0) - x(setting.total_step), v(0) - v(setting.total_step)
 */
GPMP2_EXPORT gtsam::Values BatchTrajOptimizePose2MobileArm2D(
    const Pose2MobileArmModel& marm, const PlanarSDF& sdf,
    const Pose2Vector& start_conf, const gtsam::Vector& start_vel,
    const Pose2Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting);


/**
 * @brief 3D trajectory optimizer, given start/end conf/vel, use 3D SignedDistanceField as sdf
 * @param marm mobile arm model
 * @param sdf 3d Signed Distance Field
 * @param start_conf start configuration
 * @param start_vel start velocity
 * @param end_conf end configuration
 * @param end_vel end velocity
 * @param init_values initial values, x(0) - x(setting.total_step), v(0) - v(setting.total_step)
 * @param setting trajectory optimization settings
 * @return optimized values, x(0) - x(setting.total_step), v(0) - v(setting.total_step)
 */
GPMP2_EXPORT gtsam::Values BatchTrajOptimizePose2MobileArm(
    const Pose2MobileArmModel& marm, const SignedDistanceField& sdf,
    const Pose2Vector& start_conf, const gtsam::Vector& start_vel,
    const Pose2Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting);

// Pose2Mobile2Arms
GPMP2_EXPORT gtsam::Values BatchTrajOptimizePose2Mobile2Arms(
    const Pose2Mobile2ArmsModel& marm, const SignedDistanceField& sdf,
    const Pose2Vector& start_conf, const gtsam::Vector& start_vel,
    const Pose2Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting);

// Pose2MobileVetLinArm
GPMP2_EXPORT gtsam::Values BatchTrajOptimizePose2MobileVetLinArm(
    const Pose2MobileVetLinArmModel& marm, const SignedDistanceField& sdf,
    const Pose2Vector& start_conf, const gtsam::Vector& start_vel,
    const Pose2Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting);

// Pose2MobileVetLin2Arms
GPMP2_EXPORT gtsam::Values BatchTrajOptimizePose2MobileVetLin2Arms(
    const Pose2MobileVetLin2ArmsModel& marm, const SignedDistanceField& sdf,
    const Pose2Vector& start_conf, const gtsam::Vector& start_vel,
    const Pose2Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting);

/**
 * @brief collision cost for 2D arm trajectory
 * @param arm arm model
 * @param sdf 2d Signed Distance Field
 * @param result trajectory to evaluate
 * @param setting trajectory optimization settings
 * @return collision cost
 */
GPMP2_EXPORT double CollisionCost2DArm(
    const ArmModel& arm, const PlanarSDF& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting);


/**
 * @brief collision cost for 3D arm trajectory
 * @param arm arm model
 * @param sdf 3d Signed Distance Field
 * @param result trajectory to evaluate
 * @param setting trajectory optimization settings
 * @return collision cost
 */
GPMP2_EXPORT double CollisionCost3DArm(
    const ArmModel& arm, const SignedDistanceField& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting);


/**
 * @brief collision cost for 2D mobile arm trajectory
 * @param marm mobile arm model
 * @param sdf 2d Signed Distance Field
 * @param result trajectory to evaluate
 * @param setting trajectory optimization settings
 * @return collision cost
 */
GPMP2_EXPORT double CollisionCostPose2MobileArm2D(
    const Pose2MobileArmModel& marm, const PlanarSDF& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting);

/// 2D mobile base SE(2)
GPMP2_EXPORT double CollisionCostPose2MobileBase2D(
    const Pose2MobileBaseModel& robot, const PlanarSDF& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting);

/// 3D mobile base SE(2)
GPMP2_EXPORT double CollisionCostPose2MobileBase(
    const Pose2MobileBaseModel& robot, const SignedDistanceField& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting);

/**
 * @brief collision cost for 3D mobile arm trajectory
 * @param marm mobile arm model
 * @param sdf 3d Signed Distance Field
 * @param result trajectory to evaluate
 * @param setting trajectory optimization settings
 * @return collision cost
 */
GPMP2_EXPORT double CollisionCostPose2MobileArm(
    const Pose2MobileArmModel& marm, const SignedDistanceField& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting);

// Pose2Mobile2Arms
GPMP2_EXPORT double CollisionCostPose2Mobile2Arms(
    const Pose2Mobile2ArmsModel& marm, const SignedDistanceField& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting);

// Pose2MobileVetLinArm
GPMP2_EXPORT double CollisionCostPose2MobileVetLinArm(
    const Pose2MobileVetLinArmModel& marm, const SignedDistanceField& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting);

// Pose2MobileVetLin2Arms
GPMP2_EXPORT double CollisionCostPose2MobileVetLin2Arms(
    const Pose2MobileVetLin2ArmsModel& marm, const SignedDistanceField& sdf,
    const gtsam::Values& result, const TrajOptimizerSetting& setting);


/// run a single optimizer iteration
/// compare to GTSAM default optimization method, if error increase during iteration,
/// will return the smallest error value, not the increased one
GPMP2_EXPORT gtsam::Values optimize(const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting,
    bool iter_no_increase = true);

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
template <class ROBOT, class GP, class SDF, class OBS_FACTOR, class OBS_FACTOR_GP, 
    class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
gtsam::Values BatchTrajOptimize(
    const ROBOT& arm, const SDF& sdf,
    const typename ROBOT::Pose& start_conf, const typename ROBOT::Velocity& start_vel,
    const typename ROBOT::Pose& end_conf, const typename ROBOT::Velocity& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting);


/**
 * @brief generic version implementation of collision cost for any trajectory
 * @tparam ROBOT robot body model type
 * @tparam SDF signed distance field type
 * @tparam OBS_FACTOR obstacle cost factor type
 */
template <class ROBOT, class SDF, class OBS_FACTOR>
double CollisionCost(
    const ROBOT& robot, const SDF& sdf, const gtsam::Values& result, 
    const TrajOptimizerSetting& setting);

/**
 * @brief collision cost checking for a single state
 * @tparam ROBOT robot body model type
 * @tparam SDF signed distance field type
 * @tparam OBS_FACTOR obstacle cost factor type
 */


}   // namespace internal
}   // namespace gpmp2

#include "BatchTrajOptimizer-inl.h"

