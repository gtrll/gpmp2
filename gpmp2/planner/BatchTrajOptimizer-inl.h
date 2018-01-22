/**
 *  @file  BatchTrajOptimizer-inl.h
 *  @brief batch trajectory optimizer
 *  @author Jing Dong, Mustafa Mukadam
 *  @date  May 10, 2015
 **/

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/inference/Symbol.h>


namespace gpmp2 {
namespace internal {

/* ************************************************************************** */
template <class ROBOT, class GP, class SDF, class OBS_FACTOR, class OBS_FACTOR_GP, 
    class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
gtsam::Values BatchTrajOptimize(
    const ROBOT& arm, const SDF& sdf,
    const typename ROBOT::Pose& start_conf, const typename ROBOT::Velocity& start_vel,
    const typename ROBOT::Pose& end_conf, const typename ROBOT::Velocity& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting) {

  using namespace gtsam;

  // GP interpolation setting
  const double delta_t = setting.total_time / static_cast<double>(setting.total_step);
  const double inter_dt = delta_t / static_cast<double>(setting.obs_check_inter + 1);

  // build graph
  NonlinearFactorGraph graph;

  for (size_t i = 0; i <= setting.total_step; i++) {
    Key pose_key = Symbol('x', i);
    Key vel_key = Symbol('v', i);

    // start and end
    if (i == 0) {
      graph.add(PriorFactor<typename ROBOT::Pose>(pose_key, start_conf, setting.conf_prior_model));
      graph.add(PriorFactor<typename ROBOT::Velocity>(vel_key, start_vel, setting.vel_prior_model));

    } else if (i == setting.total_step) {
      graph.add(PriorFactor<typename ROBOT::Pose>(pose_key, end_conf, setting.conf_prior_model));
      graph.add(PriorFactor<typename ROBOT::Velocity>(vel_key, end_vel, setting.vel_prior_model));
    }

    if (setting.flag_pos_limit) {
      // joint position limits
      graph.add(LIMIT_FACTOR_POS(pose_key, setting.pos_limit_model, setting.joint_pos_limits_down, 
          setting.joint_pos_limits_up, setting.pos_limit_thresh));
    }
    if (setting.flag_vel_limit) {
      // velocity limits
      graph.add(LIMIT_FACTOR_VEL(vel_key, setting.vel_limit_model, setting.vel_limits, 
          setting.vel_limit_thresh));
    }

    // non-interpolated cost factor
    graph.add(OBS_FACTOR(pose_key, arm, sdf, setting.cost_sigma, setting.epsilon));

    if (i > 0) {
      Key last_pose_key = Symbol('x', i-1);
      Key last_vel_key = Symbol('v', i-1);

      // interpolated cost factor
      if (setting.obs_check_inter > 0) {
        for (size_t j = 1; j <= setting.obs_check_inter; j++) {
          const double tau = inter_dt * static_cast<double>(j);
          graph.add(OBS_FACTOR_GP(last_pose_key, last_vel_key, pose_key, vel_key, arm, sdf,
              setting.cost_sigma, setting.epsilon, setting.Qc_model, delta_t, tau));
        }
      }

      // GP factor
      graph.add(GP(last_pose_key, last_vel_key, pose_key, vel_key, delta_t,
          setting.Qc_model));
    }
  }

  return optimize(graph, init_values, setting);
}

/* ************************************************************************** */
template <class ROBOT, class SDF, class OBS_FACTOR>
double CollisionCost(
    const ROBOT& robot, const SDF& sdf, const gtsam::Values& result, 
    const TrajOptimizerSetting& setting) {

  using namespace gtsam;

  double coll_cost = 0;
  OBS_FACTOR obs_factor = OBS_FACTOR(Symbol('x', 0), robot, sdf, setting.cost_sigma, 0);
  for (size_t i=0; i<result.size()/2; i++)
    coll_cost += (obs_factor.evaluateError(result.at<typename ROBOT::Pose>(Symbol('x', i)))).sum();
  
  return coll_cost;
}

}   // namespace internal
}   // namespace gpmp2

