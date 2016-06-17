/**
 *  @file  BatchTrajOptimizer-inl.h
 *  @brief batch trajectory optimizer
 *  @author Jing Dong
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
template <class ROBOT, class GP, class SDF, class OBS_FACTOR, class OBS_FACTOR_GP>
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

  // optimize!
  Values results;

  if (setting.opt_type == TrajOptimizerSetting::Dogleg) {
    DoglegParams opt_param;
    opt_param.setMaxIterations(setting.max_iter);
    opt_param.setRelativeErrorTol(setting.rel_thresh);
    //opt_param.setVerbosity("ERROR");
    results = DoglegOptimizer(graph, init_values, opt_param).optimize();

  } else if (setting.opt_type == TrajOptimizerSetting::LM) {
    LevenbergMarquardtParams opt_param;
    opt_param.setMaxIterations(setting.max_iter);
    opt_param.setRelativeErrorTol(setting.rel_thresh);
    //opt_param.setVerbosity("ERROR");
    results = LevenbergMarquardtOptimizer(graph, init_values, opt_param).optimize();

  } else if (setting.opt_type == TrajOptimizerSetting::GaussNewton) {
    GaussNewtonParams opt_param;
    opt_param.setMaxIterations(setting.max_iter);
    opt_param.setRelativeErrorTol(setting.rel_thresh);
    //opt_param.setVerbosity("ERROR");
    results = GaussNewtonOptimizer(graph, init_values, opt_param).optimize();
  }

  return results;
}

}   // namespace internal
}   // namespace gpmp2

