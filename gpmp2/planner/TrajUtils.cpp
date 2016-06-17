/**
 *  @file  TrajUtils-inl.h
 *  @brief utils for trajectory optimization, include initialization and interpolation
 *  @author Jing Dong
 *  @date  May 11, 2015
 **/

#include <gpmp2/planner/TrajUtils.h>
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>

#include <gtsam/inference/Symbol.h>

#include <cmath>
#include <algorithm>

using namespace gtsam;
using namespace std;


namespace gpmp2 {

/* ************************************************************************** */
gtsam::Values initArmTrajStraightLine(const Vector& init_conf,
    const Vector& end_conf, size_t total_step) {

  Values init_values;

  // init pose
  for (size_t i = 0; i <= total_step; i++) {
    Vector conf;
    if (i == 0)
      conf = init_conf;
    else if (i == total_step)
      conf = end_conf;
    else
      conf = static_cast<double>(i) / static_cast<double>(total_step) * end_conf +
          (1.0 - static_cast<double>(i) / static_cast<double>(total_step)) * init_conf;

    init_values.insert(Symbol('x', i), conf);
  }

  // init vel as avg vel
  Vector avg_vel = (end_conf - init_conf) / static_cast<double>(total_step);
  for (size_t i = 0; i <= total_step; i++)
    init_values.insert(Symbol('v', i), avg_vel);

  return init_values;
}

/* ************************************************************************** */
gtsam::Values interpolateArmTraj(const gtsam::Values& opt_values,
    const gtsam::SharedNoiseModel Qc_model, double delta_t, size_t inter_step) {

  // inter setting
  double inter_dt = delta_t / static_cast<double>(inter_step + 1);

  size_t last_pos_idx;
  size_t inter_pos_count = 0;

  // results
  Values results;

  // TODO: gtsam keyvector has issue: free invalid pointer
  KeyVector key_vec = opt_values.keys();

  // sort key list
  std::sort(key_vec.begin(), key_vec.end());

  for (size_t i = 0; i < key_vec.size(); i++) {
    Key key = key_vec[i];

    if (Symbol(key).chr() == 'x') {
      size_t pos_idx = Symbol(key).index();

      if (pos_idx != 0) {
        // skip first pos to interpolate

        for (size_t inter_idx = 1; inter_idx <= inter_step+1; inter_idx++) {

          if (inter_idx == inter_step+1) {
            // last pose
            results.insert(Symbol('x', inter_pos_count), opt_values.at<Vector>(Symbol('x', pos_idx)));
            results.insert(Symbol('v', inter_pos_count), opt_values.at<Vector>(Symbol('v', pos_idx)));

          } else {
            // inter pose
            double tau = static_cast<double>(inter_idx) * inter_dt;
            GaussianProcessInterpolatorLinear gp_inter(Qc_model, delta_t, tau);
            Vector conf1 = opt_values.at<Vector>(Symbol('x', last_pos_idx));
            Vector vel1  = opt_values.at<Vector>(Symbol('v', last_pos_idx));
            Vector conf2 = opt_values.at<Vector>(Symbol('x', pos_idx));
            Vector vel2  = opt_values.at<Vector>(Symbol('v', pos_idx));
            Vector conf  = gp_inter.interpolatePose(conf1, vel1, conf2, vel2);
            Vector vel  = gp_inter.interpolateVelocity(conf1, vel1, conf2, vel2);
            results.insert(Symbol('x', inter_pos_count), conf);
            results.insert(Symbol('v', inter_pos_count), vel);
          }
          inter_pos_count++;
        }

      } else {
        // cache first pose
        results.insert(Symbol('x', 0), opt_values.at<Vector>(Symbol('x', 0)));
        results.insert(Symbol('v', 0), opt_values.at<Vector>(Symbol('v', 0)));
        inter_pos_count++;
      }

      last_pos_idx = pos_idx;
    }
  }

  return results;
}

}

