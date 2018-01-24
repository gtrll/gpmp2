/**
 *  @file  TrajUtils-inl.h
 *  @brief utils for trajectory optimization, include initialization and interpolation
 *  @author Jing Dong, Mustafa Mukadam
 *  @date  May 11, 2015
 **/

#include <gpmp2/planner/TrajUtils.h>
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2Vector.h>

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
gtsam::Values initPose2VectorTrajStraightLine(const Pose2& init_pose, const Vector& init_conf,
    const Pose2& end_pose, const Vector& end_conf, size_t total_step) {

  Values init_values;

  Vector avg_vel = (Vector(3+init_conf.size()) << end_pose.x()-init_pose.x(), 
      end_pose.y()-init_pose.y(), end_pose.theta()-init_pose.theta(), 
      end_conf - init_conf).finished() / static_cast<double>(total_step);

  for (size_t i=0; i<=total_step; i++) {
    Vector conf;
    Pose2 pose;
    double ratio = static_cast<double>(i) / static_cast<double>(total_step);
    pose = interpolate<Pose2>(init_pose, end_pose, ratio);
    conf = (1.0 - ratio)*init_conf + ratio*end_conf;
    init_values.insert(Symbol('x', i), Pose2Vector(pose, conf));
    init_values.insert(Symbol('v', i), avg_vel);
  }

  return init_values;
}

/* ************************************************************************** */
gtsam::Values initPose2TrajStraightLine(const Pose2& init_pose, const Pose2& end_pose,
    size_t total_step) {

  Values init_values;

  Vector avg_vel = (Vector(3) << end_pose.x()-init_pose.x(), end_pose.y()-init_pose.y(),
    end_pose.theta()-init_pose.theta()).finished() / static_cast<double>(total_step);

  for (size_t i=0; i<=total_step; i++) {
    Pose2 pose;
    double ratio = static_cast<double>(i) / static_cast<double>(total_step);
    pose = interpolate<Pose2>(init_pose, end_pose, ratio);
    init_values.insert(Symbol('x', i), pose);
    init_values.insert(Symbol('v', i), avg_vel);
  }

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

/* ************************************************************************** */
gtsam::Values interpolateArmTraj(const gtsam::Values& opt_values,
    const gtsam::SharedNoiseModel Qc_model, double delta_t, size_t inter_step, 
    size_t start_index, size_t end_index) {

  Values results;

  double inter_dt = delta_t / static_cast<double>(inter_step + 1);
  size_t result_index = 0;

  for (size_t i = start_index; i < end_index; i++) {

    results.insert(Symbol('x', result_index), opt_values.at<Vector>(Symbol('x', i)));
    results.insert(Symbol('v', result_index), opt_values.at<Vector>(Symbol('v', i)));

    for (size_t inter_idx = 1; inter_idx <= inter_step; inter_idx++) {

      result_index++;
      double tau = static_cast<double>(inter_idx) * inter_dt;
      GaussianProcessInterpolatorLinear gp_inter(Qc_model, delta_t, tau);
      Vector conf1 = opt_values.at<Vector>(Symbol('x', i));
      Vector vel1  = opt_values.at<Vector>(Symbol('v', i));
      Vector conf2 = opt_values.at<Vector>(Symbol('x', i+1));
      Vector vel2  = opt_values.at<Vector>(Symbol('v', i+1));
      Vector conf  = gp_inter.interpolatePose(conf1, vel1, conf2, vel2);
      Vector vel  = gp_inter.interpolateVelocity(conf1, vel1, conf2, vel2);
      results.insert(Symbol('x', result_index), conf);
      results.insert(Symbol('v', result_index), vel);
    }

    result_index++;
  }

  results.insert(Symbol('x', result_index), opt_values.at<Vector>(Symbol('x', end_index)));
  results.insert(Symbol('v', result_index), opt_values.at<Vector>(Symbol('v', end_index)));

  return results;
}

/* ************************************************************************** */
gtsam::Values interpolatePose2MobileArmTraj(const gtsam::Values& opt_values,
    const gtsam::SharedNoiseModel Qc_model, double delta_t, size_t inter_step, 
    size_t start_index, size_t end_index) {

  Values results;

  double inter_dt = delta_t / static_cast<double>(inter_step + 1);
  size_t result_index = 0;

  for (size_t i = start_index; i < end_index; i++) {

    results.insert(Symbol('x', result_index), opt_values.at<Pose2Vector>(Symbol('x', i)));
    results.insert(Symbol('v', result_index), opt_values.at<Vector>(Symbol('v', i)));

    for (size_t inter_idx = 1; inter_idx <= inter_step; inter_idx++) {

      result_index++;
      double tau = static_cast<double>(inter_idx) * inter_dt;
      GaussianProcessInterpolatorPose2Vector gp_inter(Qc_model, delta_t, tau);
      Pose2Vector conf1 = opt_values.at<Pose2Vector>(Symbol('x', i));
      Vector vel1  = opt_values.at<Vector>(Symbol('v', i));
      Pose2Vector conf2 = opt_values.at<Pose2Vector>(Symbol('x', i+1));
      Vector vel2  = opt_values.at<Vector>(Symbol('v', i+1));
      Pose2Vector conf  = gp_inter.interpolatePose(conf1, vel1, conf2, vel2);
      Vector vel  = gp_inter.interpolateVelocity(conf1, vel1, conf2, vel2);
      results.insert(Symbol('x', result_index), conf);
      results.insert(Symbol('v', result_index), vel);
    }

    result_index++;
  }

  results.insert(Symbol('x', result_index), opt_values.at<Pose2Vector>(Symbol('x', end_index)));
  results.insert(Symbol('v', result_index), opt_values.at<Vector>(Symbol('v', end_index)));

  return results;
}

/* ************************************************************************** */
gtsam::Values interpolatePose2Traj(const gtsam::Values& opt_values,
    const gtsam::SharedNoiseModel Qc_model, double delta_t, size_t inter_step, 
    size_t start_index, size_t end_index) {

  Values results;

  double inter_dt = delta_t / static_cast<double>(inter_step + 1);
  size_t result_index = 0;

  for (size_t i = start_index; i < end_index; i++) {

    results.insert(Symbol('x', result_index), opt_values.at<Pose2>(Symbol('x', i)));
    results.insert(Symbol('v', result_index), opt_values.at<Vector>(Symbol('v', i)));

    for (size_t inter_idx = 1; inter_idx <= inter_step; inter_idx++) {

      result_index++;
      double tau = static_cast<double>(inter_idx) * inter_dt;
      GaussianProcessInterpolatorPose2 gp_inter(Qc_model, delta_t, tau);
      Pose2 conf1 = opt_values.at<Pose2>(Symbol('x', i));
      Vector vel1  = opt_values.at<Vector>(Symbol('v', i));
      Pose2 conf2 = opt_values.at<Pose2>(Symbol('x', i+1));
      Vector vel2  = opt_values.at<Vector>(Symbol('v', i+1));
      Pose2 conf  = gp_inter.interpolatePose(conf1, vel1, conf2, vel2);
      Vector vel  = gp_inter.interpolateVelocity(conf1, vel1, conf2, vel2);
      results.insert(Symbol('x', result_index), conf);
      results.insert(Symbol('v', result_index), vel);
    }

    result_index++;
  }

  results.insert(Symbol('x', result_index), opt_values.at<Pose2>(Symbol('x', end_index)));
  results.insert(Symbol('v', result_index), opt_values.at<Vector>(Symbol('v', end_index)));

  return results;
}

}
