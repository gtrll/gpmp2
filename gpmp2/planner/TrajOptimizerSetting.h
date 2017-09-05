/**
 *  @file  TrajOptimizerSetting.h
 *  @brief settings of trajectory optimizer
 *  @author Jing Dong, Mustafa Mukadam
 *  @date  May 10, 2015
 **/

#pragma once

#include <gpmp2/config.h>
#include <gtsam/linear/NoiseModel.h>


namespace gpmp2 {

/// general setting of all trajectory optimizers, batch and incremental
struct GPMP2_EXPORT TrajOptimizerSetting {

  /// optimization iteration types
  enum IterationType {GaussNewton, LM, Dogleg};
  enum VerbosityLevel {None, Error};

  /// trajectory settings
  size_t dof;                   // degree of freedom, must be given explicitly
  size_t total_step;            // number of steps (states) optimized the whole trajectory
  double total_time;            // time duration (second) of the whole trajectory
  gtsam::SharedNoiseModel conf_prior_model;    // prior constraint model for initial/end state
  gtsam::SharedNoiseModel vel_prior_model;     // prior constraint model for initial/end velocity

  /// joint position and velocity limit settings
  bool flag_pos_limit;          // whether enable joint position limits
  bool flag_vel_limit;          // whether enable velocity limits
  gtsam::Vector joint_pos_limits_up, joint_pos_limits_down; // joint position limits, if Pose2Vector, only Vector part
  gtsam::Vector vel_limits;     // joint velocity limits, for all DOF
  /// joint limit settings
  gtsam::Vector pos_limit_thresh, vel_limit_thresh;
  gtsam::SharedNoiseModel pos_limit_model, vel_limit_model;

  /// obstacle cost settings
  double epsilon;               // eps of hinge loss function (see the paper)
  double cost_sigma;            // sigma of obstacle cost (see the paper)
  size_t obs_check_inter;       // number of point interpolated for obstacle cost,
                                // 0 means do not interpolate
  /// GP settings
  gtsam::SharedNoiseModel Qc_model;    // Qc for GP (see the paper)

  /// Optimization settings
  IterationType opt_type;       // optimizer type
  VerbosityLevel opt_verbosity; // verbosity level
  bool final_iter_no_increase;  // value is guaranteed not increase at last iteration
  double rel_thresh;            // relative error decrease threshold for stopping optimization
  size_t max_iter;              // max iteration for stopping optimization

  /// default constructor, must manually set all parameters that depend on dof
  TrajOptimizerSetting();

  /// default parameters, DOF must be given explicitly
  explicit TrajOptimizerSetting(size_t system_dof);

  ~TrajOptimizerSetting() {}


  /* matlab utils for wrapper */
  // traj settings
  void set_total_step(size_t step) { total_step = step; }
  void set_total_time(double time) { total_time = time; }
  void set_conf_prior_model(double sigma);
  void set_vel_prior_model(double sigma);

  // joint limit / velocity settings
  void set_flag_pos_limit(bool flag) { flag_pos_limit = flag; }
  void set_flag_vel_limit(bool flag) { flag_vel_limit = flag; }
  void set_joint_pos_limits_up(const gtsam::Vector& v) { joint_pos_limits_up = v; }
  void set_joint_pos_limits_down(const gtsam::Vector& v) { joint_pos_limits_down = v; }
  void set_vel_limits(const gtsam::Vector& v) { vel_limits = v; }
  void set_pos_limit_thresh(const gtsam::Vector& v) { pos_limit_thresh = v; }
  void set_vel_limit_thresh(const gtsam::Vector& v) { vel_limit_thresh = v; }
  void set_pos_limit_model(const gtsam::Vector& v);
  void set_vel_limit_model(const gtsam::Vector& v);

  // obstacle settings
  void set_epsilon(double eps) { epsilon = eps; }
  void set_cost_sigma(double sigma) { cost_sigma = sigma; }
  void set_obs_check_inter(size_t inter) { obs_check_inter = inter; }

  // GP settings
  void set_Qc_model(const gtsam::Matrix& Qc);

  // optimization settings
  void setGaussNewton() { opt_type = GaussNewton; }
  void setLM() { opt_type = LM; }
  void setDogleg() { opt_type = Dogleg; }
  void set_rel_thresh(double thresh) { rel_thresh = thresh; }
  void set_max_iter(size_t iter) { max_iter = iter; }
  /// set optimization verbosity
  void setVerbosityNone() { opt_verbosity = None; }
  void setVerbosityError() { opt_verbosity = Error; }
  /// set value is guaranteed not increase
  void setOptimizationNoIncrase(bool flag) { final_iter_no_increase = flag; }
};

}

