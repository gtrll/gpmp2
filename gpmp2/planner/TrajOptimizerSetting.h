/**
 *  @file  TrajOptimizerSetting.h
 *  @brief settings of trajectory optimizer
 *  @author Jing Dong
 *  @date  May 10, 2015
 **/

#pragma once

#include <gtsam/linear/NoiseModel.h>


namespace gpmp2 {

/// general setting of all trajectory optimizers, batch and incremental
struct TrajOptimizerSetting {

  /// optimization iteration types
  enum IterationType {GaussNewton, LM, Dogleg};

  /// trajectory settings
  size_t dof;                   // degree of freedom, must be given explicitly
  size_t total_step;            // number of steps (states) optimized the whole trajectory
  double total_time;            // time duration (second) of the whole trajectory
  gtsam::SharedNoiseModel conf_prior_model;    // prior constraint model for initial/end state
  gtsam::SharedNoiseModel vel_prior_model;     // prior constraint model for initial/end velocity

  /// obstacle cost settings
  double epsilon;               // eps of hinge loss function (see the paper)
  double cost_sigma;            // sigma of obstacle cost (see the paper)
  size_t obs_check_inter;       // number of point interpolated for obstacle cost,
                                // 0 means do not interpolate
  /// GP settings
  gtsam::SharedNoiseModel Qc_model;    // Qc for GP (see the paper)

  /// Optimization settings
  IterationType opt_type;       // optimizer type
  double rel_thresh;            // relative error decrease threshold for stopping optimization
  size_t max_iter;              // max iteration for stopping optimization


  /// default parameters, DOF must be given explicitly
  TrajOptimizerSetting(size_t system_dof);

  ~TrajOptimizerSetting() {}


  /* matlab utils for wrapper */
  void set_total_step(size_t step) { total_step = step; }
  void set_total_time(double time) { total_time = time; }
  void set_epsilon(double eps) { epsilon = eps; }
  void set_cost_sigma(double sigma) { cost_sigma = sigma; }
  void set_obs_check_inter(size_t inter) { obs_check_inter = inter; }
  void set_rel_thresh(double thresh) { rel_thresh = thresh; }
  void set_max_iter(size_t iter) { max_iter = iter; }
  void set_conf_prior_model(double sigma);
  void set_vel_prior_model(double sigma);
  void set_Qc_model(const gtsam::Matrix& Qc);

  /// set optimization type
  void setGaussNewton() { opt_type = GaussNewton; }
  void setLM() { opt_type = LM; }
  void setDogleg() { opt_type = Dogleg; }
};

}

