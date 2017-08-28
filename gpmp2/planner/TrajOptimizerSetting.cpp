/**
 *  @file  TrajOptimizerSetting.cpp
 *  @brief settings of trajectory optimizer
 *  @author Jing Dong, Mustafa Mukadam
 *  @date  May 11, 2015
 **/

#include <gpmp2/planner/TrajOptimizerSetting.h>

using namespace gtsam;

namespace gpmp2 {

/* ************************************************************************** */
TrajOptimizerSetting::TrajOptimizerSetting() :
    dof(0),
    total_step(10),
    total_time(1.0),
    epsilon(0.2),
    cost_sigma(0.1),
    obs_check_inter(5),
    opt_type(LM),
    opt_verbosity(None),
    final_iter_no_increase(true),
    rel_thresh(1e-6),
    max_iter(100) {
}

/* ************************************************************************** */
TrajOptimizerSetting::TrajOptimizerSetting(size_t system_dof) :
    dof(system_dof),
    total_step(10),
    total_time(1.0),
    conf_prior_model(noiseModel::Isotropic::Sigma(system_dof, 0.0001)),
    vel_prior_model(noiseModel::Isotropic::Sigma(system_dof, 0.0001)),
    epsilon(0.2),
    cost_sigma(0.1),
    obs_check_inter(5),
    Qc_model(noiseModel::Unit::Create(system_dof)),
    opt_type(LM),
    opt_verbosity(None),
    final_iter_no_increase(true),
    rel_thresh(1e-6),
    max_iter(100) {
}

/* ************************************************************************** */
void TrajOptimizerSetting::set_conf_prior_model(double sigma) {
  conf_prior_model = noiseModel::Isotropic::Sigma(dof, sigma);
}

/* ************************************************************************** */
void TrajOptimizerSetting::set_vel_prior_model(double sigma) {
  vel_prior_model = noiseModel::Isotropic::Sigma(dof, sigma);
}

/* ************************************************************************** */
void TrajOptimizerSetting::set_Qc_model(const Matrix& Qc) {
  Qc_model = noiseModel::Gaussian::Covariance(Qc);
}


}

