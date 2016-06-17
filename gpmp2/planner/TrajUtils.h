/**
 *  @file  TrajUtils.h
 *  @brief utils for trajectory optimization, include initialization and interpolation
 *  @author Jing Dong
 *  @date  May 11, 2015
 **/

#pragma once

#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>


namespace gpmp2 {

/**
 * @brief initial the trajectory in configuration space as a straight line
 * @param init_conf trajectorystart configuration
 * @param end_conf  trajectoryend configuration
 * @param total_step is how many intervals do you want in the traj
 * @return values, x(0) - x(total_step), v(0) - v(total_step)
 */
gtsam::Values initArmTrajStraightLine(const gtsam::Vector& init_conf,
    const gtsam::Vector& end_conf, size_t total_step);

/**
 * @brief robot arm trajectory interpolater
 * @param opt_values optimized values from optimization
 * @param Qc_model GP hyperparameter
 * @param delta_t time interval between states in opt_values
 * @param output_inter_step is how many steps you want interpolate, 0 is not interpolate
 * @return interpolated values, x(0) - x(total_step), v(0) - v(total_step)
 */
gtsam::Values interpolateArmTraj(const gtsam::Values& opt_values,
    const gtsam::SharedNoiseModel Qc_model, double delta_t, size_t output_inter_step);


}

