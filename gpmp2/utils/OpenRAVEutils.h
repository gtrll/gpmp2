/**
 *  @file  OpenRAVEutils.h
 *  @brief utility functions wrapped in OpenRAVE
 *  @author Jing Dong
 *  @date  Oct 31, 2015
 **/

#pragma once

#include <gpmp2/config.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/base/Matrix.h>


namespace gpmp2 {

/// convert values into double pointer to vis in openrave plugin
/// the pointer is size (2 x step) x DOF, include both conf pose and vel
/// the pointer must be allocated and free by outside utils (memory not managed here)
/// also joint limits are applied here to make sure the range of output conf
GPMP2_EXPORT void convertValuesOpenRavePointer(size_t dof, const gtsam::Values& results, 
    double *pointer, size_t total_step, const gtsam::Vector& joint_lower_limit, 
    const gtsam::Vector& joint_upper_limit);


/// convert double pointer in openrave plugin to gtsam values
/// the pointer is size (2 x step) x DOF, include both conf pose and vel
GPMP2_EXPORT void convertOpenRavePointerValues(size_t dof, gtsam::Values& results, 
    double *pointer, size_t total_step);

}


