/**
 * @file GaussianProcessInterpolatorPose3.h
 * @brief Base and utils for Gaussian Process Interpolated measurement factor, works only in SE(3)
 * @author Jing Dong
 */

#pragma once

#include <gpmp2/gp/GaussianProcessInterpolatorLie.h>

#include <gtsam/geometry/Pose3.h>

namespace gpmp2 {

typedef GaussianProcessInterpolatorLie<gtsam::Pose3> GaussianProcessInterpolatorPose3;

} // \ namespace gpmp2

