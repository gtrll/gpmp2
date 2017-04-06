/**
 * @file GaussianProcessInterpolatorPose2.h
 * @brief Base and utils for Gaussian Process Interpolated measurement factor, works only in SE(2)
 * @author Jing Dong
 */

#pragma once

#include <gpmp2/gp/GaussianProcessInterpolatorLie.h>

#include <gtsam/geometry/Pose2.h>

namespace gpmp2 {

typedef GaussianProcessInterpolatorLie<gtsam::Pose2> GaussianProcessInterpolatorPose2;

} // \ namespace gpmp2

