/**
 *  @file  GaussianProcessPriorPose3.h
 *  @brief Pose2 GP prior
 *  @author Jing Dong
 **/

#pragma once

#include <gpmp2/gp/GaussianProcessPriorLie.h>

#include <gtsam/geometry/Pose3.h>

namespace gpmp2 {

typedef GaussianProcessPriorLie<gtsam::Pose3> GaussianProcessPriorPose3;

} // \ namespace gpmp2

