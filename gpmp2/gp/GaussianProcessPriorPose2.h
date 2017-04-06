/**
 *  @file  GaussianProcessPriorPose2.h
 *  @brief Pose2 GP prior
 *  @author Jing Dong
 **/

#pragma once

#include <gpmp2/gp/GaussianProcessPriorLie.h>

#include <gtsam/geometry/Pose2.h>

namespace gpmp2 {

typedef GaussianProcessPriorLie<gtsam::Pose2> GaussianProcessPriorPose2;

} // \ namespace gpmp2

