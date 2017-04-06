/**
 *  @file  GaussianProcessPriorPose2Vector.h
 *  @brief Pose2Vector GP prior
 *  @author Jing Dong
 **/

#pragma once

#include <gpmp2/gp/GaussianProcessPriorLie.h>
#include <gpmp2/geometry/Pose2Vector.h>


namespace gpmp2 {

typedef GaussianProcessPriorLie<Pose2Vector> GaussianProcessPriorPose2Vector;

} // \ namespace gpmp2

