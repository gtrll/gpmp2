/**
 *  @file   ObstaclePlanarSDFFactorGPPose2MobileBase.h
 *  @brief  Obstacle avoidance interpolated GP factor for 2D SE(2) base
 *  @author Mustafa Mukadam
 *  @date   Jan 23, 2018
 **/

#pragma once

#include <gpmp2/kinematics/Pose2MobileBaseModel.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGP.h>

namespace gpmp2 {

// template uses PointRobotModel as robot type
typedef ObstaclePlanarSDFFactorGP<Pose2MobileBaseModel, GaussianProcessInterpolatorPose2>
    ObstaclePlanarSDFFactorGPPose2MobileBase;

}
