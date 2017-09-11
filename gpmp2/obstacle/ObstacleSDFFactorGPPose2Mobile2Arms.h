/**
 *  @file   ObstacleSDFFactorGPPose2Mobile2Arms.h
 *  @brief  Obstacle avoidance interpolated GP factor for 2 X 3D arms with SE(2) base
 *  @author Mustafa Mukadam
 *  @date   Sep 4, 2017
 **/

#pragma once

#include <gpmp2/kinematics/Pose2Mobile2ArmsModel.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2Vector.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGP.h>

namespace gpmp2 {

// template uses Pose2Mobile2ArmsModel as robot type
typedef ObstacleSDFFactorGP<Pose2Mobile2ArmsModel, GaussianProcessInterpolatorPose2Vector>
    ObstacleSDFFactorGPPose2Mobile2Arms;

}
