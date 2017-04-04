/**
 *  @file   ObstacleSDFFactorGPPose2MobileArm.h
 *  @brief  Obstacle avoidance interpolated GP factor for 3D arm with SE(2) base
 *  @author Mustafa Mukadam
 *  @date   Nov 2, 2016
 **/

#pragma once

#include <gpmp2/kinematics/Pose2MobileArmModel.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2Vector.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGP.h>

namespace gpmp2 {

// template uses Pose2MobileArmModel as robot type
typedef ObstacleSDFFactorGP<Pose2MobileArmModel, GaussianProcessInterpolatorPose2Vector>
    ObstacleSDFFactorGPPose2MobileArm;

}
