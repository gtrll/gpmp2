/**
 *  @file   ObstacleSDFFactorPose2MobileArm.h
 *  @brief  Obstacle avoidance cost factor for 3D arm with SE(2) base
 *  @author Mustafa Mukadam
 *  @date   Nov 2, 2016
 **/

#pragma once

#include <gpmp2/kinematics/Pose2MobileArmModel.h>
#include <gpmp2/obstacle/ObstacleSDFFactor.h>

namespace gpmp2 {

// template use Pose2MobileArmModel as robot type
typedef ObstacleSDFFactor<Pose2MobileArmModel> ObstacleSDFFactorPose2MobileArm;

}
