/**
 *  @file   ObstacleSDFFactorPose2MobileBase.h
 *  @brief  Obstacle avoidance cost factor for 3D SE(2) base
 *  @author Mustafa Mukadam
 *  @date   Jan 23, 2018
 **/

#pragma once

#include <gpmp2/kinematics/Pose2MobileBaseModel.h>
#include <gpmp2/obstacle/ObstacleSDFFactor.h>

namespace gpmp2 {

// template use Pose2MobileBaseModel as robot type
typedef ObstacleSDFFactor<Pose2MobileBaseModel> ObstacleSDFFactorPose2MobileBase;

}
