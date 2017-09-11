/**
 *  @file   ObstacleSDFFactorPose2Mobile2Arms.h
 *  @brief  Obstacle avoidance cost factor for 2 X 3D arms with SE(2) base
 *  @author Mustafa Mukadam
 *  @date   Sep 4, 2017
 **/

#pragma once

#include <gpmp2/kinematics/Pose2Mobile2ArmsModel.h>
#include <gpmp2/obstacle/ObstacleSDFFactor.h>

namespace gpmp2 {

// template use Pose2Mobile2ArmsModel as robot type
typedef ObstacleSDFFactor<Pose2Mobile2ArmsModel> ObstacleSDFFactorPose2Mobile2Arms;

}
