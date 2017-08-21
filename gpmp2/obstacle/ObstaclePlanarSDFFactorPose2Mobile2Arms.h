/**
 *  @file  ObstaclePlanarSDFFactorPose2Mobile2Arms.h
 *  @brief Obstacle avoidance cost factor, for mobile robot with 2 arms
 *  @author Jing Dong
 *  @date  Aug 20, 2016
 **/

#pragma once

#include <gpmp2/kinematics/Pose2Mobile2ArmsModel.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactor.h>

namespace gpmp2 {

// template use ArmModel as robot type
typedef ObstaclePlanarSDFFactor<Pose2Mobile2ArmsModel> ObstaclePlanarSDFFactorPose2Mobile2Arms;

}


