/**
 *  @file   ObstaclePlanarSDFFactorGPPose2Mobile2Arms.h
 *  @brief  Obstacle avoidance cost factor, for mobile robot with 2 arm
 *  @author Jing Dong
 *  @date   Aug 20, 2017
 **/

#pragma once

#include <gpmp2/kinematics/Pose2Mobile2ArmsModel.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2Vector.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGP.h>

namespace gpmp2 {

// template uses PointRobotModel as robot type
typedef ObstaclePlanarSDFFactorGP<Pose2Mobile2ArmsModel, GaussianProcessInterpolatorPose2Vector>
    ObstaclePlanarSDFFactorGPPose2Mobile2Arms;

}
