/**
 *  @file   ObstaclePlanarSDFFactorGPPose2MobileArm.h
 *  @brief  Obstacle avoidance cost factor, for point robot, using signed distance field,
 *          and GP interpolation
 *  @author Jing Dong
 *  @date   Oct 14, 2016
 **/

#pragma once

#include <gpmp2/kinematics/Pose2MobileArmModel.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2Vector.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGP.h>

namespace gpmp2 {

// template uses PointRobotModel as robot type
typedef ObstaclePlanarSDFFactorGP<Pose2MobileArmModel, GaussianProcessInterpolatorPose2Vector>
    ObstaclePlanarSDFFactorGPPose2MobileArm;

}
