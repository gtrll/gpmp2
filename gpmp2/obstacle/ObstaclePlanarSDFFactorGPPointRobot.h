/**
 *  @file   ObstaclePlanarSDFFactorGPPointRobot.h
 *  @brief  Obstacle avoidance cost factor, for point robot, using signed distance field,
 *          and GP interpolation
 *  @author Mustafa Mukadam
 *  @date   July 20, 2016
 **/

#pragma once

#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGP.h>

namespace gpmp2 {

// template uses PointRobotModel as robot type
typedef ObstaclePlanarSDFFactorGP<PointRobotModel, GaussianProcessInterpolatorLinear>
    ObstaclePlanarSDFFactorGPPointRobot;

}
