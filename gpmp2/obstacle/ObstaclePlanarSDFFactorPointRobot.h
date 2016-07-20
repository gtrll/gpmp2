/**
 *  @file   ObstaclePlanarSDFFactorPointRobot.h
 *  @brief  Obstacle avoidance cost factor, for point robot, using signed distance field
 *  @author Mustafa Mukadam
 *  @date   July 20, 2016
 **/

#pragma once

#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactor.h>

namespace gpmp2 {

// template uses PointRobotModel as robot type
typedef ObstaclePlanarSDFFactor<PointRobotModel> ObstaclePlanarSDFFactorPointRobot;

}
