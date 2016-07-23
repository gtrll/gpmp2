/**
 *  @file   PointRobotModel.h
 *  @brief  PointRobot with a sphere at the centroid
 *  @author Mustafa Mukadam
 *  @date   July 20, 2016
 **/

#pragma once

#include <gpmp2/kinematics/PointRobot.h>
#include <gpmp2/kinematics/RobotModel.h>

namespace gpmp2 {

/**
 * PointRobot with a sphere at the centroid
 */
typedef RobotModel<PointRobot> PointRobotModel;

}
