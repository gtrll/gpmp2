/**
 *  @file  Pose2Mobile2ArmsModel.h
 *  @brief mobile pose2 + 2 x Arm with physical body, which represented by spheres
 *  @author Jing Dong
 *  @date  Aug 20, 2016
 **/

#pragma once

#include <gpmp2/kinematics/Pose2Mobile2Arms.h>
#include <gpmp2/kinematics/RobotModel.h>

namespace gpmp2 {

/**
 * Pose2 + Arm with physical body, which is represented by spheres
 * Used to check collisions
 */
typedef RobotModel<Pose2Mobile2Arms> Pose2Mobile2ArmsModel;

}

