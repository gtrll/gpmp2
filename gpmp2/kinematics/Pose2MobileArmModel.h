/**
 *  @file  Pose2MobileArmModel.h
 *  @brief mobile pose2 + Arm with physical body, which represented by spheres
 *  @author Jing Dong
 *  @date  Oct 13, 2016
 **/

#pragma once

#include <gpmp2/kinematics/Pose2MobileArm.h>
#include <gpmp2/kinematics/RobotModel.h>

namespace gpmp2 {

/**
 * Pose2 + Arm with physical body, which is represented by spheres
 * Used to check collisions
 */
typedef RobotModel<Pose2MobileArm> Pose2MobileArmModel;

}

