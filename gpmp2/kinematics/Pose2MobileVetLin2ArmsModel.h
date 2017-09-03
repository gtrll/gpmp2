/**
 *  @file   Pose2MobileVetLin2ArmsModel.h
 *  @brief  mobile pose2 + linear actuator + 2 x Arm with physical body, represented by spheres
 *  @author Mustafa Mukadam
 *  @date   Sep 3, 2017
 **/

#pragma once

#include <gpmp2/kinematics/Pose2MobileVetLin2Arms.h>
#include <gpmp2/kinematics/RobotModel.h>

namespace gpmp2 {

/**
 * Pose2 + lin actuator + 2 X Arm with physical body, represented by spheres
 * used to check collisions
 */
typedef RobotModel<Pose2MobileVetLin2Arms> Pose2MobileVetLin2ArmsModel;

}
