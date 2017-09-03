/**
 *  @file   Pose2MobileVetLinArmModel.h
 *  @brief  mobile pose2 + linear actuator + Arm with physical body, represented by spheres
 *  @author Mustafa Mukadam
 *  @date   Sep 3, 2017
 **/

#pragma once

#include <gpmp2/kinematics/Pose2MobileVetLinArm.h>
#include <gpmp2/kinematics/RobotModel.h>

namespace gpmp2 {

/**
 * Pose2 + lin actuator + Arm with physical body, represented by spheres
 * used to check collisions
 */
typedef RobotModel<Pose2MobileVetLinArm> Pose2MobileVetLinArmModel;

}
