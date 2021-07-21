/**
 *  @file   SelfCollisionArm.h
 *  @brief  Self collision cost factor for Arm
 *  @author Mustafa Mukadam
 *  @date   Sep 22, 2020
 **/


#pragma once

#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/obstacle/SelfCollision.h>

namespace gpmp2 {

// template use ArmModel as robot type
typedef SelfCollision<ArmModel> SelfCollisionArm;

}
