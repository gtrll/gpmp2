/**
 *  @file  ObstacleSDFFactorArm.h
 *  @brief Obstacle avoidance cost factor, using Arm and signed distance field
 *  @author Jing Dong
 *  @date  May 29, 2016
 **/

#pragma once

#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/obstacle/ObstacleSDFFactor.h>

namespace gpmp2 {

// template use ArmModel as robot type
typedef ObstacleSDFFactor<ArmModel> ObstacleSDFFactorArm;

}


