/**
 *  @file  ObstacleSDFFactorGPArm.h
 *  @brief Obstacle avoidance cost factor, using Arm planar, linear GP and signed distance field
 *  @author Jing Dong
 *  @date  May 29, 2016
 **/

#pragma once

#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGP.h>

namespace gpmp2 {

// template use ArmModel as robot type
typedef ObstacleSDFFactorGP<ArmModel, GaussianProcessInterpolatorLinear> 
    ObstacleSDFFactorGPArm;

}


