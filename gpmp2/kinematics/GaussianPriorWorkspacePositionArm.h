/**
 *  @file   GaussianPriorWorkspacePositionArm.h
 *  @brief  Gaussian prior defined on the workspace position of any link of an arm
 *          given its state in configuration space
 *  @author Mustafa Mukadam
 *  @date   Jan 8, 2018
 **/

#pragma once

#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/kinematics/GaussianPriorWorkspacePosition.h>

namespace gpmp2 {

// template use ArmModel as robot type
typedef GaussianPriorWorkspacePosition<ArmModel> GaussianPriorWorkspacePositionArm;

}
