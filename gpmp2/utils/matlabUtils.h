/**
 *  @file  matlabUtils.h
 *  @brief utility functions wrapped in matlab
 *  @author Jing Dong
 *  @date  Oct 14, 2016
 **/

#pragma once

#include <gpmp2/geometry/Pose2Vector.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>


namespace gpmp2 {

/// prior factor on Pose2Vector
typedef gtsam::PriorFactor<gpmp2::Pose2Vector> PriorFactorPose2Vector;


/// Pose2Vector Values utils
void insertPose2VectorInValues(gtsam::Key key, const gpmp2::Pose2Vector& p, gtsam::Values& values);
gpmp2::Pose2Vector atPose2VectorValues(gtsam::Key key, const gtsam::Values& values);

}


