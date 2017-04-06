/**
 *  @file  matlabUtils.cpp
 *  @brief utility functions wrapped in matlab
 *  @author Jing Dong
 *  @date  Oct 14, 2016
 **/

#include <gpmp2/utils/matlabUtils.h>


namespace gpmp2 {

/* ************************************************************************** */
void insertPose2VectorInValues(gtsam::Key key, const gpmp2::Pose2Vector& p,
    gtsam::Values& values) {
  values.insert(key, p);
}

/* ************************************************************************** */
gpmp2::Pose2Vector atPose2VectorValues(gtsam::Key key, const gtsam::Values& values) {
  return values.at<Pose2Vector>(key);
}

}


