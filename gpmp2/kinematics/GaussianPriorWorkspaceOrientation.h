/**
 *  @file   GaussianPriorWorkspaceOrientation.h
 *  @brief  Gaussian prior defined on the workspace orientation of any joint of a robot
 *          given its state in configuration space
 *  @author Mustafa Mukadam
 *  @date   Jan 8, 2018
 **/


#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

#include <gpmp2/kinematics/RobotModel.h>

namespace gpmp2 {

/**
 * Gaussian prior defined on the workspace orientation
 */
template <class ROBOT>
class GaussianPriorWorkspaceOrientation: public gtsam::NoiseModelFactor1<typename ROBOT::Pose> {

public:
  // typedefs
  typedef ROBOT Robot;
  typedef typename Robot::Pose Pose;

private:
  // typedefs
  typedef GaussianPriorWorkspaceOrientation This;
  typedef gtsam::NoiseModelFactor1<Pose> Base;

  const Robot& robot_; // Robot
  int joint_; // joint on the robot to be constrained
  gtsam::Rot3 des_orientation_; // desired workspace orientation for joint

public:
  /* Default constructor do nothing */
  GaussianPriorWorkspaceOrientation() {}

  /// Constructor
  GaussianPriorWorkspaceOrientation(gtsam::Key poseKey, const Robot& robot, int joint, 
      const gtsam::Rot3& des_orientation, const gtsam::SharedNoiseModel& cost_model) :
        Base(cost_model, poseKey), robot_(robot), joint_(joint), des_orientation_(des_orientation){}

  virtual ~GaussianPriorWorkspaceOrientation() {}

  /// factor error function
  gtsam::Vector evaluateError(
      const Pose& pose, boost::optional<gtsam::Matrix&> H1 = boost::none) const {

    using namespace gtsam;

    std::vector<Pose3> joint_pos;
    std::vector<Matrix> J_jpx_jp;
    robot_.fk_model().forwardKinematics(pose, boost::none, joint_pos, boost::none, J_jpx_jp);

    if (H1) {
      Matrix36 H_rp;
      Matrix33 H_er;
      Rot3 curr_orientation = joint_pos[joint_].rotation(H_rp);
      Vector error = des_orientation_.logmap(curr_orientation, boost::none, H_er);
      *H1 = H_er * H_rp * J_jpx_jp[joint_];
      return error;
    }
    else {
      return des_orientation_.logmap(joint_pos[joint_].rotation());
    }
  }


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "GaussianPriorWorkspaceOrientation :" << std::endl;
    Base::print("", keyFormatter);
    std::cout << "desired orientation : "; des_orientation_.print();
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }

};

}
