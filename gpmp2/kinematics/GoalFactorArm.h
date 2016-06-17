/**
 *  @file  GoalFactorArm.h
 *  @brief generate error for guild an Arm to reach a 3D point destination
 *  @author Jing Dong
 *  @date  Nov 23, 2015
 **/


#pragma once

#include <gpmp2/kinematics/Arm.h>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

#include <iostream>
#include <vector>

namespace gpmp2 {

/**
 * unary factor connect to the last pose in arm configuration space
 */
class GoalFactorArm: public gtsam::NoiseModelFactor1<gtsam::Vector> {

private:
  // typedefs
  typedef GoalFactorArm This;
  typedef gtsam::NoiseModelFactor1<gtsam::Vector> Base;

  // arm
  Arm arm_;

  // destination point
  gtsam::Point3 dest_point_;

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  GoalFactorArm() {} /* Default constructor do nothing */

  /**
   * Constructor
   * @param cost_model cost function covariance
   */
  GoalFactorArm(gtsam::Key poseKey, const gtsam::SharedNoiseModel& cost_model,
      const Arm& arm, const gtsam::Point3& dest_point) :
        Base(cost_model, poseKey), arm_(arm), dest_point_(dest_point) {}

  virtual ~GoalFactorArm() {}


  /// error function
  gtsam::Vector evaluateError(
      const gtsam::Vector& conf, boost::optional<gtsam::Matrix&> H1 = boost::none) const {

    using namespace gtsam;

    // fk
    std::vector<Pose3> joint_pos;
    std::vector<Matrix> J_jpx_jp;
    arm_.forwardKinematics(conf, boost::none, joint_pos, boost::none, J_jpx_jp);

    if (H1) {
      Matrix36 Hpp;
      Point3 end_point = joint_pos[arm_.dof() - 1].translation(Hpp);
      *H1 = Hpp * J_jpx_jp[arm_.dof() - 1];
      return end_point.vector() - dest_point_.vector();

    } else {
      return joint_pos[arm_.dof() - 1].translation().vector() - dest_point_.vector();
    }
  }


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "GoalFactorArm :" << std::endl;
    Base::print("", keyFormatter);
    std::cout << "dest : "; dest_point_.print();
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NoiseModelFactor4",
        boost::serialization::base_object<Base>(*this));
  }
};

}
