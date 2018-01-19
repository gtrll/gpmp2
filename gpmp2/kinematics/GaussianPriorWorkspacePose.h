/**
 *  @file   GaussianPriorWorkspacePose.h
 *  @brief  Gaussian prior defined on the workspace pose of any joint of a robot
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
 * Gaussian prior defined on the workspace pose
 */
template <class ROBOT>
class GaussianPriorWorkspacePose: public gtsam::NoiseModelFactor1<typename ROBOT::Pose> {

public:
  // typedefs
  typedef ROBOT Robot;
  typedef typename Robot::Pose Pose;

private:
  // typedefs
  typedef GaussianPriorWorkspacePose This;
  typedef gtsam::NoiseModelFactor1<Pose> Base;

  const Robot& robot_; // Robot
  int joint_; // joint on the robot to be constrained
  gtsam::Pose3 des_pose_; // desired workspace pose for joint

public:
  /* Default constructor do nothing */
  GaussianPriorWorkspacePose() {}

  /// Constructor
  GaussianPriorWorkspacePose(gtsam::Key poseKey, const Robot& robot, int joint, 
      const gtsam::Pose3& des_pose, const gtsam::SharedNoiseModel& cost_model) :
        Base(cost_model, poseKey), robot_(robot), joint_(joint), des_pose_(des_pose){}

  virtual ~GaussianPriorWorkspacePose() {}

  /// factor error function
  gtsam::Vector evaluateError(
      const Pose& pose, boost::optional<gtsam::Matrix&> H1 = boost::none) const {

    using namespace gtsam;

    std::vector<Pose3> joint_pos;
    std::vector<Matrix> J_jpx_jp;
    robot_.fk_model().forwardKinematics(pose, boost::none, joint_pos, boost::none, J_jpx_jp);

    if (H1) {
      Matrix66 H_ep;
      Vector error = des_pose_.logmap(joint_pos[joint_], boost::none, H_ep);
      *H1 = H_ep * J_jpx_jp[joint_];
      return error;
    }
    else {
      return des_pose_.logmap(joint_pos[joint_]);
    }
  }


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "GaussianPriorWorkspacePose :" << std::endl;
    Base::print("", keyFormatter);
    std::cout << "desired pose : "; des_pose_.print();
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
