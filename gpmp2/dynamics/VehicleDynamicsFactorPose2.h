/**
 *  @file   VehicleDynamicsFactorPose2.h
 *  @brief  simple 2D vehicle dynamics factor for mobile base in Lie group SE(2)
 *  @author Mustafa Mukadam
 *  @date   Jan 24, 2018
 **/


#pragma once

#include <gpmp2/dynamics/VehicleDynamics.h>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>

#include <iostream>
#include <vector>


namespace gpmp2 {

/**
 * unary factor for vehicle dynamics
 */
class VehicleDynamicsFactorPose2: public gtsam::NoiseModelFactor2<gtsam::Pose2, gtsam::Vector> {

private:
  // typedefs
  typedef VehicleDynamicsFactorPose2 This;
  typedef gtsam::NoiseModelFactor2<gtsam::Pose2, gtsam::Vector> Base;

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /* Default constructor do nothing */
  VehicleDynamicsFactorPose2() {}

  /**
   * Constructor
   * @param cost_sigma cost function covariance, should to identity model
   */
  VehicleDynamicsFactorPose2(gtsam::Key poseKey, gtsam::Key velKey, double cost_sigma) :
        Base(gtsam::noiseModel::Isotropic::Sigma(1, cost_sigma), poseKey, velKey) {}

  virtual ~VehicleDynamicsFactorPose2() {}


  /// error function
  /// numerical/analytic Jacobians from cost function
  gtsam::Vector evaluateError(const gtsam::Pose2& pose, const gtsam::Vector& vel,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const {

    using namespace gtsam;

    if (H1 || H2) {
      Matrix13 Hp, Hv;
      const double err = simple2DVehicleDynamicsPose2(pose, vel.head<3>(), Hp, Hv);
      if (H1) {
        *H1 = Matrix::Zero(1, 3);
        H1->block<1,3>(0,0) = Hp;
      }
      if (H2) {
        *H2 = Matrix::Zero(1, 3);
        H2->block<1,3>(0,0) = Hv;
      }
      return (Vector(1) << err).finished();

    } else {
      return (Vector(1) << simple2DVehicleDynamicsPose2(pose, vel.head<3>())).finished();
    }
  }


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "VehicleDynamicsFactorPose2 :" << std::endl;
    Base::print("", keyFormatter);
  }


  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NoiseModelFactor2",
        boost::serialization::base_object<Base>(*this));
  }
};

}


