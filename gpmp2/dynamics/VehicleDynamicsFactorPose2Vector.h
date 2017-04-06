/**
 *  @file  VehicleDynamicsFactorPose2Vector.h
 *  @brief simple 2D vehicle dynamics factor for mobile arm base
 *  @author Jing Dong
 *  @date  Oct 14, 2016
 **/


#pragma once

#include <gpmp2/dynamics/VehicleDynamics.h>
#include <gpmp2/geometry/Pose2Vector.h>

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
class VehicleDynamicsFactorPose2Vector: public gtsam::NoiseModelFactor2<Pose2Vector, gtsam::Vector> {

private:
  // typedefs
  typedef VehicleDynamicsFactorPose2Vector This;
  typedef gtsam::NoiseModelFactor2<Pose2Vector, gtsam::Vector> Base;

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /* Default constructor do nothing */
  VehicleDynamicsFactorPose2Vector() {}

  /**
   * Constructor
   * @param cost_model cost function covariance, should to identity model
   * @param field      signed distance field
   * @param nn_index   nearest neighbour index of signed distance field
   */
  VehicleDynamicsFactorPose2Vector(gtsam::Key poseKey, gtsam::Key velKey, double cost_sigma) :
        Base(gtsam::noiseModel::Isotropic::Sigma(1, cost_sigma), poseKey, velKey) {}

  virtual ~VehicleDynamicsFactorPose2Vector() {}


  /// error function
  /// numerical jacobians / analytic jacobians from cost function
  gtsam::Vector evaluateError(const Pose2Vector& conf, const gtsam::Vector& vel,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const {

    using namespace gtsam;

    if (H1 || H2) {
      Matrix13 Hp, Hv;
      const double err = simple2DVechileDyanmics(conf.pose(),
          vel.head<3>(), Hp, Hv);
      if (H1) {
        *H1 = Matrix::Zero(1, conf.dim());
        H1->block<1,3>(0,0) = Hp;
      }
      if (H2) {
        *H2 = Matrix::Zero(1, conf.dim());
        H2->block<1,3>(0,0) = Hv;
      }
      return (Vector(1) << err).finished();

    } else {
      return (Vector(1) << simple2DVechileDyanmics(conf.pose(),
          vel.head<3>())).finished();
    }
  }


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "VehicleDynamicsFactorPose2Vector :" << std::endl;
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


