/**
 *  @file   SelfCollision.h
 *  @brief  Self collision cost factor
 *  @author Mustafa Mukadam
 *  @date   Sep 22, 2020
 **/


#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>

#include <iostream>
#include <vector>


namespace gpmp2 {

/**
 * unary factor for self collision avoidance
 * template robot model version
 */
template <class ROBOT>
class SelfCollision: public gtsam::NoiseModelFactor1<typename ROBOT::Pose> {

public:
  // typedefs
  typedef ROBOT Robot;
  typedef typename Robot::Pose Pose;

private:
  // typedefs
  typedef SelfCollision This;
  typedef gtsam::NoiseModelFactor1<Pose> Base;

  /** (N x 4) matrix of parameters
   * rows N : number of self collision checks
   * col  0 : id of sphere A
   * col  1 : id of sphere B
   * col  2 : epsilon or safety distance needed between sphere A and B
   * col  3 : sigma for cost between A and B
   */
  gtsam::Matrix data_;

  // arm: planar one, all alpha = 0
  const Robot& robot_;

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /* Default constructor do nothing */
  SelfCollision() : robot_(Robot()) {}

  SelfCollision(gtsam::Key poseKey, const Robot& robot, const gtsam::Matrix& data) :
      robot_(robot), data_(data), Base(gtsam::noiseModel::Diagonal::Sigmas(data.col(3)), poseKey) {}

  virtual ~SelfCollision() {}


  /// error function
  gtsam::Vector evaluateError(const typename Robot::Pose& conf,
      boost::optional<gtsam::Matrix&> H = boost::none) const {
    
    gtsam::Vector error(data_.rows());
    if (H) {
      *H = gtsam::Matrix::Zero(data_.rows(), robot_.dof());
    }

    // forward kinematics to get location of robot body spheres
    std::vector<gtsam::Point3> sph_centers;
    std::vector<gtsam::Matrix> Hs_sph_conf;
    if (H) {
      robot_.sphereCenters(conf, sph_centers, Hs_sph_conf);
    }
    else {
      robot_.sphereCenters(conf, sph_centers);
    }

    // for each sphere pair of interest evaluate self collision
    for (size_t i = 0; i < data_.rows(); i++) {

      size_t sph_A_id = static_cast<size_t>(data_(i, 0));
      size_t sph_B_id = static_cast<size_t>(data_(i, 1));
      const double total_eps = robot_.sphere_radius(sph_A_id) + robot_.sphere_radius(sph_B_id) + data_(i, 2);

      if (H) {
        gtsam::Matrix13 H_e_sphA, H_e_sphB;
        error[i] = hingeLossSelfCollisionCost(sph_centers[sph_A_id], sph_centers[sph_B_id], total_eps, H_e_sphA, H_e_sphB);
        gtsam::Matrix16 H_e_sph;
        H_e_sph  << H_e_sphA, H_e_sphB;
        gtsam::Matrix H_sph_conf(6, robot_.dof());
        H_sph_conf << Hs_sph_conf[sph_A_id], Hs_sph_conf[sph_B_id];
        H->row(i) = H_e_sph * H_sph_conf;
      }
      else {
        error[i] = hingeLossSelfCollisionCost(sph_centers[sph_A_id], sph_centers[sph_B_id], total_eps);
      }
    }

    return error;
  }

  double hingeLossSelfCollisionCost(const gtsam::Point3& pointA, const gtsam::Point3& pointB, double eps,
    gtsam::OptionalJacobian<1, 3> H_pointA = boost::none, gtsam::OptionalJacobian<1, 3> H_pointB = boost::none) const {

    gtsam::Matrix H_A, H_B;
    double dist = gtsam::distance3(pointA, pointB, H_A, H_B);

    if (dist > eps) {
      if (H_pointA)
        *H_pointA = gtsam::Matrix13::Zero();
      if (H_pointB)
        *H_pointB = gtsam::Matrix13::Zero();
      return 0.0;
    }
    else {
      if (H_pointA)
        *H_pointA = -H_A;
      if (H_pointB)
        *H_pointB = -H_B;
      return eps - dist;
    }
  }


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "SelfCollision :" << std::endl;
    Base::print("", keyFormatter);
  }


  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NoiseModelFactor1",
        boost::serialization::base_object<Base>(*this));
  }
};

}
