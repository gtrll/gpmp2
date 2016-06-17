/**
 * @file GaussianProcessInterpolatorLinear.h
 * @brief Base and utils for Gaussian Process Interpolation, linear version
 * @author Jing Dong, Xinyan Yan
 * @date Oct 26, 2015
 */

#pragma once

#include <gpmp2/gp/GPutils.h>

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Matrix.h>

#include <boost/serialization/array.hpp>


namespace gpmp2 {

/**
 * 4-way factor for Gaussian Process interpolator, linear version
 * interpolate pose and velocity given consecutive poses and velocities
 */
class GaussianProcessInterpolatorLinear {

private:
  typedef GaussianProcessInterpolatorLinear This;

  size_t dof_;
  double delta_t_;		// t_{i+1} - t_i
  double tau_;			// tau - t_i. we use tau as time interval from t_i instead of from t_0 as in Barfoot papers

  gtsam::Matrix Qc_;
  gtsam::Matrix Lambda_;
  gtsam::Matrix Psi_;

public:

  /// Default constructor: only for serialization
  GaussianProcessInterpolatorLinear() {}

  /**
   * Constructor
   * @param Qc noise model of Qc
   * @param delta_t the time between the two states
   * @param tau the time of interval status
   */
  GaussianProcessInterpolatorLinear(const gtsam::SharedNoiseModel Qc_model,
      double delta_t, double tau) : dof_(Qc_model->dim()), delta_t_(delta_t), tau_(tau) {

    // Calcuate Lambda and Psi
    Qc_ = getQc(Qc_model);
    Lambda_ = calcLambda(Qc_, delta_t_, tau_);
    Psi_ = calcPsi(Qc_, delta_t_, tau_);
  }

  /** Virtual destructor */
  virtual ~GaussianProcessInterpolatorLinear() {}


  /// interpolate pose with Jacobians
  gtsam::Vector interpolatePose(
      const gtsam::Vector& pose1, const gtsam::Vector& vel1,
      const gtsam::Vector& pose2, const gtsam::Vector& vel2,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H1 = boost::none,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H2 = boost::none,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H3 = boost::none,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H4 = boost::none) const {

    using namespace gtsam;

    // state vector
    Vector x1 = (Vector(2*dof_) << pose1, vel1).finished();
    Vector x2 = (Vector(2*dof_) << pose2, vel2).finished();

    // jacobians
    if (H1) *H1 = Lambda_.block(0, 0, dof_, dof_);
    if (H2) *H2 = Lambda_.block(0, dof_, dof_, dof_);
    if (H3) *H3 = Psi_.block(0, 0, dof_, dof_);
    if (H4) *H4 = Psi_.block(0, dof_, dof_, dof_);

    // interpolate pose (just calculate upper part of the interpolated state vector to save time)
    return Lambda_.block(0, 0, dof_, 2*dof_) * x1 + Psi_.block(0, 0, dof_, 2*dof_) * x2;
  }


  /// update jacobian based on interpolated jacobians
  static void updatePoseJacobians(const gtsam::Matrix& Hpose,  const gtsam::Matrix& Hint1,
      const gtsam::Matrix& Hint2, const gtsam::Matrix& Hint3, const gtsam::Matrix& Hint4,
      boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
      boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4) {
    if (H1) *H1 = Hpose * Hint1;
    if (H2) *H2 = Hpose * Hint2;
    if (H3) *H3 = Hpose * Hint3;
    if (H4) *H4 = Hpose * Hint4;
  }


  /// interpolate velocity with Jacobians
  gtsam::Vector interpolateVelocity(
      const gtsam::Vector& pose1, const gtsam::Vector& vel1,
      const gtsam::Vector& pose2, const gtsam::Vector& vel2,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H1 = boost::none,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H2 = boost::none,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H3 = boost::none,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H4 = boost::none) const {

    using namespace gtsam;

    // state vector
    Vector x1 = (Vector(2*dof_) << pose1, vel1).finished();
    Vector x2 = (Vector(2*dof_) << pose2, vel2).finished();

    // jacobians
    if (H1) *H1 = Lambda_.block(dof_, 0, dof_, dof_);
    if (H2) *H2 = Lambda_.block(dof_, dof_, dof_, dof_);
    if (H3) *H3 = Psi_.block(dof_, 0, dof_, dof_);
    if (H4) *H4 = Psi_.block(dof_, dof_, dof_, dof_);

    // interpolate pose (just calculate lower part of the interpolated state vector to save time)
    return Lambda_.block(dof_, 0, dof_, 2*dof_) * x1 + Psi_.block(dof_, 0, dof_, 2*dof_) * x2;
  }

  /** demensions */
  size_t dim() const { return dof_; }


  /**
   * Testables
   */

  /** equals specialized to this factor */
  virtual bool equals(const This& expected, double tol=1e-9) const {
    return fabs(this->delta_t_ - expected.delta_t_) < tol &&
        fabs(this->tau_ - expected.tau_) < tol &&
        gtsam::equal_with_abs_tol(this->Qc_, expected.Qc_, tol) &&
        gtsam::equal_with_abs_tol(this->Lambda_, expected.Lambda_, tol) &&
        gtsam::equal_with_abs_tol(this->Psi_, expected.Psi_, tol);
  }

  /** print contents */
  void print(const std::string& s="") const {
    std::cout << s << "GaussianProcessInterpolatorLinear(" << dof_ << ")" << std::endl;
    std::cout << "delta_t = " << delta_t_ << ", tau = " << tau_ << std::endl;
    //std::cout << "Qc = " << Qc_ << std::endl;
  }


private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(dof_);
    ar & BOOST_SERIALIZATION_NVP(delta_t_);
    ar & BOOST_SERIALIZATION_NVP(tau_);
    using namespace boost::serialization;
    ar & make_nvp("Qc", make_array(Qc_.data(), Qc_.size()));
    ar & make_nvp("Lambda", make_array(Lambda_.data(), Lambda_.size()));
    ar & make_nvp("Psi", make_array(Psi_.data(), Psi_.size()));
  }
};

} // \ namespace gpmp2


/// traits
namespace gtsam {
template<>
struct traits<gpmp2::GaussianProcessInterpolatorLinear> : public Testable<
    gpmp2::GaussianProcessInterpolatorLinear> {};
}

