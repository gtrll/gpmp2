/**
 *  @file  GaussianProcessPriorLinear.h
 *  @brief Linear GP prior, see Barfoot14rss
 *  @author Xinyan Yan, Jing Dong
 **/


#pragma once

#include <gpmp2/gp/GPutils.h>

#include <boost/lexical_cast.hpp>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>


namespace gpmp2 {

/**
 * 4-way factor for Gaussian Process prior factor, linear version
 */
class GaussianProcessPriorLinear: public gtsam::NoiseModelFactor4<
    gtsam::Vector, gtsam::Vector, gtsam::Vector, gtsam::Vector> {

private:
  size_t dof_;
  double delta_t_;

  typedef GaussianProcessPriorLinear This;
  typedef gtsam::NoiseModelFactor4<gtsam::Vector, gtsam::Vector, gtsam::Vector,
      gtsam::Vector> Base;

public:

  GaussianProcessPriorLinear() {}	/* Default constructor only for serialization */

  /// Constructor
  /// @param delta_t is the time between the two states
  GaussianProcessPriorLinear(gtsam::Key poseKey1, gtsam::Key velKey1,
      gtsam::Key poseKey2, gtsam::Key velKey2,
      double delta_t, const gtsam::SharedNoiseModel Qc_model) :
        Base(gtsam::noiseModel::Gaussian::Covariance(calcQ(getQc(Qc_model), delta_t)),
        poseKey1, velKey1, poseKey2, velKey2), dof_(Qc_model->dim()),
        delta_t_(delta_t) {}

  virtual ~GaussianProcessPriorLinear() {}


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }


  /// factor error function
  gtsam::Vector evaluateError(
      const gtsam::Vector& pose1, const gtsam::Vector& vel1,
      const gtsam::Vector& pose2, const gtsam::Vector& vel2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none,
      boost::optional<gtsam::Matrix&> H4 = boost::none) const {

    using namespace gtsam;

    // state vector
    Vector x1 = (Vector(2*dof_) << pose1, vel1).finished();
    Vector x2 = (Vector(2*dof_) << pose2, vel2).finished();

    // Jacobians
    if (H1) *H1 = (Matrix(2*dof_, dof_) << Matrix::Identity(dof_, dof_),
        Matrix::Zero(dof_, dof_)).finished();
    if (H2) *H2 = (Matrix(2*dof_, dof_) << delta_t_ * Matrix::Identity(dof_, dof_),
        Matrix::Identity(dof_, dof_)).finished();
    if (H3) *H3 = (Matrix(2*dof_, dof_) << -1.0 * Matrix::Identity(dof_, dof_),
        Matrix::Zero(dof_, dof_)).finished();
    if (H4) *H4 = (Matrix(2*dof_, dof_) << Matrix::Zero(dof_, dof_),
        -1.0 * Matrix::Identity(dof_, dof_)).finished();

    // transition matrix & error
    return calcPhi(dof_, delta_t_) * x1 - x2;
  }


  /** demensions */
  size_t dim() const { return dof_; }

  /** number of variables attached to this factor */
  size_t size() const {
    return 4;
  }

  /** equals specialized to this factor */
  virtual bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const {
    const This *e =  dynamic_cast<const This*> (&expected);
    return e != NULL && Base::equals(*e, tol) && fabs(this->delta_t_ - e->delta_t_) < tol;
  }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "4-way Gaussian Process Factor Linear(" << dof_ << ")" << std::endl;
    Base::print("", keyFormatter);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(dof_);
    ar & BOOST_SERIALIZATION_NVP(delta_t_);
  }

}; // GaussianProcessPriorLinear


} // namespace gpmp2



/// traits
namespace gtsam {
template<>
struct traits<gpmp2::GaussianProcessPriorLinear> : public Testable<
    gpmp2::GaussianProcessPriorLinear> {};
}
