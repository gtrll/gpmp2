/**
 * @file ProductDynamicLieGroup.h
 * @date Oct 4, 2016
 * @author Frank Dellaert
 * @author Jing Dong
 * @brief Group product of two Lie Groups, customize for dynamic size types
 */

#pragma once

#include <gpmp2/geometry/utilsDynamic.h>

#include <gtsam/base/Lie.h>
#include <utility>  // pair


namespace gpmp2 {


/// Template to construct the product Lie group of two other Lie groups
/// Assumes Lie group structure for G and H, at least one is dynamic size types
template <typename G, typename H>
class ProductDynamicLieGroup: public std::pair<G, H> {

private:
  BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<G>));
  BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<H>));
  typedef std::pair<G, H> Base;

protected:
  // static dimensions
  enum {dimension1 = gtsam::traits<G>::dimension};
  enum {dimension2 = gtsam::traits<H>::dimension};

public:
  /// Default constructor: just call subgroup default constructors
  ProductDynamicLieGroup() : Base(G(), H()) {}

  // Construct from two subgroup elements
  ProductDynamicLieGroup(const G& g, const H& h) : Base(g,h) {}

  // Construct from base
  ProductDynamicLieGroup(const Base& base) : Base(base) {}

  virtual ~ProductDynamicLieGroup() {}


  /// @name Group
  /// @{
  static ProductDynamicLieGroup identity() {
    throw std::runtime_error("[ProductDynamicLieGroup] ERROR: no static identity implemented");
    return ProductDynamicLieGroup();
  }

  ProductDynamicLieGroup operator*(const ProductDynamicLieGroup& other) const {
    return ProductDynamicLieGroup(gtsam::traits<G>::Compose(this->first, other.first),
        gtsam::traits<H>::Compose(this->second, other.second));
  }
  ProductDynamicLieGroup inverse() const {
    return ProductDynamicLieGroup(gtsam::traits<G>::Inverse(this->first),
        gtsam::traits<H>::Inverse(this->second));
  }
  ProductDynamicLieGroup compose(const ProductDynamicLieGroup& g) const {
    return (*this) * g;
  }
  ProductDynamicLieGroup between(const ProductDynamicLieGroup& g) const {
    return this->inverse() * g;
  }
  /// @}

  /// @name Manifold
  /// @{
  enum { dimension = Eigen::Dynamic };
  inline static size_t Dim() { return dimension; }

  // dynamic dimensions
  inline size_t dim1() const { return DimensionUtils<G, dimension1>::getDimension(this->first); }
  inline size_t dim2() const { return DimensionUtils<H, dimension2>::getDimension(this->second); }
  inline size_t dim() const { return dim1() + dim2(); }

  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef gtsam::OptionalJacobian<dimension, dimension> ChartJacobian;

  ProductDynamicLieGroup retract(const TangentVector& v, //
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) const {
    if (H1||H2) throw std::runtime_error("ProductLieGroup::retract derivatives not implemented yet");
    G g = gtsam::traits<G>::Retract(this->first, v.head(dim1()));
    H h = gtsam::traits<H>::Retract(this->second, v.tail(dim2()));
    return ProductDynamicLieGroup(g,h);
  }

  TangentVector localCoordinates(const ProductDynamicLieGroup& g, //
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) const {
    if (H1||H2) throw std::runtime_error("ProductLieGroup::localCoordinates derivatives not implemented yet");
    typename gtsam::traits<G>::TangentVector v1 = gtsam::traits<G>::Local(this->first, g.first);
    typename gtsam::traits<H>::TangentVector v2 = gtsam::traits<H>::Local(this->second, g.second);
    TangentVector v(dim());
    v << v1, v2;
    return v;
  }
  /// @}

  /// @name Lie Group
  /// @{
protected:
  typedef Eigen::Matrix<double, dimension, dimension> Jacobian;
  typedef Eigen::Matrix<double, dimension1, dimension1> Jacobian1;
  typedef Eigen::Matrix<double, dimension2, dimension2> Jacobian2;

public:
  ProductDynamicLieGroup compose(const ProductDynamicLieGroup& other, ChartJacobian H1,
      ChartJacobian H2 = boost::none) const {
    if (H1 || H2) {
      Jacobian1 D_g_first; Jacobian2 D_h_second;
      G g = gtsam::traits<G>::Compose(this->first, other.first, D_g_first);
      H h = gtsam::traits<H>::Compose(this->second, other.second, D_h_second);
      if (H1) {
        H1->setZero(dim() ,dim());
        H1->topLeftCorner(dim1(), dim1()) = D_g_first;
        H1->bottomRightCorner(dim2(), dim2()) = D_h_second;
      }
      if (H2) *H2 = Jacobian::Identity(dim() ,dim());
      return ProductDynamicLieGroup(g,h);

    } else {
      G g = gtsam::traits<G>::Compose(this->first, other.first);
      H h = gtsam::traits<H>::Compose(this->second, other.second);
      return ProductDynamicLieGroup(g,h);
    }
  }

  ProductDynamicLieGroup between(const ProductDynamicLieGroup& other, ChartJacobian H1,
      ChartJacobian H2 = boost::none) const {
    if (H1 || H2) {
      Jacobian1 D_g_first; Jacobian2 D_h_second;
      G g = gtsam::traits<G>::Between(this->first,other.first, D_g_first);
      H h = gtsam::traits<H>::Between(this->second,other.second, D_h_second);
      if (H1) {
        H1->setZero(dim() ,dim());
        H1->topLeftCorner(dim1(), dim1()) = D_g_first;
        H1->bottomRightCorner(dim2(), dim2()) = D_h_second;
      }
      if (H2) *H2 = Jacobian::Identity(dim() ,dim());
      return ProductDynamicLieGroup(g,h);

    } else {
      G g = gtsam::traits<G>::Between(this->first,other.first);
      H h = gtsam::traits<H>::Between(this->second,other.second);
      return ProductDynamicLieGroup(g,h);
    }
  }

  ProductDynamicLieGroup inverse(ChartJacobian D) const {
    if (D) {
      Jacobian1 D_g_first; Jacobian2 D_h_second;
      G g = gtsam::traits<G>::Inverse(this->first, D_g_first);
      H h = gtsam::traits<H>::Inverse(this->second, D_h_second);
      D->setZero(dim() ,dim());
      D->topLeftCorner(dim1(), dim1()) = D_g_first;
      D->bottomRightCorner(dim2(), dim2()) = D_h_second;
      return ProductDynamicLieGroup(g,h);
    } else {
      G g = gtsam::traits<G>::Inverse(this->first);
      H h = gtsam::traits<H>::Inverse(this->second);
      return ProductDynamicLieGroup(g,h);
    }
  }

  static ProductDynamicLieGroup Expmap(const TangentVector& v, ChartJacobian Hv = boost::none) {

    // figure out total dim from v input
    const size_t dim_v = v.size();
    // figure out dim for 1 and 2
    size_t dim1_v, dim2_v;
    if (dimension1 != Eigen::Dynamic) {
      dim1_v = dimension1;
      dim2_v = dim_v - dimension1;
    } else if (dimension2 != Eigen::Dynamic) {
      dim1_v = dim_v - dimension2;
      dim2_v = dimension2;
    } else {
      throw std::runtime_error("[ProductDynamicLieGroup] ERROR: not implement both dynamic types in Expmap");
    }

    if (Hv) {
      Jacobian1 D_g_first; Jacobian2 D_h_second;
      G g = gtsam::traits<G>::Expmap(v.head(dim1_v), D_g_first);
      H h = gtsam::traits<H>::Expmap(v.tail(dim2_v), D_h_second);
      Hv->setZero(dim_v, dim_v);
      Hv->template topLeftCorner<dimension1,dimension1>(dim1_v, dim1_v) = D_g_first;
      Hv->template bottomRightCorner<dimension2,dimension2>(dim2_v, dim2_v) = D_h_second;
      return ProductDynamicLieGroup(g,h);

    } else {
      return ProductDynamicLieGroup(gtsam::traits<G>::Expmap(v.head(dim1_v)),
          gtsam::traits<H>::Expmap(v.tail(dim2_v)));
    }
  }

  static TangentVector Logmap(const ProductDynamicLieGroup& p, ChartJacobian Hp = boost::none) {
    if (Hp) {
      Jacobian1 D_g_first; Jacobian2 D_h_second;
      typename gtsam::traits<G>::TangentVector v1 = gtsam::traits<G>::Logmap(p.first, D_g_first);
      typename gtsam::traits<H>::TangentVector v2 = gtsam::traits<H>::Logmap(p.second, D_h_second);
      TangentVector v(p.dim());
      v << v1, v2;
      Hp->setZero(p.dim(), p.dim());
      Hp->topLeftCorner(p.dim1(), p.dim1()) = D_g_first;
      Hp->bottomRightCorner(p.dim2(), p.dim2()) = D_h_second;
      return v;

    } else {
      TangentVector v(p.dim());
      v << gtsam::traits<G>::Logmap(p.first), gtsam::traits<H>::Logmap(p.second);
      return v;
    }
  }

  ProductDynamicLieGroup expmap(const TangentVector& v) const {
    return compose(ProductDynamicLieGroup::Expmap(v));
  }
  TangentVector logmap(const ProductDynamicLieGroup& g) const {
    return ProductDynamicLieGroup::Logmap(between(g));
  }
  /// @}

};

} // namespace gpmp2


