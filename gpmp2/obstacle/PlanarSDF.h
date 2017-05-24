/**
 *  @file  PlanarSDF.h
 *  @brief util functions for planar signed distance field
 *  @author Jing Dong
 *  @date  May 7, 2016
 **/

#pragma once

#include <gpmp2/obstacle/SDFexception.h>
#include <gpmp2/config.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>

#include <iostream>


namespace gpmp2 {

/**
 * Signed distance field use Matrix as data type
 * Matrix represent the X (col) & Y (row) dimension
 */
class GPMP2_EXPORT PlanarSDF {

public:
  // index and float_index is <row, col>
  typedef boost::tuple<size_t, size_t> index;
  typedef boost::tuple<double, double> float_index;
  typedef boost::shared_ptr<PlanarSDF> shared_ptr;

private:
  gtsam::Point2 origin_;
  // geometry setting of signed distance field
  size_t field_rows_, field_cols_;
  double cell_size_;
  gtsam::Matrix data_;

public:
  /// constructor
  PlanarSDF() : field_rows_(0), field_cols_(0), cell_size_(0.0) {}

  /// constructor with data
  PlanarSDF(const gtsam::Point2& origin, double cell_size, const gtsam::Matrix& data) :
      origin_(origin), field_rows_(data.rows()), field_cols_(data.cols()),
      cell_size_(cell_size), data_(data) {}

  ~PlanarSDF() {}


  /// give a point, search for signed distance field and (optional) gradient
  /// return signed distance
  inline double getSignedDistance(const gtsam::Point2& point) const {
    const float_index pidx = convertPoint2toCell(point);
    return signed_distance(pidx);
  }

  inline double getSignedDistance(const gtsam::Point2& point, gtsam::Vector2& g) const {
    const float_index pidx = convertPoint2toCell(point);
    const gtsam::Vector2 g_idx = gradient(pidx);
    // convert gradient of index to gradient of metric unit
    g = gtsam::Vector2(g_idx(1), g_idx(0)) / cell_size_;
    return signed_distance(pidx);
  }


  /// convert between point and cell corrdinate
  inline float_index convertPoint2toCell(const gtsam::Point2& point) const {
    // check point range
    if (point.x() < origin_.x() || point.x() > (origin_.x() + (field_cols_-1.0)*cell_size_) ||
        point.y() < origin_.y() || point.y() > (origin_.y() + (field_rows_-1.0)*cell_size_)) {
        
      throw SDFQueryOutOfRange();
    }

    const double col = (point.x() - origin_.x()) / cell_size_;
    const double row = (point.y() - origin_.y()) / cell_size_;
    return boost::make_tuple(row, col);
  }

  inline gtsam::Point2 convertCelltoPoint2(const float_index& cell) const {
    return origin_ + gtsam::Point2(
        cell.get<1>() * cell_size_,
        cell.get<0>() * cell_size_);
  }


  /// bilinear interpolation
  inline double signed_distance(const float_index& idx) const {
    const double lr = floor(idx.get<0>()), lc = floor(idx.get<1>());
    const double hr = lr + 1.0, hc = lc + 1.0;
    const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc),
        hri = static_cast<size_t>(hr), hci = static_cast<size_t>(hc);
    return
        (hr-idx.get<0>())*(hc-idx.get<1>())*signed_distance(lri, lci) +
        (idx.get<0>()-lr)*(hc-idx.get<1>())*signed_distance(hri, lci) +
        (hr-idx.get<0>())*(idx.get<1>()-lc)*signed_distance(lri, hci) +
        (idx.get<0>()-lr)*(idx.get<1>()-lc)*signed_distance(hri, hci);
  }

  /// gradient operator for bilinear interpolation
  /// gradient regrads to float_index
  /// not numerical differentiable at index point
  inline gtsam::Vector2 gradient(const float_index& idx) const {
    const double lr = floor(idx.get<0>()), lc = floor(idx.get<1>());
    const double hr = lr + 1.0, hc = lc + 1.0;
    const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc),
        hri = static_cast<size_t>(hr), hci = static_cast<size_t>(hc);
    return gtsam::Vector2(
        (hc-idx.get<1>()) * (signed_distance(hri, lci)-signed_distance(lri, lci)) +
        (idx.get<1>()-lc) * (signed_distance(hri, hci)-signed_distance(lri, hci)),

        (hr-idx.get<0>()) * (signed_distance(lri, hci)-signed_distance(lri, lci)) +
        (idx.get<0>()-lr) * (signed_distance(hri, hci)-signed_distance(hri, lci)));
  }

  /// access
  inline double signed_distance(size_t r, size_t c) const {
    return data_(r, c);
  }

  const gtsam::Point2& origin() const { return origin_; }
  size_t x_count() const { return field_cols_; }
  size_t y_count() const { return field_rows_; }
  double cell_size() const { return cell_size_; }
  const gtsam::Matrix& raw_data() const { return data_; }

  /// print
  void print(const std::string& str = "") const {
    std::cout << str;
    std::cout << "field origin:     "; origin_.print();
    std::cout << "field resolution: " << cell_size_ << std::endl;
    std::cout << "field size:       " << field_cols_ << " x "
        << field_rows_ << std::endl;
  }

};

}


