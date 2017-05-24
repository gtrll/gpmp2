/**
 *  @file  SignedDistanceField.h
 *  @brief util functions for signed distance field
 *  @author Jing Dong, Mustafa Mukadam
 *  @date  Dec 2, 2015
 **/

#pragma once

#include <gpmp2/obstacle/SDFexception.h>
#include <gpmp2/config.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>

#include <boost/tuple/tuple.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>


namespace gpmp2 {

/**
 * Signed distance field use vector<Matrix> as data type
 * Matrix represent the X (col) & Y (row) dimension, and vector is used to Z
 */
class GPMP2_EXPORT SignedDistanceField {

public:
  // index and float_index is <row, col, z>
  typedef boost::tuple<size_t, size_t, size_t> index;
  typedef boost::tuple<double, double, double> float_index;
  typedef boost::shared_ptr<SignedDistanceField> shared_ptr;

private:
  gtsam::Point3 origin_;
  // geometry setting of signed distance field
  size_t field_rows_, field_cols_, field_z_;
  double cell_size_;
  // sdf data
  std::vector<gtsam::Matrix> data_;

public:
  /// constructor
  SignedDistanceField() {}

  /// constructor with all data
  SignedDistanceField(const gtsam::Point3& origin, double cell_size, const std::vector<gtsam::Matrix>& data) :
      origin_(origin), field_rows_(data[0].rows()), field_cols_(data[0].cols()),
      field_z_(data.size()), cell_size_(cell_size), data_(data) {}

  /// constructor with no data, insert the data later
  /// used by matlab wrapper
  SignedDistanceField(const gtsam::Point3& origin, double cell_size, size_t field_rows,
      size_t field_cols, size_t field_z) :
      origin_(origin), field_rows_(field_rows), field_cols_(field_cols),
      field_z_(field_z), cell_size_(cell_size), data_(std::vector<gtsam::Matrix>(field_z)) {}

  ~SignedDistanceField() {}


  /// insert data matrix to each layer of sdf
  /// @param z_idx the z index of 3-D sdf
  /// @param field_layer matrix of each slice of 3-D sdf, Matrix represent the X (col) & Y (row)
  void initFieldData(size_t z_idx, const gtsam::Matrix& field_layer) {
    if (z_idx >= field_z_)
      throw std::runtime_error("[SignedDistanceField] matrix layer out of index");
    data_[z_idx] = field_layer;
  }

  /// give a point, search for signed distance field and (optional) gradient
  /// @param point query position
  /// @return signed distance
  inline double getSignedDistance(const gtsam::Point3& point) const {
    const float_index pidx = convertPoint3toCell(point);
    return signed_distance(pidx);
  }

  /// give a point, search for signed distance field and (optional) gradient
  /// @param point query position
  /// @param g returned gradient reference
  /// @return signed distance
  inline double getSignedDistance(const gtsam::Point3& point, gtsam::Vector3& g) const {
    const float_index pidx = convertPoint3toCell(point);
    const gtsam::Vector3 g_idx = gradient(pidx);
    // convert gradient of index to gradient of metric unit
    g = gtsam::Vector3(g_idx(1), g_idx(0), g_idx(2)) / cell_size_;
    return signed_distance(pidx);
  }


  /// convert between point and cell corrdinate
  inline float_index convertPoint3toCell(const gtsam::Point3& point) const {
    // check point range
    if (point.x() < origin_.x() || point.x() > (origin_.x() + (field_cols_-1.0)*cell_size_) ||
        point.y() < origin_.y() || point.y() > (origin_.y() + (field_rows_-1.0)*cell_size_) ||
        point.z() < origin_.z() || point.z() > (origin_.z() + (field_z_-1.0)*cell_size_)) {
        
      throw SDFQueryOutOfRange();
    }

    const double col = (point.x() - origin_.x()) / cell_size_;
    const double row = (point.y() - origin_.y()) / cell_size_;
    const double z   = (point.z() - origin_.z()) / cell_size_;
    return boost::make_tuple(row, col, z);
  }

  inline gtsam::Point3 convertCelltoPoint3(const float_index& cell) const {
    return origin_ + gtsam::Point3(
        cell.get<1>() * cell_size_,
        cell.get<0>() * cell_size_,
        cell.get<2>() * cell_size_);
  }


  /// tri-linear interpolation
  inline double signed_distance(const float_index& idx) const {
    const double lr = floor(idx.get<0>()), lc = floor(idx.get<1>()), lz = floor(idx.get<2>());
    const double hr = lr + 1.0, hc = lc + 1.0, hz = lz + 1.0;
    const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc), lzi = static_cast<size_t>(lz),
        hri = static_cast<size_t>(hr), hci = static_cast<size_t>(hc), hzi = static_cast<size_t>(hz);
    return
        (hr-idx.get<0>())*(hc-idx.get<1>())*(hz-idx.get<2>())*signed_distance(lri, lci, lzi) +
        (idx.get<0>()-lr)*(hc-idx.get<1>())*(hz-idx.get<2>())*signed_distance(hri, lci, lzi) +
        (hr-idx.get<0>())*(idx.get<1>()-lc)*(hz-idx.get<2>())*signed_distance(lri, hci, lzi) +
        (idx.get<0>()-lr)*(idx.get<1>()-lc)*(hz-idx.get<2>())*signed_distance(hri, hci, lzi) +
        (hr-idx.get<0>())*(hc-idx.get<1>())*(idx.get<2>()-lz)*signed_distance(lri, lci, hzi) +
        (idx.get<0>()-lr)*(hc-idx.get<1>())*(idx.get<2>()-lz)*signed_distance(hri, lci, hzi) +
        (hr-idx.get<0>())*(idx.get<1>()-lc)*(idx.get<2>()-lz)*signed_distance(lri, hci, hzi) +
        (idx.get<0>()-lr)*(idx.get<1>()-lc)*(idx.get<2>()-lz)*signed_distance(hri, hci, hzi);
  }

  /// gradient operator for tri-linear interpolation
  /// gradient regrads to float_index
  /// not differentiable at index point
  inline gtsam::Vector3 gradient(const float_index& idx) const {
    const double lr = floor(idx.get<0>()), lc = floor(idx.get<1>()), lz = floor(idx.get<2>());
    const double hr = lr + 1.0, hc = lc + 1.0, hz = lz + 1.0;
    const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc), lzi = static_cast<size_t>(lz),
        hri = static_cast<size_t>(hr), hci = static_cast<size_t>(hc), hzi = static_cast<size_t>(hz);
    return gtsam::Vector3(
        (hc-idx.get<1>())*(hz-idx.get<2>()) * (signed_distance(hri, lci, lzi)-signed_distance(lri, lci, lzi)) +
        (idx.get<1>()-lc)*(hz-idx.get<2>()) * (signed_distance(hri, hci, lzi)-signed_distance(lri, hci, lzi)) +
        (hc-idx.get<1>())*(idx.get<2>()-lz) * (signed_distance(hri, lci, hzi)-signed_distance(lri, lci, hzi)) +
        (idx.get<1>()-lc)*(idx.get<2>()-lz) * (signed_distance(hri, hci, hzi)-signed_distance(lri, hci, hzi)),

        (hr-idx.get<0>())*(hz-idx.get<2>()) * (signed_distance(lri, hci, lzi)-signed_distance(lri, lci, lzi)) +
        (idx.get<0>()-lr)*(hz-idx.get<2>()) * (signed_distance(hri, hci, lzi)-signed_distance(hri, lci, lzi)) +
        (hr-idx.get<0>())*(idx.get<2>()-lz) * (signed_distance(lri, hci, hzi)-signed_distance(lri, lci, hzi)) +
        (idx.get<0>()-lr)*(idx.get<2>()-lz) * (signed_distance(hri, hci, hzi)-signed_distance(hri, lci, hzi)),

        (hr-idx.get<0>())*(hc-idx.get<1>()) * (signed_distance(lri, lci, hzi)-signed_distance(lri, lci, lzi)) +
        (idx.get<0>()-lr)*(hc-idx.get<1>()) * (signed_distance(hri, lci, hzi)-signed_distance(hri, lci, lzi)) +
        (hr-idx.get<0>())*(idx.get<1>()-lc) * (signed_distance(lri, hci, hzi)-signed_distance(lri, hci, lzi)) +
        (idx.get<0>()-lr)*(idx.get<1>()-lc) * (signed_distance(hri, hci, hzi)-signed_distance(hri, hci, lzi))
    );
  }

  /// access
  inline double signed_distance(size_t r, size_t c, size_t z) const {
    return data_[z](r, c);
  }

  const gtsam::Point3& origin() const { return origin_; }
  size_t x_count() const { return field_cols_; }
  size_t y_count() const { return field_rows_; }
  size_t z_count() const { return field_z_; }
  double cell_size() const { return cell_size_; }
  const std::vector<gtsam::Matrix>& raw_data() const { return data_; }

  /// print
  void print(const std::string& str = "") const {
    std::cout << str;
    std::cout << "field origin:     "; origin_.print();
    std::cout << "field resolution: " << cell_size_ << std::endl;
    std::cout << "field size:       " << field_cols_ << " x "
        << field_rows_ << " x " << field_z_ << std::endl;
  }

  /// save to file
  void saveSDF(const std::string filename);

  /// load from file
  void loadSDF(const std::string filename);


private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /* version */) {
    ar & BOOST_SERIALIZATION_NVP(origin_);
    ar & BOOST_SERIALIZATION_NVP(field_rows_);
    ar & BOOST_SERIALIZATION_NVP(field_cols_);
    ar & BOOST_SERIALIZATION_NVP(field_z_);
    ar & BOOST_SERIALIZATION_NVP(cell_size_);
    ar & BOOST_SERIALIZATION_NVP(data_);
  }

};

}


