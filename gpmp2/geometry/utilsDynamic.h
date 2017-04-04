/**
 * @file utilsDynamic.h
 * @date Oct 4, 2016
 * @author Jing Dong
 * @brief dynamic size utility
 */

#pragma once

#include <Eigen/Core>


namespace gpmp2 {

/// get dimension from fixed size type
template <typename T, int N>
struct DimensionUtils {
  static inline int getDimension(const T&) {
    return N;
  }
};

/// template specialization get dimension from dynamic size type
template <typename T>
struct DimensionUtils<T, Eigen::Dynamic> {
  static inline int getDimension(const T& m) {
    return m.dim();
  }
};

} // namespace gpmp2


