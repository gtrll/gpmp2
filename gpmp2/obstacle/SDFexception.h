/**
 *  @file  SDFexception.h
 *  @brief custom exceptions for signed distance field
 *  @author Jing Dong
 *  @date  May 26, 2016
 **/

#pragma once

#include <stdexcept>


namespace gpmp2 {

/// query out of range exception
class SDFQueryOutOfRange : public std::runtime_error {

public:
  /// constructor
  SDFQueryOutOfRange() : std::runtime_error("Querying SDF out of range") {}
};

}


