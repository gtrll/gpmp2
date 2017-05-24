/**
 *  @file  fileUtils.h
 *  @brief utility functions wrapped in matlab
 *  @author Jing Dong
 *  @date  Jan 28, 2017
 **/

#pragma once

#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/config.h>


namespace gpmp2 {

/// read SDF from .vol file
/// the filename_pre.vol.head has the size/origin/resolution data
/// the filename_pre.vol.data contains actual data
/// return whether successfully read
GPMP2_EXPORT bool readSDFvolfile(const std::string& filename_pre, SignedDistanceField& sdf);


}


