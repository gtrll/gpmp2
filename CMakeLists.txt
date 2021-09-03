cmake_minimum_required(VERSION 3.0)
enable_testing()
project(gpmp2 CXX C)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# Mac ONLY. Define Relative Path on Mac OS
if(NOT DEFINED CMAKE_MACOSX_RPATH)
  set(CMAKE_MACOSX_RPATH 0)
endif()

# version indicator
set(GPMP2_VERSION_MAJOR 0)
set(GPMP2_VERSION_MINOR 3)
set(GPMP2_VERSION_PATCH 0)
set(GPMP2_VERSION_STRING "${GPMP2_VERSION_MAJOR}.${GPMP2_VERSION_MINOR}.${GPMP2_VERSION_PATCH}")


# option: whether turn on Matlab toolbox
option(GPMP2_BUILD_STATIC_LIBRARY "whether build static library" OFF)
option(GPMP2_BUILD_MATLAB_TOOLBOX "whether build matlab toolbox, need shared lib" OFF)
option(GPMP2_BUILD_PYTHON_TOOLBOX "whether build python toolbox, need shared lib" OFF)

if(GPMP2_BUILD_STATIC_LIBRARY AND GPMP2_BUILD_MATLAB_TOOLBOX)
  message(FATAL_ERROR "matlab toolbox needs shared lib")
endif()


# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)


# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost - same requirement as gtsam
find_package(Boost 1.50 REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Boost COMPONENTS serialization REQUIRED)

include_directories(${Boost_INCLUDE_DIR})


# Generate and install config and dllexport files
configure_file("gpmp2/config.h.in" "gpmp2/config.h")
list(APPEND gpmp2_srcs "${PROJECT_BINARY_DIR}/gpmp2/config.h")
include_directories(BEFORE ${PROJECT_BINARY_DIR}) # So we can include generated config header files
install(FILES "${PROJECT_BINARY_DIR}/gpmp2/config.h" DESTINATION include/gpmp2)


# include current source folder, at the very beginning
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR})

# Process source subdirs
add_subdirectory(gpmp2)

# Wrapping to MATLAB
if(GPMP2_BUILD_MATLAB_TOOLBOX)
  # wrap
  include(GtsamMatlabWrap)
  wrap_and_install_library(gpmp2.h ${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}" "")
  
  # install matlab functions and scripts
  add_subdirectory(matlab)
endif()


# Wrapping to Python
if(GPMP2_BUILD_PYTHON_TOOLBOX)
  include_directories(${GTSAM_DIR}/cython)
  include_directories(/usr/local/cython)
  include(GtsamCythonWrap)
  include_directories(${GTSAM_EIGENCY_INSTALL_PATH})

  wrap_and_install_library_cython("gpmp2.h"
                                  "from gtsam.gtsam cimport *" # extra import of gtsam/gtsam.pxd Cython header
                                  "${CMAKE_INSTALL_PREFIX}/cython" # install path
                                  gpmp2  # library to link with
                                  "gtsam"  # dependencies which need to be built before wrapping
                                  )
  add_definitions(-DBOOST_OPTIONAL_ALLOW_BINDING_TO_RVALUES -DBOOST_OPTIONAL_CONFIG_ALLOW_BINDING_TO_RVALUES)
endif()

# Install config and export files
GtsamMakeConfigFile(GPMP2)
export(TARGETS ${GPMP2_EXPORTED_TARGETS} FILE gpmp2-exports.cmake)
