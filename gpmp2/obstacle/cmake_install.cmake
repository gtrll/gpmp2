# Install script for directory: /home/matt/Documents/gpmp2/gpmp2/obstacle

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gpmp2/obstacle" TYPE FILE FILES
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleCost.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactor-inl.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactor.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorArm.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorGP-inl.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorGP.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorGPArm.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorGPPointRobot.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2Mobile2Arms.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileArm.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileBase.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorPose2Mobile2Arms.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileArm.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileBase.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleSDFFactor-inl.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleSDFFactor.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleSDFFactorArm.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleSDFFactorGP-inl.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleSDFFactorGP.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleSDFFactorGPArm.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleSDFFactorGPPose2Mobile2Arms.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileArm.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileBase.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileVetLin2Arms.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileVetLinArm.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleSDFFactorPose2Mobile2Arms.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleSDFFactorPose2MobileArm.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleSDFFactorPose2MobileBase.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleSDFFactorPose2MobileVetLin2Arms.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/ObstacleSDFFactorPose2MobileVetLinArm.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/PlanarSDF.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/SDFexception.h"
    "/home/matt/Documents/gpmp2/gpmp2/obstacle/SignedDistanceField.h"
    )
endif()

