# We split the library in to separate subfolders, each containing
# tests, timing, and an optional convenience library.
# The following variable is the master list of subdirs to add
set(gpmp2_subdirs 
    geometry
    gp 
    kinematics
    dynamics
    obstacle
    planner
    utils
)
set(gpmp2_srcs)

# files want to be excluded
set(excluded_sources "")

# Library sources
foreach(subdir ${gpmp2_subdirs})
  file(GLOB subdir_srcs "${subdir}/*.cpp" "${subdir}/*.h")
  list(REMOVE_ITEM subdir_srcs "${excluded_sources}")

  file(GLOB subdir_test_files "${subdir}/tests/*")
  
  list(APPEND gpmp2_srcs ${subdir_srcs})
  message(STATUS "Building Module: ${subdir}")

  # local and tests
  add_subdirectory(${subdir})
endforeach(subdir)


# Versions
set(gpmp2_version   ${GPMP2_VERSION_MAJOR}.${GPMP2_VERSION_MINOR}.${GPMP2_VERSION_PATCH})
set(gpmp2_soversion ${GPMP2_VERSION_MAJOR})
message(STATUS "GPMP2 Version: ${gpmp2_version}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")


# build shared lib
if(GPMP2_BUILD_STATIC_LIBRARY)
  # static
  message(STATUS "Build static library")
  add_library(${PROJECT_NAME} STATIC ${gpmp2_srcs})
  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${GTSAM_LIBRARIES})
  set_target_properties(gpmp2 PROPERTIES
      OUTPUT_NAME         gpmp2
      CLEAN_DIRECT_OUTPUT 1
      VERSION             ${gpmp2_version}
      SOVERSION           ${gpmp2_soversion})
  if(WIN32) # Add 'lib' prefix to static library to avoid filename collision with shared library
    set_target_properties(gpmp2 PROPERTIES
      PREFIX "lib")
  endif()
  install(TARGETS ${PROJECT_NAME} EXPORT gpmp2-exports ARCHIVE DESTINATION lib)

else()
  # shared
  message(STATUS "Build shared library")
  add_library(${PROJECT_NAME} SHARED ${gpmp2_srcs})
  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${GTSAM_LIBRARIES})
  set_target_properties(gpmp2 PROPERTIES
      OUTPUT_NAME         gpmp2
      CLEAN_DIRECT_OUTPUT 1
      VERSION             ${gpmp2_version}
      SOVERSION           ${gpmp2_soversion})
  if(WIN32)
    set_target_properties(gpmp2 PROPERTIES
        PREFIX ""
        DEFINE_SYMBOL GPMP2_EXPORTS
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
  endif()
  install(TARGETS ${PROJECT_NAME} EXPORT gpmp2-exports LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

endif()

list(APPEND GPMP2_EXPORTED_TARGETS gpmp2)
set(GPMP2_EXPORTED_TARGETS "${GPMP2_EXPORTED_TARGETS}" PARENT_SCOPE)

