# We split the library in to separate subfolders, each containing
# tests, timing, and an optional convenience library.
# The following variable is the master list of subdirs to add
set(gpmp2_subdirs 
    gp 
    kinematics
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


# build shared lib
add_library(${PROJECT_NAME} SHARED ${gpmp2_srcs})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${GTSAM_LIBRARIES})

# Install library
install(TARGETS ${PROJECT_NAME} EXPORT gpmp2-exports LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

