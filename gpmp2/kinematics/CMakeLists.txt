# Install headers
file(GLOB kinematics_headers "*.h")
install(FILES ${kinematics_headers} DESTINATION include/gpmp2/kinematics)

# Build tests
gtsamAddTestsGlob(kinematics "tests/*.cpp" "" ${PROJECT_NAME})
