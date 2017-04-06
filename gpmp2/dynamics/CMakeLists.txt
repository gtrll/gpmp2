# Install headers
file(GLOB dynamics_headers "*.h")
install(FILES ${dynamics_headers} DESTINATION include/gpmp2/dynamics)

# Build tests
gtsamAddTestsGlob(dynamics "tests/*.cpp" "" ${PROJECT_NAME})
