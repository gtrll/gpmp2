# Install headers
file(GLOB obstacle_headers "*.h")
install(FILES ${obstacle_headers} DESTINATION include/gpmp2/obstacle)

# Build tests
gtsamAddTestsGlob(obstacle "tests/*.cpp" "" ${PROJECT_NAME})
