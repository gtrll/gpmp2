# Install headers
file(GLOB planner_headers "*.h")
install(FILES ${planner_headers} DESTINATION include/gpmp2/planner)

# Build tests
gtsamAddTestsGlob(planner "tests/*.cpp" "" ${PROJECT_NAME})
