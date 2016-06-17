# Install headers
file(GLOB gp_headers "*.h")
install(FILES ${gp_headers} DESTINATION include/gpmp2/gp)

# Build tests
gtsamAddTestsGlob(gp "tests/*.cpp" "" ${PROJECT_NAME})
