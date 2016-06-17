# Examples
message(STATUS "Installing Matlab toolbox scripts")

install_matlab_scripts("${CMAKE_CURRENT_SOURCE_DIR}/" "+gpmp2/*.m")
