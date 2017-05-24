Installation CMake settings
===================================================
GPMP2 can be compiled by steps in README if Boost and GTSAM installed at default locations, and the default GPMP2 install location is ```/usr/local``` on Linux and ```C:\Program Files\gpmp2``` on Windows. If Boost and GTSAM are installed at non-default locations, or if you want to install GPMP2 to a non-default location, please update following CMake entries. 


| Entry         | Value         | Comment  |
| ------------- |:--------------|:---------|
| ```CMAKE_INSTALL_PREFIX``` | ```${gpmp2_installed_path}``` |  |
| ```BOOST_INCLUDEDIR``` | ```${boost_include_installed_path}``` |  |
| ```BOOST_LIBRARYDIR``` | ```${boost_library_installed_path}``` |  |
| ```Boost_USE_STATIC_LIBS``` | ON/OFF | Set to ON if use statlic Boost libraries |
| ```GTSAM_DIR``` | ```${gtsam_installed_path}/CMake``` |  |
| ```GTSAMCMakeTools_DIR``` | ```${gtsam_installed_path}/CMake/GTSAMCMakeTools``` |  |
