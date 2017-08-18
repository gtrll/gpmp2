Installation guide
=====

CMake settings
-----

GPMP2 can be compiled by steps in README if Boost and GTSAM installed at default locations, and the default GPMP2 install location is ```/usr/local``` on Linux and ```C:\Program Files\gpmp2``` on Windows. If Boost and GTSAM are installed at non-default locations, or if you want to install GPMP2 to a non-default location, please update following CMake entries. 


| Entry         | Value         | Comment  |
| ------------- |:--------------|:---------|
| ```CMAKE_INSTALL_PREFIX``` | ```${gpmp2_installed_path}``` |  |
| ```BOOST_INCLUDEDIR``` | ```${boost_include_installed_path}``` |  |
| ```BOOST_LIBRARYDIR``` | ```${boost_library_installed_path}``` |  |
| ```Boost_USE_STATIC_LIBS``` | ON/OFF | Set to ON if use statlic Boost libraries |
| ```GTSAM_DIR``` | ```${gtsam_installed_path}/CMake``` |  |
| ```GTSAMCMakeTools_DIR``` | ```${gtsam_installed_path}/CMake/GTSAMCMakeTools``` |  |

Troubleshooting
-----

### Ubuntu

* If you compile Matlab toolbox and everything compiles smoothly, but when you run any of the Matlab script, you get following error messages in Matlab
```
Invalid MEX-file '/usr/local/gtsam_toolbox/gtsam_wrapper.mexa64':
Missing symbol 'mexAtExit' required by '/usr/local/gtsam_toolbox/gtsam_wrapper.mexa64'
Missing symbol 'mexCallMATLABWithObject' required by '/usr/local/gtsam_toolbox/gtsam_wrapper.mexa64'
...
```
you can run following bash line
```bash
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libprotobuf.so.9
```
before you run Matlab, or write this line in your `$HOME/.bashrc` so you don't have to type everytime before start Matlab. This mainly happens if you have GCC >= 5 and newer version Matlab like R2017a.


