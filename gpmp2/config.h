/**
 *  @file  config.h
 *  @brief configurations import from CMake
 *  @author Jing Dong
 *  @date  May 21, 2017
 **/

#pragma once


// Whether GPMP2 is compiled as static or DLL in windows. 
// This will be used to decide whether include __declspec(dllimport) or not in headers
/* #undef GPMP2_BUILD_STATIC_LIBRARY */


// Macros for exporting DLL symbols on Windows
// Usage example:
// In header file:
//   class GPMP2_EXPORT MyClass { ... };
//   
// Results in the following declarations:
// When included while compiling the GPMP2 library itself:
//   class __declspec(dllexport) MyClass { ... };
// When included while compiling other code against GPMP2:
//   class __declspec(dllimport) MyClass { ... };

#ifdef _WIN32
#  ifdef GPMP2_BUILD_STATIC_LIBRARY
#    define GPMP2_EXPORT
#    define GPMP2_EXTERN_EXPORT extern
#  else /* GPMP2_BUILD_STATIC_LIBRARY */
#    ifdef GPMP2_EXPORTS
#      define GPMP2_EXPORT __declspec(dllexport)
#      define GPMP2_EXTERN_EXPORT __declspec(dllexport) extern
#    else /* GPMP2_EXPORTS */
#      define GPMP2_EXPORT __declspec(dllimport)
#      define GPMP2_EXTERN_EXPORT __declspec(dllimport)
#    endif /* GPMP2_EXPORTS */
#  endif /* GPMP2_BUILD_STATIC_LIBRARY */
#else /* _WIN32 */
#  define GPMP2_EXPORT
#  define GPMP2_EXTERN_EXPORT extern
#endif
