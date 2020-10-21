#ifndef ZED_CVT_COMPONENT__VISIBILITY_CONTROL_H_
#define ZED_CVT_COMPONENT__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ZED_CVT_COMPONENT_EXPORT __attribute__ ((dllexport))
    #define ZED_CVT_COMPONENT_IMPORT __attribute__ ((dllimport))
  #else
    #define ZED_CVT_COMPONENT_EXPORT __declspec(dllexport)
    #define ZED_CVT_COMPONENT_IMPORT __declspec(dllimport)
  #endif
  #ifdef ZED_CVT_COMPONENT_BUILDING_DLL
    #define ZED_CVT_COMPONENT_PUBLIC ZED_CVT_COMPONENT_EXPORT
  #else
    #define ZED_CVT_COMPONENT_PUBLIC ZED_CVT_COMPONENT_IMPORT
  #endif
  #define ZED_CVT_COMPONENT_PUBLIC_TYPE ZED_CVT_COMPONENT_PUBLIC
  #define ZED_CVT_COMPONENT_LOCAL
#else
  #define ZED_CVT_COMPONENT_EXPORT __attribute__ ((visibility("default")))
  #define ZED_CVT_COMPONENT_IMPORT
  #if __GNUC__ >= 4
    #define ZED_CVT_COMPONENT_PUBLIC __attribute__ ((visibility("default")))
    #define ZED_CVT_COMPONENT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ZED_CVT_COMPONENT_PUBLIC
    #define ZED_CVT_COMPONENT_LOCAL
  #endif
  #define ZED_CVT_COMPONENT_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ZED_CVT_COMPONENT__VISIBILITY_CONTROL_H_
