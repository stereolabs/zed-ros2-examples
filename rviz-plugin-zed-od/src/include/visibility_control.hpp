#ifndef ZED_OD_PLUGIN__VISIBILITY_CONTROL_H_
#define ZED_OD_PLUGIN__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ZED_OD_PLUGIN_EXPORT __attribute__ ((dllexport))
    #define ZED_OD_PLUGIN_IMPORT __attribute__ ((dllimport))
  #else
    #define ZED_OD_PLUGIN_EXPORT __declspec(dllexport)
    #define ZED_OD_PLUGIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef ZED_OD_PLUGIN_BUILDING_LIBRARY
    #define ZED_OD_PLUGIN_PUBLIC ZED_OD_PLUGIN_EXPORT
  #else
    #define ZED_OD_PLUGIN_PUBLIC ZED_OD_PLUGIN_IMPORT
  #endif
  #define ZED_OD_PLUGIN_PUBLIC_TYPE ZED_OD_PLUGIN_PUBLIC
  #define ZED_OD_PLUGIN_LOCAL
#else
  #define ZED_OD_PLUGIN_EXPORT __attribute__ ((visibility("default")))
  #define ZED_OD_PLUGIN_IMPORT
  #if __GNUC__ >= 4
    #define ZED_OD_PLUGIN_PUBLIC __attribute__ ((visibility("default")))
    #define ZED_OD_PLUGIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ZED_OD_PLUGIN_PUBLIC
    #define ZED_OD_PLUGIN_LOCAL
  #endif
  #define ZED_OD_PLUGIN_PUBLIC_TYPE
#endif

#endif  // ZED_OD_PLUGIN__VISIBILITY_CONTROL_H_
