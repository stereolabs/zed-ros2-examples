#ifndef TOPIC_BENCHMARK__VISIBILITY_CONTROL_H_
#define TOPIC_BENCHMARK__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TOPIC_BENCHMARK_EXPORT __attribute__ ((dllexport))
    #define TOPIC_BENCHMARK_IMPORT __attribute__ ((dllimport))
  #else
    #define TOPIC_BENCHMARK_EXPORT __declspec(dllexport)
    #define TOPIC_BENCHMARK_IMPORT __declspec(dllimport)
  #endif
  #ifdef TOPIC_BENCHMARK_BUILDING_DLL
    #define TOPIC_BENCHMARK_PUBLIC TOPIC_BENCHMARK_EXPORT
  #else
    #define TOPIC_BENCHMARK_PUBLIC TOPIC_BENCHMARK_IMPORT
  #endif
  #define TOPIC_BENCHMARK_PUBLIC_TYPE TOPIC_BENCHMARK_PUBLIC
  #define TOPIC_BENCHMARK_LOCAL
#else
  #define TOPIC_BENCHMARK_EXPORT __attribute__ ((visibility("default")))
  #define TOPIC_BENCHMARK_IMPORT
  #if __GNUC__ >= 4
    #define TOPIC_BENCHMARK_PUBLIC __attribute__ ((visibility("default")))
    #define TOPIC_BENCHMARK_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TOPIC_BENCHMARK_PUBLIC
    #define TOPIC_BENCHMARK_LOCAL
  #endif
  #define TOPIC_BENCHMARK_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // TOPIC_BENCHMARK__VISIBILITY_CONTROL_H_
