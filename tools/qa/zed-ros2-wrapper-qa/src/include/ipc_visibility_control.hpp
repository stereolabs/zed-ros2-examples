// Copyright 2024 Stereolabs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VISIBILITY_CONTROL_HPP_
#define VISIBILITY_CONTROL_HPP_

/* *INDENT-OFF* */
#ifdef __cplusplus
extern "C" {
#endif
/* *INDENT-ON* */

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define IPC_COMPONENT_EXPORT __attribute__((dllexport))
#define IPC_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define IPC_COMPONENT_EXPORT __declspec(dllexport)
#define IPC_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef IPC_COMPONENT_BUILDING_DLL
#define IPC_COMPONENT_PUBLIC IPC_COMPONENT_EXPORT
#else
#define IPC_COMPONENT_PUBLIC IPC_COMPONENT_IMPORT
#endif
#define IPC_COMPONENT_PUBLIC_TYPE IPC_COMPONENT_PUBLIC
#define IPC_COMPONENT_LOCAL
#else
#define IPC_COMPONENT_EXPORT __attribute__((visibility("default")))
#define IPC_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define IPC_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define IPC_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define IPC_COMPONENT_PUBLIC
#define IPC_COMPONENT_LOCAL
#endif
#define IPC_COMPONENT_PUBLIC_TYPE
#endif

/* *INDENT-OFF* */
#ifdef __cplusplus
}
#endif
/* *INDENT-ON* */

#endif  // CVT_VISIBILITY_CONTROL_HPP_
