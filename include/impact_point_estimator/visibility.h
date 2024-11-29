// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IMPACT_POINT_ESTIMATOR__VISIBILITY_H_
#define IMPACT_POINT_ESTIMATOR__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define IMPACT_POINT_ESTIMATOR_EXPORT __attribute__((dllexport))
#define IMPACT_POINT_ESTIMATOR_IMPORT __attribute__((dllimport))
#else
#define IMPACT_POINT_ESTIMATOR_EXPORT __declspec(dllexport)
#define IMPACT_POINT_ESTIMATOR_IMPORT __declspec(dllimport)
#endif

#ifdef IMPACT_POINT_ESTIMATOR_DLL
#define IMPACT_POINT_ESTIMATOR_PUBLIC IMPACT_POINT_ESTIMATOR_EXPORT
#else
#define IMPACT_POINT_ESTIMATOR_PUBLIC IMPACT_POINT_ESTIMATOR_IMPORT
#endif

#define IMPACT_POINT_ESTIMATOR_PUBLIC_TYPE IMPACT_POINT_ESTIMATOR_PUBLIC

#define IMPACT_POINT_ESTIMATOR_LOCAL

#else

#define IMPACT_POINT_ESTIMATOR_EXPORT __attribute__((visibility("default")))
#define IMPACT_POINT_ESTIMATOR_IMPORT

#if __GNUC__ >= 4
#define IMPACT_POINT_ESTIMATOR_PUBLIC __attribute__((visibility("default")))
#define IMPACT_POINT_ESTIMATOR_LOCAL __attribute__((visibility("hidden")))
#else
#define IMPACT_POINT_ESTIMATOR_PUBLIC
#define IMPACT_POINT_ESTIMATOR_LOCAL
#endif

#define IMPACT_POINT_ESTIMATOR_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif // IMPACT_POINT_ESTIMATOR__VISIBILITY_H_