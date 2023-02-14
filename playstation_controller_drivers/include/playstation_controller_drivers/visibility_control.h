// Copyright (c) 2021 OUXT Polaris
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

#ifndef PLAYSTATION_CONTROLLER_DRIVERS__VISIBILITY_CONTROL_H_
#define PLAYSTATION_CONTROLLER_DRIVERS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PLAYSTATION_CONTROLLER_DRIVERS_EXPORT __attribute__((dllexport))
#define PLAYSTATION_CONTROLLER_DRIVERS_IMPORT __attribute__((dllimport))
#else
#define PLAYSTATION_CONTROLLER_DRIVERS_EXPORT __declspec(dllexport)
#define PLAYSTATION_CONTROLLER_DRIVERS_IMPORT __declspec(dllimport)
#endif
#ifdef PLAYSTATION_CONTROLLER_DRIVERS_BUILDING_LIBRARY
#define PLAYSTATION_CONTROLLER_DRIVERS_PUBLIC PLAYSTATION_CONTROLLER_DRIVERS_EXPORT
#else
#define PLAYSTATION_CONTROLLER_DRIVERS_PUBLIC PLAYSTATION_CONTROLLER_DRIVERS_IMPORT
#endif
#define PLAYSTATION_CONTROLLER_DRIVERS_PUBLIC_TYPE PLAYSTATION_CONTROLLER_DRIVERS_PUBLIC
#define PLAYSTATION_CONTROLLER_DRIVERS_LOCAL
#else
#define PLAYSTATION_CONTROLLER_DRIVERS_EXPORT __attribute__((visibility("default")))
#define PLAYSTATION_CONTROLLER_DRIVERS_IMPORT
#if __GNUC__ >= 4
#define PLAYSTATION_CONTROLLER_DRIVERS_PUBLIC __attribute__((visibility("default")))
#define PLAYSTATION_CONTROLLER_DRIVERS_LOCAL __attribute__((visibility("hidden")))
#else
#define PLAYSTATION_CONTROLLER_DRIVERS_PUBLIC
#define PLAYSTATION_CONTROLLER_DRIVERS_LOCAL
#endif
#define PLAYSTATION_CONTROLLER_DRIVERS_PUBLIC_TYPE
#endif

#endif  // PLAYSTATION_CONTROLLER_DRIVERS__VISIBILITY_CONTROL_H_
