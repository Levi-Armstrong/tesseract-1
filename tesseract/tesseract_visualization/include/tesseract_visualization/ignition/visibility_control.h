// Copyright 2015 Open Source Robotics Foundation, Inc.
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

/* This header must be included by all descartes headers which declare symbols
 * which are defined in the descartes library. When not building the descartes
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the descartes
 * library cannot have, but the consuming code must have inorder to link.
 */
#ifndef TESSERACT_VISUALIZATION_IGNITION_VISIBILITY_CONTROL_H
#define TESSERACT_VISUALIZATION_IGNITION_VISIBILITY_CONTROL_H

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

// clang-format off
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TESSERACT_VISUALIZATION_IGNITION_EXPORT __attribute__ ((dllexport))
    #define TESSERACT_VISUALIZATION_IGNITION_IMPORT __attribute__ ((dllimport))
  #else
    #define TESSERACT_VISUALIZATION_IGNITION_EXPORT __declspec(dllexport)
    #define TESSERACT_VISUALIZATION_IGNITION_IMPORT __declspec(dllimport)
  #endif
  #ifndef TESSERACT_VISUALIZATION_IGNITION_STATIC_LIBRARY
    #ifdef TESSERACT_VISUALIZATION_IGNITION_LIBRARY_SHARED
      #define TESSERACT_VISUALIZATION_IGNITION_PUBLIC TESSERACT_VISUALIZATION_IGNITION_EXPORT
    #else
      #define TESSERACT_VISUALIZATION_IGNITION_PUBLIC TESSERACT_VISUALIZATION_IGNITION_IMPORT
    #endif
  #else
    #define TESSERACT_VISUALIZATION_IGNITION_PUBLIC
  #endif
  #define TESSERACT_VISUALIZATION_IGNITION_PUBLIC_TYPE TESSERACT_VISUALIZATION_IGNITION_PUBLIC
  #define TESSERACT_VISUALIZATION_IGNITION_LOCAL
#else
  #define TESSERACT_VISUALIZATION_IGNITION_EXPORT __attribute__ ((visibility("default")))
  #define TESSERACT_VISUALIZATION_IGNITION_IMPORT
  #if __GNUC__ >= 4
    #define TESSERACT_VISUALIZATION_IGNITION_PUBLIC __attribute__ ((visibility("default")))
    #define TESSERACT_VISUALIZATION_IGNITION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TESSERACT_VISUALIZATION_IGNITION_PUBLIC
    #define TESSERACT_VISUALIZATION_IGNITION_LOCAL
  #endif
  #define TESSERACT_VISUALIZATION_IGNITION_PUBLIC_TYPE
#endif
// clang-format on

#endif  // TESSERACT_VISUALIZATION_IGNITION_VISIBILITY_CONTROL_H