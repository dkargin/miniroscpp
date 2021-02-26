/*
 * Copyright (C) 2010, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "miniros/config.h"

#if defined(__GNUC__)
#define MINIROS_DEPRECATED __attribute__((deprecated))
#define MINIROS_FORCE_INLINE __attribute__((always_inline))
#elif defined(_MSC_VER)
#define MINIROS_DEPRECATED
#define MINIROS_FORCE_INLINE __forceinline
#else
#define MINIROS_DEPRECATED
#define MINIROS_FORCE_INLINE inline
#endif

/*
  Windows import/export and gnu http://gcc.gnu.org/wiki/Visibility
  macros.
 */
#if defined(_MSC_VER)
    #define MINIROS_HELPER_IMPORT __declspec(dllimport)
    #define MINIROS_HELPER_EXPORT __declspec(dllexport)
    #define MINIROS_HELPER_LOCAL
#elif __GNUC__ >= 4
    #define MINIROS_HELPER_IMPORT __attribute__ ((visibility("default")))
    #define MINIROS_HELPER_EXPORT __attribute__ ((visibility("default")))
    #define MINIROS_HELPER_LOCAL  __attribute__ ((visibility("hidden")))
#else
    #define MINIROS_HELPER_IMPORT
    #define MINIROS_HELPER_EXPORT
    #define MINIROS_HELPER_LOCAL
#endif

// Ignore warnings about import/exports when deriving from std classes.
#ifdef _MSC_VER
  #pragma warning(disable: 4251)
  #pragma warning(disable: 4275)
#endif

#ifdef MINIROS_BUILD_SHARED_LIBS // ros is being built around shared libraries
  #ifdef miniros_EXPORTS      // we are building a shared lib/dll
    #define MINIROS_DECL MINIROS_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define MINIROS_DECL MINIROS_HELPER_IMPORT
  #endif
#else // ros is being built around static libraries
  #define MINIROS_DECL
#endif
