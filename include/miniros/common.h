/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
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

#ifndef MINIROSCPP_COMMON_H
#define MINIROSCPP_COMMON_H

/// This file contains some very common includes.
#include <string>

#include "miniros/rosassert.h"
#include "miniros/internal/forwards.h"
#include "miniros/serialized_message.h"
#include "miniros/errors.h"

namespace miniros
{

MINIROS_DECL void disableAllSignalsInThisThread();
MINIROS_DECL void setThreadName(const char* name);

/// Notify system that node has successfully started.
MINIROS_DECL Error notifyNodeStarted();
MINIROS_DECL Error notifyNodeExiting();

/// UUID has the following hex structure:
/// 8-4-4-4-12
///  0 1 2 3  4 5  6 7  8 9 101112131415
/// xxxxxxxx-xxxx-Mxxx-Nxxx-xxxxxxxxxxxx
/// M and N encode type of UUID. They correspond to character 6 and 8.
struct MINIROS_DECL UUID {
  uint8_t bytes[16] = {};

  void generate();

  /// Reset all values to zero.
  void reset();

  /// Check if UUID is valid.
  bool valid() const;

  std::string toString() const;
};

MINIROS_DECL Error makeDirectory(const std::string& path);

MINIROS_DECL Error changeCurrentDirectory(const std::string& path);

/// Enable printing backtrace during crash.
/// For internal usage only.
MINIROS_DECL Error handleCrashes();


}

#endif

