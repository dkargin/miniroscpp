/*
 * Copyright (c) 2013, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#ifndef MINIROSCONSOLE_CONSOLE_IMPL_H
#define MINIROSCONSOLE_CONSOLE_IMPL_H

#include <miniros/macros.h>
#include "miniros/console.h"


// declare interface for rosconsole implementations
namespace miniros
{
namespace console
{
namespace impl
{

MINIROS_DECL void initialize();

MINIROS_DECL void shutdown();

MINIROS_DECL void register_appender(LogAppender* appender);

MINIROS_DECL void deregister_appender(LogAppender* appender);

MINIROS_DECL void print(void* handle, console::Level level, const char* str, const char* file, const char* function, int line);

MINIROS_DECL bool isEnabledFor(void* handle, console::Level level);

MINIROS_DECL void* getHandle(const std::string& name);

MINIROS_DECL std::string getName(void* handle);

MINIROS_DECL bool get_loggers(std::map<std::string, console::Level>& loggers);

MINIROS_DECL bool set_logger_level(const std::string& name, console::Level level);

} // namespace impl
} // namespace console
} // namespace miniros

#endif // MINIROSCONSOLE_CONSOLE_IMPL_H
