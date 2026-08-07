//
// Created by dkargin on 8/31/25.
//

#include <cassert>
#include <csignal>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

#include "miniros/io/io.h"
#include "miniros/launcher.h"
#include "miniros/rostime.h"

#if defined(WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace miniros {

namespace {

bool parseNotifyPayload(const std::string& msg, ChildReady& out)
{
  std::istringstream iss(msg);
  std::string line;
  bool sawReady = false;
  while (std::getline(iss, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    const size_t eq = line.find('=');
    if (eq == std::string::npos)
      continue;
    const std::string key = line.substr(0, eq);
    const std::string val = line.substr(eq + 1);
    if (key == "READY" && val == "1")
      sawReady = true;
    else if (key == "MAINPID")
      out.pid = static_cast<int64_t>(std::strtoll(val.c_str(), nullptr, 10));
    else if (key == "X_MINIROS_RPC_PORT")
      out.rpcPort = static_cast<int>(std::strtol(val.c_str(), nullptr, 10));
    else if (key == "X_MINIROS_URI")
      out.uri = val;
  }
  out.ready = sawReady;
  return sawReady;
}

/// Build child environment: parent environ + overrides (last wins).
std::vector<std::string> buildChildEnviron(const std::vector<std::string>& overrides)
{
  std::map<std::string, std::string> vars;
#if defined(WIN32)
  LPWCH envBlock = GetEnvironmentStringsW();
  if (envBlock) {
    for (LPWCH p = envBlock; *p; ) {
      std::wstring wentry(p);
      p += wentry.size() + 1;
      const int need = WideCharToMultiByte(CP_UTF8, 0, wentry.c_str(), -1, nullptr, 0, nullptr, nullptr);
      if (need <= 1)
        continue;
      std::string entry(static_cast<size_t>(need - 1), '\0');
      WideCharToMultiByte(CP_UTF8, 0, wentry.c_str(), -1, &entry[0], need, nullptr, nullptr);
      const size_t eq = entry.find('=');
      if (eq != std::string::npos && eq > 0)
        vars[entry.substr(0, eq)] = entry.substr(eq + 1);
    }
    FreeEnvironmentStringsW(envBlock);
  }
#else
  for (char** e = ::environ; e && *e; ++e) {
    const char* entry = *e;
    const char* eq = strchr(entry, '=');
    if (!eq)
      continue;
    vars[std::string(entry, eq)] = std::string(eq + 1);
  }
#endif
  for (const std::string& ov : overrides) {
    const size_t eq = ov.find('=');
    if (eq == std::string::npos || eq == 0)
      continue;
    vars[ov.substr(0, eq)] = ov.substr(eq + 1);
  }
  std::vector<std::string> out;
  out.reserve(vars.size());
  for (const auto& [k, v] : vars)
    out.push_back(k + "=" + v);
  return out;
}

} // namespace

struct Launcher::Internal {
#ifdef WIN32
  HANDLE process = INVALID_HANDLE_VALUE;
  HANDLE notifyRead = INVALID_HANDLE_VALUE;
  HANDLE notifyWrite = INVALID_HANDLE_VALUE; // closed after CreateProcess

  bool isPidValid() const { return process != INVALID_HANDLE_VALUE; }

  void closeNotify()
  {
    if (notifyRead != INVALID_HANDLE_VALUE) {
      CloseHandle(notifyRead);
      notifyRead = INVALID_HANDLE_VALUE;
    }
    if (notifyWrite != INVALID_HANDLE_VALUE) {
      CloseHandle(notifyWrite);
      notifyWrite = INVALID_HANDLE_VALUE;
    }
  }
#else
  pid_t pid = -1;
  int notifyFd = -1; // AF_UNIX datagram listen / pipe read end
  std::string notifySocketPath; // for filesystem-backed sockets (empty if abstract)

  bool isPidValid() const { return pid != -1; }

  void closeNotify()
  {
    if (notifyFd >= 0) {
      ::close(notifyFd);
      notifyFd = -1;
    }
    if (!notifySocketPath.empty()) {
      ::unlink(notifySocketPath.c_str());
      notifySocketPath.clear();
    }
  }
#endif

  std::vector<std::string> env;
  bool detached = false;
  bool notifyEnabled = false;
  ChildReady ready;
};

Launcher::Launcher()
  : internal_(nullptr)
{
  internal_ = new Internal();
}

Launcher::Launcher(int64_t pid)
{
  assert(pid > 0);
  internal_ = new Internal();
#ifdef WIN32
  internal_->process = reinterpret_cast<HANDLE>(pid);
#else
  internal_->pid = static_cast<pid_t>(pid);
#endif
}

Launcher::~Launcher()
{
  if (internal_) {
    if (valid() && !internal_->detached) {
      // Best-effort stop so destruction cannot hang forever if kill is denied
      // (restricted environments) or the child ignores catchable signals.
      (void)terminate();
#if defined(WIN32)
      WaitForSingleObject(internal_->process, 2000);
      DWORD exitCode = 0;
      (void)GetExitCodeProcess(internal_->process, &exitCode);
      CloseHandle(internal_->process);
      internal_->process = INVALID_HANDLE_VALUE;
#else
      for (int i = 0; i < 40; ++i) {
        int status = 0;
        const pid_t r = waitpid(internal_->pid, &status, WNOHANG);
        if (r == internal_->pid || (r < 0 && errno == ECHILD)) {
          internal_->pid = -1;
          break;
        }
        usleep(50000);
      }
      if (internal_->pid != -1) {
        // Timed out waiting; drop tracking (may leave a zombie until this process exits).
        internal_->pid = -1;
      }
#endif
    }
    internal_->closeNotify();
#ifdef WIN32
    if (internal_->process != INVALID_HANDLE_VALUE) {
      CloseHandle(internal_->process);
      internal_->process = INVALID_HANDLE_VALUE;
    }
#endif
    delete internal_;
    internal_ = nullptr;
  }
}

Error Launcher::start(const std::filesystem::path& appPath, const std::vector<std::string>& args, int flags)
{
  if (!internal_)
    return Error::InternalError;

  std::error_code ec;
  auto status = std::filesystem::status(appPath, ec);
  if (ec || !std::filesystem::exists(status)) {
    std::cerr << "Launcher: file \"" << appPath << "\" does not exist" << std::endl;
    return Error::FileNotFound;
  }
  if (!std::filesystem::is_regular_file(status) && !std::filesystem::is_symlink(status)) {
    std::cerr << "Launcher: \"" << appPath << "\" is not a regular file" << std::endl;
    return Error::InvalidValue;
  }

#ifndef WIN32
  if (access(appPath.c_str(), X_OK) != 0) {
    std::cerr << "Launcher: no execute permission for \"" << appPath << "\"" << std::endl;
    return Error::PermissionDenied;
  }
#endif

  std::string compatPath = appPath.u8string();
  internal_->closeNotify();
  internal_->ready = {};
  internal_->notifyEnabled = (flags & FLAG_NOTIFY) != 0;

  std::vector<std::string> envOverrides = internal_->env;

#ifdef WIN32
  if (internal_->notifyEnabled) {
    SECURITY_ATTRIBUTES sa;
    ZeroMemory(&sa, sizeof(sa));
    sa.nLength = sizeof(sa);
    sa.bInheritHandle = TRUE;
    if (!CreatePipe(&internal_->notifyRead, &internal_->notifyWrite, &sa, 0)) {
      return Error::SystemError;
    }
    // Parent keeps the read end; child inherits write end only.
    SetHandleInformation(internal_->notifyRead, HANDLE_FLAG_INHERIT, 0);
    std::ostringstream oss;
    oss << reinterpret_cast<uintptr_t>(internal_->notifyWrite);
    envOverrides.push_back(std::string("MINIROS_NOTIFY_HANDLE=") + oss.str());
  }

  STARTUPINFOA si;
  PROCESS_INFORMATION pi;
  ZeroMemory(&si, sizeof(si));
  si.cb = sizeof(si);
  ZeroMemory(&pi, sizeof(pi));

  std::string argString = "\"" + compatPath + "\"";
  for (const auto& arg : args) {
    argString += " ";
    argString += arg;
  }

  std::vector<std::string> childEnv = buildChildEnviron(envOverrides);
  // Windows environment block: KEY=VAL\0...\0\0
  size_t envBytes = 1;
  for (const auto& e : childEnv)
    envBytes += e.size() + 1;
  std::vector<char> envBlock(envBytes, '\0');
  size_t pos = 0;
  for (const auto& e : childEnv) {
    memcpy(&envBlock[pos], e.c_str(), e.size());
    pos += e.size() + 1;
  }

  DWORD creationFlags = CREATE_UNICODE_ENVIRONMENT;
  // We built an ANSI env block above; use CreateProcessA without UNICODE env.
  creationFlags = 0;

  if (!CreateProcessA(compatPath.c_str(),
        argString.data(),
        nullptr,
        nullptr,
        TRUE, // inherit handles (notify write pipe)
        creationFlags,
        envBlock.data(),
        nullptr,
        &si,
        &pi)) {
    const DWORD err = GetLastError();
    internal_->closeNotify();
    if (err == ERROR_FILE_NOT_FOUND || err == ERROR_PATH_NOT_FOUND)
      return Error::FileNotFound;
    if (err == ERROR_ACCESS_DENIED || err == ERROR_BAD_EXE_FORMAT)
      return Error::PermissionDenied;
    return Error::SystemError;
  }
  CloseHandle(pi.hThread);
  if (internal_->notifyWrite != INVALID_HANDLE_VALUE) {
    CloseHandle(internal_->notifyWrite);
    internal_->notifyWrite = INVALID_HANDLE_VALUE;
  }
  internal_->process = pi.hProcess;
  if (flags & FLAG_DETACHED)
    internal_->detached = true;
  return Error::Ok;

#else
  if (internal_->notifyEnabled) {
    // Prefer NOTIFY_SOCKET (sd_notify) so the same child path works under systemd.
    internal_->notifyFd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (internal_->notifyFd < 0)
      return Error::SystemError;

    sockaddr_un addr = {};
    addr.sun_family = AF_UNIX;
    // Abstract socket: leading '@' in env, leading '\0' in bind path.
    const std::string name = "miniros-launch-" + std::to_string(getpid()) + "-" +
      std::to_string(SteadyTime::now().toNSec());
    if (name.size() + 1 >= sizeof(addr.sun_path)) {
      internal_->closeNotify();
      return Error::OutOfMemory;
    }
    addr.sun_path[0] = '\0';
    memcpy(addr.sun_path + 1, name.c_str(), name.size());
    const socklen_t addrLen = static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1 + name.size());
    if (bind(internal_->notifyFd, reinterpret_cast<sockaddr*>(&addr), addrLen) != 0) {
      internal_->closeNotify();
      return Error::SystemError;
    }
    envOverrides.push_back(std::string("NOTIFY_SOCKET=@") + name);
  }

  pid_t child = fork();
  if (child < 0) {
    internal_->closeNotify();
    return Error::SystemError;
  }
  if (child == 0) {
    // Child: close listen socket; connect via NOTIFY_SOCKET env.
    if (internal_->notifyFd >= 0)
      ::close(internal_->notifyFd);

    std::vector<char*> pargs{const_cast<char*>(compatPath.c_str())};
    for (const std::string& arg : args)
      pargs.push_back(const_cast<char*>(arg.c_str()));
    pargs.push_back(nullptr);

    std::vector<std::string> childEnvStorage = buildChildEnviron(envOverrides);
    std::vector<char*> penv;
    penv.reserve(childEnvStorage.size() + 1);
    for (std::string& e : childEnvStorage)
      penv.push_back(e.data());
    penv.push_back(nullptr);

    if (flags & FLAG_DETACHED) {
      setsid();
      ::signal(SIGHUP, SIG_IGN);
      close(STDIN_FILENO);
      close(STDOUT_FILENO);
      close(STDERR_FILENO);
    }
    if (execve(appPath.c_str(), pargs.data(), penv.data()) != 0)
      _exit(EXIT_FAILURE);
  }

  internal_->pid = child;
  if (flags & FLAG_DETACHED)
    internal_->detached = true;
  return Error::Ok;
#endif
}

bool Launcher::valid() const
{
  return internal_ != nullptr && internal_->isPidValid();
}

bool Launcher::running()
{
  if (!valid())
    return false;

#ifdef WIN32
  DWORD exitCode = 0;
  if (!GetExitCodeProcess(internal_->process, &exitCode))
    return false;
  if (exitCode == STILL_ACTIVE)
    return true;
  CloseHandle(internal_->process);
  internal_->process = INVALID_HANDLE_VALUE;
  internal_->closeNotify();
  return false;
#else
  int status = 0;
  const pid_t r = waitpid(internal_->pid, &status, WNOHANG);
  if (r == 0)
    return true;
  if (r == internal_->pid || (r < 0 && errno == ECHILD)) {
    internal_->pid = -1;
    internal_->closeNotify();
    return false;
  }
  return false;
#endif
}

Launcher& Launcher::env(const char* name, const char* value)
{
  if (internal_ && name && value) {
    internal_->env.push_back(std::string(name) + "=" + value);
  }
  return *this;
}

Error Launcher::signal(int signo)
{
  if (!internal_)
    return Error::InternalError;
  if (!internal_->isPidValid())
    return Error::InvalidValue;
#ifdef WIN32
  if (signo == SIGINT || signo == SIGTERM)
    return stop();
  return terminate();
#else
  if (kill(internal_->pid, signo) != 0) {
    if (errno == EPERM || errno == EACCES)
      return Error::PermissionDenied;
    if (errno == ESRCH)
      return Error::InvalidValue;
    return Error::SystemError;
  }
  return Error::Ok;
#endif
}

Error Launcher::stop()
{
  if (!internal_)
    return Error::InternalError;
  if (!internal_->isPidValid())
    return Error::InvalidValue;
#ifdef WIN32
  // Soft interrupt only — do not fall back to TerminateProcess (that is terminate()).
  if (!GenerateConsoleCtrlEvent(CTRL_BREAK_EVENT, GetProcessId(internal_->process)))
    return Error::SystemError;
  return Error::Ok;
#else
  if (kill(internal_->pid, SIGINT) != 0) {
    if (errno == EPERM || errno == EACCES)
      return Error::PermissionDenied;
    if (errno == ESRCH)
      return Error::InvalidValue;
    return Error::SystemError;
  }
  return Error::Ok;
#endif
}

Error Launcher::terminate()
{
  if (!internal_)
    return Error::InternalError;
  if (!internal_->isPidValid())
    return Error::InvalidValue;
#ifdef WIN32
  if (!TerminateProcess(internal_->process, 1))
    return Error::SystemError;
  return Error::Ok;
#else
  if (kill(internal_->pid, SIGKILL) != 0) {
    if (errno == EPERM || errno == EACCES)
      return Error::PermissionDenied;
    if (errno == ESRCH)
      return Error::InvalidValue;
    return Error::SystemError;
  }
  return Error::Ok;
#endif
}

const ChildReady& Launcher::childReady() const
{
  static ChildReady empty;
  return internal_ ? internal_->ready : empty;
}

Error Launcher::waitReady(ChildReady* out, const WallDuration& timeout)
{
  if (!internal_)
    return Error::InternalError;
  if (!internal_->notifyEnabled)
    return Error::InvalidValue;

  const SteadyTime deadline = SteadyTime::now() + timeout;
  std::string buf;

#ifdef WIN32
  if (internal_->notifyRead == INVALID_HANDLE_VALUE)
    return Error::InvalidHandle;

  while (SteadyTime::now() < deadline) {
    DWORD avail = 0;
    if (!PeekNamedPipe(internal_->notifyRead, nullptr, 0, nullptr, &avail, nullptr)) {
      const DWORD err = GetLastError();
      if (err == ERROR_BROKEN_PIPE)
        break;
      return Error::SystemError;
    }
    if (avail > 0) {
      std::vector<char> chunk(avail);
      DWORD got = 0;
      if (!ReadFile(internal_->notifyRead, chunk.data(), avail, &got, nullptr))
        return Error::SystemError;
      buf.append(chunk.data(), got);
      ChildReady parsed;
      if (parseNotifyPayload(buf, parsed)) {
        internal_->ready = parsed;
        if (out)
          *out = parsed;
        return Error::Ok;
      }
    } else {
      Sleep(10);
    }
  }
  return Error::Timeout;

#else
  if (internal_->notifyFd < 0)
    return Error::InvalidHandle;

  while (SteadyTime::now() < deadline) {
    const double remain = (deadline - SteadyTime::now()).toSec();
    if (remain <= 0)
      break;
    pollfd pfd = {};
    pfd.fd = internal_->notifyFd;
    pfd.events = POLLIN;
    const int ms = static_cast<int>(std::min(remain * 1000.0, 1000.0));
    const int pr = poll(&pfd, 1, std::max(ms, 1));
    if (pr < 0) {
      if (errno == EINTR)
        continue;
      return Error::SystemError;
    }
    if (pr == 0)
      continue;

    char packet[4096];
    const ssize_t n = recv(internal_->notifyFd, packet, sizeof(packet) - 1, 0);
    if (n < 0) {
      if (errno == EINTR)
        continue;
      return Error::SystemError;
    }
    if (n == 0)
      continue;
    buf.assign(packet, static_cast<size_t>(n));
    ChildReady parsed;
    if (parseNotifyPayload(buf, parsed)) {
      internal_->ready = parsed;
      if (out)
        *out = parsed;
      return Error::Ok;
    }
  }
  return Error::Timeout;
#endif
}

int64_t Launcher::pid() const
{
#ifdef WIN32
  if (!internal_ || internal_->process == INVALID_HANDLE_VALUE)
    return -1;
  return static_cast<int64_t>(GetProcessId(internal_->process));
#else
  return internal_ ? static_cast<int64_t>(internal_->pid) : -1;
#endif
}

int Launcher::waitExit()
{
  if (!valid())
    return -1;

#ifdef WIN32
  WaitForSingleObject(internal_->process, INFINITE);
  DWORD exitCode = 1;
  if (!GetExitCodeProcess(internal_->process, &exitCode))
    return -1;
  CloseHandle(internal_->process);
  internal_->process = INVALID_HANDLE_VALUE;
  internal_->closeNotify();
  return static_cast<int>(exitCode);
#else
  int status = 0;
  for (;;) {
    const pid_t r = waitpid(internal_->pid, &status, 0);
    if (r == internal_->pid)
      break;
    if (r < 0 && errno == EINTR)
      continue;
    internal_->pid = -1;
    internal_->closeNotify();
    return -1;
  }
  internal_->pid = -1;
  internal_->closeNotify();
  if (WIFEXITED(status))
    return WEXITSTATUS(status);
  if (WIFSIGNALED(status))
    return 128 + WTERMSIG(status);
  return -1;
#endif
}

Error Launcher::waitExit(const WallDuration& timeout, int* exitCode)
{
  if (!valid())
    return Error::InvalidValue;

  const SteadyTime deadline = SteadyTime::now() + timeout;

  auto finish = [&](int code) -> Error {
    if (exitCode)
      *exitCode = code;
    return Error::Ok;
  };

#ifdef WIN32
  for (;;) {
    const double remainSec = (deadline - SteadyTime::now()).toSec();
    const DWORD ms = remainSec <= 0.0 ? 0 : static_cast<DWORD>(std::min(remainSec * 1000.0, 50.0));
    const DWORD wr = WaitForSingleObject(internal_->process, ms);
    if (wr == WAIT_OBJECT_0) {
      DWORD code = 1;
      if (!GetExitCodeProcess(internal_->process, &code))
        return Error::SystemError;
      CloseHandle(internal_->process);
      internal_->process = INVALID_HANDLE_VALUE;
      internal_->closeNotify();
      return finish(static_cast<int>(code));
    }
    if (wr == WAIT_TIMEOUT) {
      if (SteadyTime::now() >= deadline)
        return Error::Timeout;
      continue;
    }
    return Error::SystemError;
  }
#else
  for (;;) {
    int status = 0;
    const pid_t r = waitpid(internal_->pid, &status, WNOHANG);
    if (r == internal_->pid) {
      internal_->pid = -1;
      internal_->closeNotify();
      if (WIFEXITED(status))
        return finish(WEXITSTATUS(status));
      if (WIFSIGNALED(status))
        return finish(128 + WTERMSIG(status));
      return finish(-1);
    }
    if (r < 0) {
      if (errno == EINTR)
        continue;
      if (errno == ECHILD) {
        internal_->pid = -1;
        internal_->closeNotify();
        return finish(-1);
      }
      return Error::SystemError;
    }
    // r == 0: still running
    if (SteadyTime::now() >= deadline)
      return Error::Timeout;
    const double remain = (deadline - SteadyTime::now()).toSec();
    const int ms = static_cast<int>(std::min(std::max(remain * 1000.0, 1.0), 50.0));
    usleep(static_cast<useconds_t>(ms * 1000));
  }
#endif
}

int Launcher::stopAndWait(const WallDuration& grace)
{
  if (!valid())
    return -1;

  (void)stop();
  int code = -1;
  if (waitExit(grace, &code) == Error::Ok)
    return code;

  (void)terminate();
  return waitExit();
}

int64_t Launcher::myPid()
{
#if defined(WIN32)
  return static_cast<int64_t>(GetCurrentProcessId());
#else
  return static_cast<int64_t>(::getpid());
#endif
}

} // namespace miniros
