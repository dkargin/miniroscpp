//
// Created by dkargin on 8/31/25.
//

#include <cassert>
#include <csignal>
#include <iostream>

#include "miniros/transport/io.h"


#include "miniros/launcher.h"


namespace miniros {

struct Launcher::Internal {
#ifdef WIN32
  /// Handle of a process.
  HANDLE pid = INVALID_HANDLE_VALUE;

  bool isPidValid() const
  {
    return pid != INVALID_HANDLE_VALUE;
  }
#else
  pid_t pid = -1;

  bool isPidValid() const
  {
    return pid != -1;
  }
#endif

  /// File descriptor, associated with pid.
  //int pidfd = -1;

  /// A collection of baked environment assignments.
  /// Each string looks like VAR=VAL
  std::vector<std::string> env;
  bool detached = false;
};

Launcher::Launcher()
  :internal_(nullptr)
{
  internal_ = new Internal();
}

Launcher::Launcher(int64_t pid)
{
  assert(pid > 0);
  internal_ = new Internal();
#ifdef WIN32
  internal_->pid = reinterpret_cast<HANDLE>(pid);
#else
  internal_->pid = pid;
#endif
}

Launcher::~Launcher()
{
  if (internal_) {
    if (valid() && !internal_->detached) {
      waitExit();
    }

    delete internal_;
    internal_ = nullptr;
  }
}

Error Launcher::start(const std::filesystem::path& appPath, const std::vector<std::string>& args, int flags)
{
  if (!internal_)
    return Error::InternalError;

  auto status = std::filesystem::status(appPath);
  if (!std::filesystem::exists(status)) {
    std::cerr << "Launcher: file \"" << appPath << "\" does not exists" << std::endl;
    return Error::InvalidValue;
  }

  // A collection of program arguments.
  std::string compatPath = appPath.u8string();

  // Start the child process.

#ifdef WIN32
  /*
  [in, optional]      LPCSTR                lpApplicationName,
  [in, out, optional] LPSTR                 lpCommandLine,
  [in, optional]      LPSECURITY_ATTRIBUTES lpProcessAttributes,
  [in, optional]      LPSECURITY_ATTRIBUTES lpThreadAttributes,
  [in]                BOOL                  bInheritHandles,
  [in]                DWORD                 dwCreationFlags,
  [in, optional]      LPVOID                lpEnvironment,
  [in, optional]      LPCSTR                lpCurrentDirectory,
  [in]                LPSTARTUPINFOA        lpStartupInfo,
  [out]               LPPROCESS_INFORMATION lpProcessInformation
   */
  STARTUPINFO si;
  PROCESS_INFORMATION pi;

  ZeroMemory( &si, sizeof(si) );
  si.cb = sizeof(si);
  ZeroMemory( &pi, sizeof(pi) );
  std::string argString = compatPath;
  for (const auto& arg : args) {
    argString += " ";
    argString += arg;
  }

  if( !CreateProcess(compatPath.c_str(),
        (char*)argString.c_str(),        // Command line
        NULL,           // Process handle not inheritable
        NULL,           // Thread handle not inheritable
        FALSE,          // Set handle inheritance to FALSE
        0,              // No creation flags
        NULL,           // Use parent's environment block
        NULL,           // Use parent's starting directory
        &si,            // Pointer to STARTUPINFO structure
        &pi )           // Pointer to PROCESS_INFORMATION structure
    )
  {
    int error = GetLastError();
    std::cerr << "CreateProcess failed: " << error << std::endl;
    return Error::SystemError;;
  }
  internal_->pid = pi.hProcess;
  return Error::NotImplemented;
#else

  pid_t pid = fork();
  if (pid == 0) {
    // Child process. Never return here.

    std::vector<char*> pargs {const_cast<char*>(compatPath.c_str())};
    for (const std::string& arg : args) {
      pargs.push_back(const_cast<char*>(arg.c_str()));
    }
    pargs.push_back(nullptr);

    std::vector<char*> penv;
    for (std::string& env: internal_->env) {
      penv.push_back(const_cast<char*>(env.c_str()));
    }
    penv.push_back(nullptr);
    if (flags & FLAG_DETACHED) {
      setsid();
      // Ignore notification from parent process.
      ::signal(SIGHUP, SIG_IGN);
      // CTest can wait until all child handles are closed.
      // https://gitlab.kitware.com/cmake/cmake/-/issues/20116
      close(STDIN_FILENO);
      close(STDOUT_FILENO);
      close(STDERR_FILENO);
    }
    if (execve(appPath.c_str(), pargs.data(), penv.data()) != 0) {
      exit(EXIT_FAILURE);
    }

  }
  // Parent thread.
  internal_->pid = pid;
  if (flags & FLAG_DETACHED) {
    internal_->detached = true;
  }
  //internal_->pidfd = pidfd_open(pid);
#endif
  return Error::Ok;
}

bool Launcher::valid() const
{
  return internal_ != nullptr && internal_->isPidValid();
}

/// Add environment variables.
Launcher& Launcher::env(const char* name, const char* value)
{
  if (internal_ && name && value) {
    std::string env_pair = std::string(name) + "=" + value;
    internal_->env.push_back(std::move(env_pair));
  }
  return *this;
}

Error Launcher::signal(int signal)
{
  if (!internal_) {
    return Error::InternalError;
  }

  if (!internal_->isPidValid()) {
    return Error::InvalidValue;
  }
#ifdef WIN32
  return Error::NotImplemented;
#else
  if (kill(internal_->pid, signal) != 0) {
    return Error::SystemError;
  }
#endif
  return Error::Ok;
}

int64_t Launcher::pid() const
{
#ifdef WIN32
  HANDLE pid = internal_ ? internal_->pid : INVALID_HANDLE_VALUE;
  return reinterpret_cast<int64_t>(pid);
#else
  return internal_ ? static_cast<int64_t>(internal_->pid) : -1;
#endif
}

int Launcher::waitExit()
{
  if (!valid()) {
    return Error::InternalError;
  }

#ifdef WIN32
  DWORD exitCode = 0;
  if (!GetExitCodeProcess(internal_->pid, &exitCode)) {
    return Error::SystemError;
  }
  return Error::Ok;
#else

  int status = 0;
  waitpid(internal_->pid, &status, 0);
  return WEXITSTATUS(status);
#endif
}

int64_t Launcher::myPid()
{
  return ::getpid();
}

}