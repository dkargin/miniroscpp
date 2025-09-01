//
// Created by dkargin on 8/31/25.
//

#include <cassert>
#include <csignal>
#include <iostream>

#include <unistd.h>
#include <sys/wait.h>
#include <sys/syscall.h>

#include "miniros/launcher.h"


namespace miniros {

struct Launcher::Internal {
  pid_t pid = -1;

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

Launcher::Launcher(int pid)
{
  assert(pid > 0);
  internal_ = new Internal();
  internal_->pid = pid;
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

  pid_t pid = fork();
  if (pid == 0) {
    // Child process. Never return here.

    // A collection of program arguments.
    std::vector<char*> pargs {const_cast<char*>(appPath.c_str())};

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
  return Error::Ok;
}

bool Launcher::valid() const
{
  return internal_ != nullptr && internal_->pid != -1;
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

  if (internal_->pid < 0) {
    return Error::InvalidValue;
  }
  if (kill(internal_->pid, signal) != 0) {
    return Error::SystemError;
  }
  return Error::Ok;
}

int Launcher::pid() const
{
  return internal_ ? internal_->pid : -1;
}

int Launcher::waitExit()
{
  if (!valid()) {
    return Error::InternalError;
  }

  int status = 0;
  waitpid(internal_->pid, &status, 0);
  return WEXITSTATUS(status);
}

}