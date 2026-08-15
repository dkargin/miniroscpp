//
// NetworkMonitor implementation (Linux netlink or stub).
//

#include "miniros/network/network_monitor.h"

#include "internal_config.h"

#include "miniros/io/poll_set.h"
#include "miniros/io/io.h"
#include "miniros/internal/code_location.h"
#include "miniros/network/net_address.h"

#include "rosconsole/local_log.h"

#include <cerrno>
#include <cstring>
#include <vector>

#define MINIROS_PACKAGE_NAME "net"

#if defined(HAVE_NETLINK)
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace miniros {
namespace network {

namespace {

#if defined(HAVE_NETLINK)
Error fillAddressFromNl(const sockaddr* sa, NetAddress& out)
{
  if (!sa)
    return Error::InvalidAddress;
  return fillAddress(sa, out);
}

std::string ifNameFromIndex(int ifindex)
{
  char buf[IF_NAMESIZE] = {};
  if (ifindex > 0 && if_indextoname(static_cast<unsigned>(ifindex), buf))
    return buf;
  return {};
}
#endif

} // namespace

struct NetworkMonitor::Internal {
  PollSet* pollSet = nullptr;
  Callback callback;
  int fd = -1;
  bool started = false;

  void closeFd()
  {
    if (fd >= 0) {
      if (pollSet)
        pollSet->delSocket(fd);
      ::close(fd);
      fd = -1;
    }
  }

#if defined(HAVE_NETLINK)
  void onReadable()
  {
    // Netlink can deliver multiple messages; drain the socket.
    constexpr size_t kBuf = 8192;
    alignas(nlmsghdr) char buf[kBuf];
    for (;;) {
      const ssize_t n = ::recv(fd, buf, sizeof(buf), MSG_DONTWAIT);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
          return;
        LOCAL_WARN("NetworkMonitor::recv failed: %s", strerror(errno));
        return;
      }
      if (n == 0)
        return;
      parseBuffer(buf, static_cast<size_t>(n));
    }
  }

  void parseBuffer(const char* buf, size_t len)
  {
    for (const nlmsghdr* nh = reinterpret_cast<const nlmsghdr*>(buf);
         NLMSG_OK(nh, len);
         nh = NLMSG_NEXT(nh, len)) {
      if (nh->nlmsg_type == NLMSG_DONE || nh->nlmsg_type == NLMSG_ERROR)
        continue;

      Event ev;
      if (nh->nlmsg_type == RTM_NEWLINK || nh->nlmsg_type == RTM_DELLINK) {
        const ifinfomsg* ifi = static_cast<const ifinfomsg*>(NLMSG_DATA(nh));
        ev.ifindex = ifi->ifi_index;
        ev.ifname = ifNameFromIndex(ifi->ifi_index);
        const bool up = (ifi->ifi_flags & IFF_UP) && (ifi->ifi_flags & IFF_RUNNING);
        if (nh->nlmsg_type == RTM_DELLINK)
          ev.kind = EventKind::LinkDown;
        else
          ev.kind = up ? EventKind::LinkUp : EventKind::LinkDown;
        emit(ev);
        continue;
      }

      if (nh->nlmsg_type == RTM_NEWADDR || nh->nlmsg_type == RTM_DELADDR) {
        const ifaddrmsg* ifa = static_cast<const ifaddrmsg*>(NLMSG_DATA(nh));
        if (ifa->ifa_family != AF_INET && ifa->ifa_family != AF_INET6)
          continue;
        ev.ifindex = static_cast<int>(ifa->ifa_index);
        ev.ifname = ifNameFromIndex(static_cast<int>(ifa->ifa_index));
        ev.kind = (nh->nlmsg_type == RTM_NEWADDR) ? EventKind::AddressAdded : EventKind::AddressRemoved;

        const rtattr* rta = IFA_RTA(ifa);
        int rtl = IFA_PAYLOAD(nh);
        for (; RTA_OK(rta, rtl); rta = RTA_NEXT(rta, rtl)) {
          if (rta->rta_type != IFA_LOCAL && rta->rta_type != IFA_ADDRESS)
            continue;
          sockaddr_storage ss{};
          if (ifa->ifa_family == AF_INET) {
            auto* sin = reinterpret_cast<sockaddr_in*>(&ss);
            sin->sin_family = AF_INET;
            std::memcpy(&sin->sin_addr, RTA_DATA(rta), sizeof(sin->sin_addr));
          } else {
            auto* sin6 = reinterpret_cast<sockaddr_in6*>(&ss);
            sin6->sin6_family = AF_INET6;
            std::memcpy(&sin6->sin6_addr, RTA_DATA(rta), sizeof(sin6->sin6_addr));
          }
          if (fillAddressFromNl(reinterpret_cast<sockaddr*>(&ss), ev.address))
            break;
        }
        emit(ev);
      }
    }
  }

  void emit(const Event& ev)
  {
    if (!callback)
      return;
    const char* kind = "?";
    switch (ev.kind) {
    case EventKind::LinkUp:
      kind = "link-up";
      break;
    case EventKind::LinkDown:
      kind = "link-down";
      break;
    case EventKind::AddressAdded:
      kind = "addr-add";
      break;
    case EventKind::AddressRemoved:
      kind = "addr-del";
      break;
    }
    LOCAL_INFO("NetworkMonitor %s if=%s(%d) addr=%s", kind, ev.ifname.c_str(), ev.ifindex,
      ev.address.valid() ? ev.address.str().c_str() : "-");
    callback(ev);
  }
#endif
};

NetworkMonitor::NetworkMonitor() : internal_(std::make_unique<Internal>()) {}

NetworkMonitor::~NetworkMonitor()
{
  stop();
}

bool NetworkMonitor::supported()
{
#if defined(HAVE_NETLINK)
  return true;
#else
  return false;
#endif
}

Error NetworkMonitor::start(PollSet* pollSet, Callback callback)
{
  if (!internal_)
    return Error::InternalError;
  if (!pollSet)
    return Error::InvalidValue;
  if (internal_->started)
    return Error::Ok;

#if !defined(HAVE_NETLINK)
  (void)callback;
  return Error::NotSupported;
#else
  const int fd = ::socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC | SOCK_NONBLOCK, NETLINK_ROUTE);
  if (fd < 0) {
    LOCAL_ERROR("NetworkMonitor: netlink socket failed: %s", strerror(errno));
    return Error::SystemError;
  }

  sockaddr_nl addr{};
  addr.nl_family = AF_NETLINK;
  addr.nl_groups = RTMGRP_LINK | RTMGRP_IPV4_IFADDR | RTMGRP_IPV6_IFADDR;
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    LOCAL_ERROR("NetworkMonitor: netlink bind failed: %s", strerror(errno));
    ::close(fd);
    return Error::SystemError;
  }

  internal_->fd = fd;
  internal_->pollSet = pollSet;
  internal_->callback = std::move(callback);

  if (!pollSet->addSocket(fd, PollSet::EventIn,
        [this](int flags) {
          if (flags & PollSet::EventIn)
            internal_->onReadable();
          return 0;
        },
        {}, THIS_LOCATION)) {
    LOCAL_ERROR("NetworkMonitor: failed to add netlink fd to PollSet");
    internal_->closeFd();
    return Error::InternalError;
  }

  internal_->started = true;
  LOCAL_INFO("NetworkMonitor started (netlink RTMGRP_LINK|IFADDR)");
  return Error::Ok;
#endif
}

void NetworkMonitor::stop()
{
  if (!internal_)
    return;
  internal_->closeFd();
  internal_->callback = {};
  internal_->pollSet = nullptr;
  internal_->started = false;
}

bool NetworkMonitor::running() const
{
  return internal_ && internal_->started;
}

} // namespace network
} // namespace miniros
