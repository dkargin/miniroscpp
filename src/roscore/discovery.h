//
// Created by dkargin on 8/24/25.
//

#ifndef MINIROS_DISCOVERY_H
#define MINIROS_DISCOVERY_H

#include <memory>

#include "miniros/macros.h"
#include "miniros/errors.h"

namespace miniros {

class PollSet;
struct UUID;

namespace master {

class AddressResolver;

/// Discovery deals with finding other miniros masters across the network.
class MINIROS_DECL Discovery {
public:
  Discovery(AddressResolver* resolver);
  ~Discovery();

  /// Start discovery.
  Error start(PollSet* pollSet, const UUID& uuid, int port);

  /// Stop discovery.
  void stop();

  /// Run broadcast about this master.
  Error doBroadcast();

protected:
  struct Internal;
  std::unique_ptr<Internal> internal_;
};
}
}
#endif // MINIROS_DISCOVERY_H
