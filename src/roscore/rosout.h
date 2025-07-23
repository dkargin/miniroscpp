//
// Created by dkargin on 2/27/25.
//

#ifndef MINIROS_ROSOUT_H
#define MINIROS_ROSOUT_H

#include <string>

#include "miniros/node_handle.h"

#ifdef _MSC_VER
#define _SILENCE_CXX17_OLD_ALLOCATOR_MEMBERS_DEPRECATION_WARNING
#endif
#include "rosgraph_msgs/Log.hxx"

namespace miniros {
namespace master {

/**
 * @mainpage
 *
 * @htmlinclude manifest.html
 *
 * @b rosout logs messages sent to the /rosout topic,
 * which is a system-wide string logging mechanism.
 */

/**
 * the rosout node subscribes to /rosout, logs the messages to file, and re-broadcasts the messages to /rosout_agg
 */
class MINIROS_DECL Rosout
{
public:
  std::string log_file_name_;
  FILE* handle_;
  size_t max_file_size_;
  size_t current_file_size_;
  size_t max_backup_index_;
  size_t current_backup_index_;

  NodeHandle& node_;
  Subscriber rosout_sub_;
  Publisher agg_pub_;
  bool omit_topics_;

  Rosout(NodeHandle& nh);

  void init();
  void rosoutCallback(const rosgraph_msgs::Log::ConstPtr& msg);
};

}
} // namespace miniros

#endif //MINIROS_ROSOUT_H
