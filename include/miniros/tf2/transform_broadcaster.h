/*
 * transform_broadcaster.h
 *
 *  Created on: Oct 29, 2024
 *      Author: vrobot
 */

#ifndef MINIROS_INCLUDE_MINIROS_TF2_TRANSFORM_BROADCASTER_H_
#define MINIROS_INCLUDE_MINIROS_TF2_TRANSFORM_BROADCASTER_H_

#include <memory>

#include "miniros/macros.h"

#include "geometry_msgs/TransformStamped.hxx"

namespace miniros {

class NodeHandle;

namespace tf2 {
/** \brief This class provides an easy way to publish coordinate frame transform information.
 * It will handle all the messaging and stuffing of messages.  And the function prototypes lay out all the
 * necessary data needed for each message.  */

class MINIROS_DECL TransformBroadcaster{
public:
  /** \brief Constructor (needs a ros::Node reference) */
  TransformBroadcaster();
  TransformBroadcaster(miniros::NodeHandle& node);

  ~TransformBroadcaster();

  /** \brief Send a StampedTransform
   * The stamped data structure includes frame_id, and time, and parent_id already.  */
  //  void sendTransform(const StampedTransform & transform);

  /** \brief Send a vector of StampedTransforms
   * The stamped data structure includes frame_id, and time, and parent_id already.  */
  //void sendTransform(const std::vector<StampedTransform> & transforms);

  /** \brief Send a TransformStamped message
   * The stamped data structure includes frame_id, and time, and parent_id already.  */
  void sendTransform(const geometry_msgs::TransformStamped & transform);

  /** \brief Send a vector of TransformStamped messages
   * The stamped data structure includes frame_id, and time, and parent_id already.  */
  void sendTransform(const std::vector<geometry_msgs::TransformStamped> & transforms);

private:
  void init(int sendQueueLength = 100);

  struct Internal;
  std::unique_ptr<Internal> m_internal;
};

} // namespace tf2

} // namespace miniros

#endif /* MINIROS_INCLUDE_MINIROS_TF2_TRANSFORM_BROADCASTER_H_ */
