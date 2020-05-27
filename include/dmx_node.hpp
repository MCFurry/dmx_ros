#pragma once

// ROS
#include <ros/ros.h>

namespace dmx_node {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class DmxNode
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  DmxNode(ros::NodeHandle& nh_priv, ros::NodeHandle& nh_global);

  /*!
   * Destructor.
   */
  virtual ~DmxNode();

 private:

  //! node handles
  ros::NodeHandle& nh_priv_;
  ros::NodeHandle& nh_global_;

};

} /* namespace */
