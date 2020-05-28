#pragma once

// ROS
#include <ros/ros.h>
#include <dmx_ros/Dmx.h>
#include <dmx_ros/SetChannel.h>
#include <dmx_ros/GetChannel.h>

// DMX485 library
extern "C" {
  #include <dmx_ros/dmx485.h>
}

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

  void updateTimerCB(const ros::TimerEvent&);
  void dmxMsgCb(const dmx_ros::Dmx::ConstPtr& msg);
  bool setChanCB(dmx_ros::SetChannel::Request& request, dmx_ros::SetChannel::Response& response);
  bool getChanCB(dmx_ros::GetChannel::Request& request, dmx_ros::GetChannel::Response& response);

  /* ROS timer for periodic updates */
  ros::Timer updateTimer;

  /* ROS services */
  ros::ServiceServer setChanService;
  ros::ServiceServer getChanService;

  /* ROS subscribers */
  ros::Subscriber dmxSubscriber;

  // Global vars
  dmx_state *dmx;
  std::string com_port;
  float rate;

};

} /* namespace */
