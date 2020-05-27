#include <dmx_ros/dmx_node.hpp>

namespace dmx_node
{

DmxNode::DmxNode(ros::NodeHandle& nh_priv, ros::NodeHandle& nh_global)
    : nh_priv_(nh_priv), nh_global_(nh_global)
{
  ROS_INFO_STREAM("Successfully launched " << ros::this_node::getName());
}

DmxNode::~DmxNode()
{
}

}  // namespace path_action_relay
