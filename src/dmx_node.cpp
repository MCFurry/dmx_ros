#include <dmx_ros/dmx_node.hpp>

namespace dmx_node
{

DmxNode::DmxNode(ros::NodeHandle& nh_priv, ros::NodeHandle& nh_global)
    : nh_priv_(nh_priv), nh_global_(nh_global)
{
  nh_priv_.param<std::string>("com_port", com_port, "/dev/ttyUSB0");
  nh_priv_.param<float>("rate", rate, 25.0);

  dmx = dmx_open(com_port.c_str());
	if(dmx == NULL) {
		fprintf(stderr, "Error initializing DMX\n");
		ros::shutdown();
	}

  /* Setup timer for streaming dmx frames */
  updateTimer = nh_priv_.createTimer(rate, &DmxNode::updateTimerCB, this);

  /* ROS services */
  setChanService = nh_priv_.advertiseService("set_channel", &DmxNode::setChanCB, this);
  getChanService = nh_priv_.advertiseService("get_channel", &DmxNode::getChanCB, this);

  /* ROS subscribe topics */
  dmxSubscriber = nh_global.subscribe<dmx_ros::Dmx>("dmx_tx", 1, &DmxNode::dmxMsgCb, this);

  ROS_INFO_STREAM("Successfully launched " << ros::this_node::getName());
}

DmxNode::~DmxNode()
{
  memset(dmx->dmx_values, 0, 512);
  send_state(dmx);
  dmx_close(dmx);
}

void DmxNode::updateTimerCB(const ros::TimerEvent& event)
{
  send_state(dmx);
}

void DmxNode::dmxMsgCb(const dmx_ros::Dmx::ConstPtr& msg)
{
  if (msg->channel.size() == msg->value.size())
  {
    for (int idx = 0; idx < msg->channel.size(); ++idx)
    {
      if (msg->channel[idx] > 0 and msg->channel[idx] <= 512)
        dmx->dmx_values[msg->channel[idx]-1] = msg->value[idx];
      else
        ROS_ERROR("Channel %d is out of bounds! (min=1, max=512)", msg->channel[idx]);
    }
  }
  else
    ROS_ERROR("Number of channels should be the same as number of values! Got: %lu channels and %lu values ",
               msg->channel.size(), msg->value.size());
}

bool DmxNode::setChanCB(dmx_ros::SetChannel::Request& request,
                        dmx_ros::SetChannel::Response& response)
{
  if (request.channel.size() == request.value.size())
  {
    for (int idx = 0; idx < request.channel.size(); ++idx)
    {
      if (request.channel[idx] > 0 and request.channel[idx] <= 512)
        dmx->dmx_values[request.channel[idx]-1] = request.value[idx];
      else
      {
        ROS_ERROR("Channel %d is out of bounds! (min=1, max=512)", request.channel[idx]);
        response.success = false;
        return true;
      }
    }
    response.success = true;
  }
  else
  {
    ROS_ERROR("Number of channels should be the same as number of values! Got: %lu channels and %lu values ",
               request.channel.size(), request.value.size());
    response.success = false;
  }
  return true;
}

bool DmxNode::getChanCB(dmx_ros::GetChannel::Request& request,
                        dmx_ros::GetChannel::Response& response)
{
  response.success = true;
  for (int idx = 0; idx < request.channel.size(); ++idx)
  {
    if (request.channel[idx] > 0 and request.channel[idx] <= 512)
      response.value.push_back(dmx->dmx_values[request.channel[idx]-1]);
    else
    {
      ROS_ERROR("Channel %d is out of bounds! (min=1, max=512)", request.channel[idx]);
      response.success = false;
    }
  }
  return true;
}

}  // namespace dmx_node

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dmx_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // Create DmxNode class
  dmx_node::DmxNode n(nh_priv, nh);

  ros::spin();
  return 0;
}
