# dmx_ros
ROS node to talk with DMX devices over a RS485 serial interface.
Supported on Linux only.

# Credits
This node makes us of the dmx485 library:
https://sourceforge.net/projects/dmx485/
Mike Bourgeous <nitrogen at users.sourceforge.net>
  - Original dmx485 author.

And is losely based on a pyton version:
https://github.com/ernestmc/ros-dmx
Ernesto Corbellini <ernestmc at github.com>
  - Original ros-dmx author.

# Usage
The node reacts both to streaming msgs on the dmx_tx topic as service calls to SetChannel.
The GetChannel service is available to request current channel statusses.

All messages and services support multiple dmx channels with a maximum of 512.