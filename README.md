# dmx_ros
ROS node to talk with DMX devices over an RS485 serial interface.
Supported on Linux only.

# Credits
This node makes use of the dmx485 library:
https://sourceforge.net/projects/dmx485/
Mike Bourgeous <nitrogen at users.sourceforge.net>
  - Original dmx485 author.

And is losely based on a pyton version:
https://github.com/ernestmc/ros-dmx
Ernesto Corbellini <ernestmc at github.com>
  - Original ros-dmx author.

# Usage
The node reacts both to streaming msgs on the dmx_tx topic as service calls to SetChannel to set dmx channel values.
The GetChannel service is available to request current channel statusses.

All messages and services support multiple dmx channels with a maximum of 512.

Channels are adressed 1 to 512.
Values are of type uint8 and range from 0 to 255. Please note the usage of an uint8 array for the values! rospy for example treats this as a byte type!

# Examples
Command line rostopic pub:
```
rostopic pub /dmx_node/dmx_tx dmx_ros/Dmx "channel: [1, 2, 3] value: [0, 0, 0]"
```
Will set channels 1,2 and 3 to a maximum value.

Command line service calls:
```
rosservice call /dmx_node/set_channel "channel: [1,2,3] value: [255,255,255]"
```
Will set channels 1,2 and 3 to a maximum value.

```
rosservice call /dmx_node/get_channel "channel: [1,2,3]"
```
Will reply the current value for channels 1, 2 and 3.

Python minimal example:
```
import rospy
from dmx_ros.msg import Dmx

rospy.init_node("dmx_test")

dmx_pub = rospy.Publisher("dmx_node/dmx_tx", Dmx, queue_size=1)

dmx_msg = Dmx()
dmx_msg.channel = [1, 2, 3]
dmx_msg.value = str(bytearray([255, 255, 255]))
# or:
dmx_msg.value = '\xFF\xFF\xFF'
dmx_pub.publish(dmx_msg)
```
