#!/usr/bin/env python

import array
import serial
import termios
import fcntl
import time
import rospy
from dmx_ros.msg import Dmx


class DmxTest:
    def __init__(self):
        self.dmx_pub = rospy.Publisher("dmx_tx", Dmx, queue_size=1)

        rospy.Timer(rospy.Duration(1.0), self.send_frame)

        rospy.spin()

    def send_frame(self, event):
        # Example 1: Loop over all channels and put values from 0 to 255
        num_channels = 512
        dmx_msg = Dmx()
        values = bytearray(num_channels)
        for channel in range(0, num_channels):
            dmx_msg.channel.append(channel)
            values[channel] = channel//2
        dmx_msg.value = str(values)
        self.dmx_pub.publish(dmx_msg)

        # Example 2: Set some specific channels
        dmx_msg2 = Dmx()
        dmx_msg2.channel = [0, 12, 42, 511]
        dmx_msg2.value = str(bytearray([0, 128, 200, 255]))
        self.dmx_pub.publish(dmx_msg2)

if __name__ == '__main__':
    rospy.init_node("dmx_test")

    try:
        dmx_node = DmxTest()
    except rospy.ROSInterruptException:
        pass
