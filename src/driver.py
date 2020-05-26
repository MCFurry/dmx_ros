#!/usr/bin/env python

import array
import serial
import termios
import fcntl
import time
import rospy
from dmx_ros.srv import SetChannel, SetChannelResponse, GetChannel, GetChannelResponse


class DmxDriver:
    def __init__(self):
        self.com_port = rospy.get_param('~com_port', '/dev/ttyUSB0')
        self.rate = rospy.get_param('~rate', 50)
        self.BAUD_RATE = 250000
        try:
            self.serial_device = serial.Serial(self.com_port, baudrate=self.BAUD_RATE, stopbits=serial.STOPBITS_TWO)
        except serial.SerialException, e:
            rospy.logerr(str(e))
            rospy.signal_shutdown(str(e))

        self.TIOCSBRK = getattr(termios, 'TIOCSBRK', 0x5427)
        self.TIOCCBRK = getattr(termios, 'TIOCCBRK', 0x5428)
        self.dmx_frame = [0] * 512

        self.services = [
            rospy.Service("~set_channel", SetChannel, self.set_channel_srv_cb),
            rospy.Service("~get_channel", GetChannel, self.get_channel_srv_cb)
        ]
        rospy.Timer(rospy.Duration(1.0/self.rate), self.send_frame)

        rospy.spin()

    def __del__(self):
        self.serial_device.close()

    def set_channel_srv_cb(self, req):
        # Python treats unit8[] as bytes, hence convert them that way
        # http://wiki.ros.org/msg#Fields
        values = array.array('B', req.value)
        if len(req.channel) == len(values):
            for channel, value in zip(req.channel, values):
                if channel >= 0 and channel < 512:
                    self.dmx_frame[channel] = value
                else:
                    rospy.logerr("set_channel(): channel {} out of bounds!".format(channel))
                    return SetChannelResponse(success=False)
            return SetChannelResponse(success=True)
        else:
            rospy.logerr("Number of channels should be the same as number of values!")
            return SetChannelResponse(success=False)

    def get_channel_srv_cb(self, req):
        resp = GetChannelResponse()
        resp.success = True
        for channel in req.channel:
            if channel >= 0 and channel < 512:
                resp.value += chr(self.dmx_frame[channel])
            else:
                rospy.logerr("get_channel(): channel {} out of bounds!".format(channel))
                resp.success = False
        return resp

    def send_frame(self, event):
        """
        Send a DMX frame.
        :return: None
        """
        # Convert frame to string
        # Channel 0 is the break condition
        serial_data = chr(0)
        for i in self.dmx_frame:
          serial_data += str(i)
        try:
            # Manually emulate the serial break condition
            fcntl.ioctl(self.serial_device.fd, self.TIOCSBRK)
            time.sleep(100 / 1e6)
            fcntl.ioctl(self.serial_device.fd, self.TIOCCBRK)
            time.sleep(100 / 1e6)
            self.serial_device.write(serial_data)
        except IOError, e:
            rospy.logerr(str(e))
            rospy.signal_shutdown(str(e))

if __name__ == '__main__':
    rospy.init_node("dmx_driver")

    try:
        dmx_node = DmxDriver()
    except rospy.ROSInterruptException:
        pass
