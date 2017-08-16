import serial
import termios
import fcntl
import time
import rospy

class DmxDriver(object):
  def __init__(self, com_port="/dev/ttyUSB0"):
    self.dmx_frame = [0] * 512
    self.BAUD_RATE = 250000
    self.serial_device = serial.Serial(com_port, baudrate=self.BAUD_RATE, stopbits=serial.STOPBITS_TWO)
    self.TIOCSBRK = getattr(termios, 'TIOCSBRK', 0x5427)
    self.TIOCCBRK = getattr(termios, 'TIOCCBRK', 0x5428)

  def __del__(self):
    self.serial_device.close()

  def set_channel(self, channel, value):
    """
    Set a channel to a specific value.
    :param channel: A channel value in the range [0; 511]
    :param value: A value in the range [0; 255]
    :return: None
    """
    channel -= 1
    if channel > 511:
      channel = 511
    elif channel < 0:
      channel
    self.dmx_frame[channel] = value

  def zero_dmx_frame(self):
    """
    Put all channels of the DMX frame to zero.
    :return: None
    """
    for i in range(512):
      self.dmx_frame[i] = 0

  def send_frame(self):
    """
    Send a DMX frame.
    :return: None
    """
    # Convert frame to string
    # Channel 0 is the break condition
    serial_data = chr(0)
    for i in self.dmx_frame:
      serial_data += chr(i)
    # Manually emulate the serial break condition
    fcntl.ioctl(self.serial_device.fd, self.TIOCSBRK)
    time.sleep(100 / 1e6)
    fcntl.ioctl(self.serial_device.fd, self.TIOCCBRK)
    time.sleep(100 / 1e6)
    self.serial_device.write(serial_data)

if __name__ == '__main__':
  dmx = DmxDriver()
  a = 0
  s = 1
  #for i in range(10):
  while 1:
    dmx.set_channel(1, 128)
    dmx.send_frame()
    time.sleep(0.02)
  time.sleep(60)
  while 1:
    a += s
    if a > 255:
      a = 255
      s = -s
    if a < 0:
      a = 0
      s = -s
    dmx.set_channel(1, a)
    dmx.set_channel(3, a)
    dmx.send_frame()
    time.sleep(0.02)
