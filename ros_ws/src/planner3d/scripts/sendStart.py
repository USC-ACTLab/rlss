#!/usr/bin/env python

from udp_multicast import UdpMulticastSender

if __name__ == '__main__':
  sender = UdpMulticastSender()
  for i in range(0, 100):
    sender.send("startTrajectory")
