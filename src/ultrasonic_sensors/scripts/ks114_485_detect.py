#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy

from ks114_485 import Ultrasonic_KS114_485

import numpy as np


if __name__ == "__main__":

    import setproctitle
    import os
    import argparse
    setproctitle.setproctitle(os.path.basename(__file__))

    rospy.init_node("ultrasonic_ks114_485")

    port_name = rospy.get_param("~port", "/dev/ttyUSB0")
    baudrate = rospy.get_param("~baudrate", "115200")

    rospy.loginfo("Ultrasonic KS114 485:  port[{}]  baudrate[{}]".format(port_name, baudrate))

    parser = argparse.ArgumentParser(description="Modify KS114-485 address.")
    parser.add_argument(dest="addr",
                        help="address of sensor",
                        default="0x01")

    us_ks114_485 = Ultrasonic_KS114_485(port_name=port_name, baudrate=baudrate)



    import sys
    if len(sys.argv) > 1:
        args = parser.parse_args()
        addr_0 = int(args.addr, 16)
        print ("addr: %02x" % addr_0)
        us_ks114_485.add_sensor(addr_0, x=0.25, y=0.0, yaw=0.0)
    else:
        us_ks114_485.add_sensor(0xd0, x= 0.386, y= 0.08475, yaw=0.0)
        us_ks114_485.add_sensor(0xd2, x= 0.386, y=-0.08475, yaw=0.0)
        us_ks114_485.add_sensor(0xd4, x= 0.110, y=-0.28225, yaw= - np.pi / 2.0 )
        us_ks114_485.add_sensor(0xd6, x=-0.290, y=-0.28225, yaw= - np.pi / 2.0 )
        us_ks114_485.add_sensor(0xd8, x=-0.408, y=-0.16250, yaw=   np.pi       )
        us_ks114_485.add_sensor(0xda, x=-0.408, y= 0.16250, yaw=   np.pi       )
        us_ks114_485.add_sensor(0xdc, x=-0.290, y= 0.28225, yaw=   np.pi / 2.0 )
        us_ks114_485.add_sensor(0xde, x= 0.110, y= 0.28225, yaw=   np.pi / 2.0 )
    pass



    us_ks114_485.connect()

    #us_ks114_485.change_sensor_id(0xe8, 0xd0)

    #for i in xrange(10):
    while not rospy.is_shutdown():
        us_ks114_485.query_all_sensors()
    pass
