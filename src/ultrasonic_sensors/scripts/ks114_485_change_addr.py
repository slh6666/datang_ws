#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from ks114_485 import Ultrasonic_KS114_485


if __name__ == "__main__":

    import setproctitle
    import os
    import os
    import argparse

    setproctitle.setproctitle(os.path.basename(__file__))

    parser = argparse.ArgumentParser(description="Modify KS114-485 address.")

    parser.add_argument('-v', '--version', action='version',
                        version='%(prog)s version : v 0.01', help='show the version')

    # --- 参数默认值示例 ---
    # parser.add_argument('--debug', '-d', action='store_true',
    #                     help='show the version',
    #                     default=False)

    # # 互斥组：该组内的参数互斥，只能设置其中一个
    # group = parser.add_mutually_exclusive_group()
    # group.add_argument("-x", "--to-xml", action="store_true",  # 有设置的时候，对应变量置为 True
    #                    help="Convert JSON map to XML format",
    #                    default=False)  # 没有设置的时候，对应变量默认为 False
    # group.add_argument("-j", "--to-json", action="store_true",  # 有设置的时候，对应变量置为 True
    #                    help="Convert XML map to JSON format",
    #                    default=False)  # 没有设置的时候，对应变量默认为 False

    parser.add_argument("port",
                        help="serial port",
                        default="/dev/ttyUSB0")

    parser.add_argument("baudrate",
                        help="baudrate",
                        default="115200",
                        choices=["9600", "115200"])

    parser.add_argument(dest="addr_before",
                        help="address before change",
                        default="0xe8")

    parser.add_argument(dest="addr_after",
                        help="address after change")

    args = parser.parse_args()

    print ("Arguments: %s" % args)

    print ("port       : %s" % args.port)
    print ("baudrate   : %d" % int(args.baudrate))
    print ("addr_before: %02x" % int(args.addr_before, 16))
    print ("addr_after : %02x" % int(args.addr_after, 16))

    us_ks114_485 = Ultrasonic_KS114_485(port_name=args.port, baudrate=int(args.baudrate), noros=True)
    #us_ks114_485.add_sensor(0xd0, x=0.25, y=0.0, yaw=0.0)

    us_ks114_485.connect()

    us_ks114_485.change_sensor_id(int(args.addr_before, 16), int(args.addr_after, 16))
