#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import std_msgs.msg
import sensor_msgs.msg
import serial

import time
import threading

import struct
from enum import IntEnum, unique
import sys

import binascii

import os
import math

import tf
import tf2_ros
import geometry_msgs.msg
import numpy as np

from std_msgs.msg import String

class UsRangeMsgCreator(object):
    __min_range = 0.010  # meters
    __max_range = 1.000  # meters
    __radiation_type = sensor_msgs.msg.Range.ULTRASOUND
    __field_of_view_degrees = 10  # I won't check whether it's between 0 and 90 here
    __field_of_view = __field_of_view_degrees * np.pi / 180.0
    #
    #  f = field_of_view (rad)
    #
    #              X
    #              ^
    #        \     |     /
    #         \  f |  f /
    #          \---|---/
    #           \  |  /
    #            \ | /
    #             \|/
    #  Y <---------*  Sensor
    #
    @classmethod
    def create_msg(cls, header=None):
        if (header):
            return sensor_msgs.msg.Range(radiation_type=cls.__radiation_type,
                                         field_of_view=cls.__field_of_view,
                                         min_range=cls.__min_range,
                                         max_range=cls.__max_range,
                                         header=header)
        else:
            return sensor_msgs.msg.Range(
                radiation_type=cls.__radiation_type,
                field_of_view=cls.__field_of_view,
                min_range=cls.__min_range,
                max_range=cls.__max_range,
                header=std_msgs.msg.Header(stamp=rospy.Time.now()))
        pass


class UsSensor(object):

    def __init__(self, id, addr, x, y, yaw, static_tf, msg_template, msg_pub):
        self.__id = id
        self.__addr = addr
        self.__x = x
        self.__y = y
        self.__yaw = yaw
        self.__static_tf = static_tf
        self.__msg_template = msg_template
        self.__msg_pub = msg_pub

    @property
    def id(self):
        return self.__id

    @property
    def addr(self):
        return self.__addr

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y

    @property
    def yaw(self):
        return self.__yaw

    @property
    def static_tf(self):
        return self.__static_tf

    @property
    def msg_template(self):
        return self.__msg_template

    @property
    def msg_pub(self):
        return self.__msg_pub


class Ultrasonic_KS114_485(object):

    def __init__(self, port_name, baudrate, noros=False):

        # 保存超声波传感器不同位置的编号id
        self.__pos_id = {}
        self.__init_param()

        self.__broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.__port_name = port_name
        self.__baudrate = baudrate

        self.__time_detection_latency = 0.5 #yuan lai shi 0.009
        self.__time_between_sensors = 0.51   #yuan lai shi 0.025
        assert (self.__time_between_sensors > self.__time_detection_latency)
        self.__time_to_next_sensor_after_read = self.__time_between_sensors - self.__time_detection_latency

        #self.__sensor_addrs = []
        self.__sensors = dict()

        #self.__serial_port = None

        # self.setup_static_tf()

        if not noros:
            self.__spin_thrd = None

            self.__start()
        pass

    def __init_param(self):
        list_pos = [
            "front_left", "front_right", "left_side_front", "left_side_back",
            "right_side_front", "right_side_back", "back_left", "back_right"
        ]
        for str_pos in list_pos:
            id = rospy.get_param("~" + str_pos, "-1")
            if not int(id) < 0:
                self.__pos_id[int(id)] = str_pos
        # print(self.__pos_id)

    def __send_static_tf(self):
        static_tf_list = map(lambda us: us.static_tf, self.__sensors.values())
        self.__broadcaster.sendTransform(static_tf_list)

    def __start(self):
        self.__spin_thrd = threading.Thread(target=self.__spin)

    def __spin(self):
        rospy.spin()

    def get_pos_by_id(self, sid):
        if sid not in self.__pos_id:
            return "undefined_" + str(sid)
        return self.__pos_id[sid]

    def get_tf_frame_by_id(self, sid):
        return "us_%d" % (sid)

    # def setup_all_static_tf(self):
    #     for sid in xrange(len(self.__sensor_addrs)):
    #         self.setup_sensor_static_tf(sid=sid)
    #     pass

    def setup_sensor_static_tf(self, sid, x, y, yaw):
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "base_footprint"
        static_transformStamped.child_frame_id = self.get_tf_frame_by_id(sid)

        static_transformStamped.transform.translation.x = x
        static_transformStamped.transform.translation.y = y
        static_transformStamped.transform.translation.z = 0.0

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]
        return static_transformStamped

    def connect(self):
        self.__serial_port = serial.Serial(self.__port_name,
                                           baudrate=115200,
                                           timeout=0.1,
                                           bytesize=8,
                                           parity='N',
                                           stopbits=1,
                                           xonxoff=0)
        if self.__serial_port.is_open:
            rospy.loginfo("port %s open OK" % (self.__port_name))
        else:
            rospy.logerr("port %s open FAILED" % (self.__port_name))
            sys.exit(-1)
        pass

    def add_sensor(self, sensor_addr, x, y, yaw):
        #self.__sensor_addrs.append(sensor_addr)
        id = self.number_of_sensors()  # len(self.__sensors)
        #static_tf, tf_broadcaster = self.setup_sensor_static_tf(id, x, y, yaw)
        static_tf = self.setup_sensor_static_tf(id, x, y, yaw)

        msg_template = UsRangeMsgCreator.create_msg()
        msg_template.header.frame_id = self.get_tf_frame_by_id(sid=id)
        msg_template.range = 0.0

        msg_pub = rospy.Publisher("~out/{}".format(self.get_pos_by_id(id)),
                                  sensor_msgs.msg.Range,
                                  queue_size=1)

        self.__sensors[id] = UsSensor(id=id,
                                      addr=sensor_addr,
                                      x=x,
                                      y=y,
                                      yaw=yaw,
                                      static_tf=static_tf,
                                      msg_template=msg_template,
                                      msg_pub=msg_pub)

        self.__send_static_tf()

    def number_of_sensors(self):
        #return len(self.__sensor_addrs)
        return len(self.__sensors)

    def read_two_bytes(self, sleep_duration):
        buf = ""
        size_remain = 7 #yuan ben 2
        sleep_time = 0
        while size_remain and (not rospy.is_shutdown()):
            ack = self.__serial_port.read(size_remain)
            if not ack:
                time.sleep(sleep_duration)
                sleep_time += sleep_duration
                print("sleep time")
            else:
                buf += ack
                size_remain -= len(buf)
            pass
        else:
            if rospy.is_shutdown():
                sys.exit()
            pass
        pass

        #print('{}'.format(buf))
        i = 1
        result = ""
        for re in buf:
            if (i == 4 or i == 5):
                result += re
            i += 1

        return result, sleep_time

        #return buf, sleep_time


    def query_all_sensors(self):
        pub = rospy.Publisher('/obstacle/alarm', String, queue_size=10)

        #for sid in xrange(len(self.__sensor_addrs)):
        print self.__sensors.keys()
        for sid in self.__sensors.keys():
            #saddr = self.__sensor_addrs[sid]
            sensor_ = self.__sensors[sid]
            assert (isinstance(sensor_, UsSensor))
            saddr = sensor_.addr

            #req = struct.pack("3B", saddr, 0x02, 0x0f) #这里可以修改距离
            req = struct.pack("8B", 0x01, 0x03, 0x01, 0x01, 0x00, 0x01, 0xD4, 0x36)
            #req = struct.pack("8B", 0x01, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85, 0xF6)
            self.__serial_port.write(data=req)
            #print("write finish")
            #print binascii.hexlify(req)
            time.sleep(self.__time_detection_latency)

            #ack = self.__serial_port.read(2)
            ack, sleep_time = self.read_two_bytes(0.00001) #0.01
            #print("read finish")

            if not ack:
                print "no response from [%1d|%02x]" % (sid, saddr)
                continue
            pass

            #print binascii.hexlify(ack)

            #ack, sleep_time = self.read_two_bytes(0.01)
            #print binascii.hexlify(ack), sleep_time, "s"
            
            #origin code
            
            ack_data = struct.unpack(">H", ack)[0]
            #print ack_data, ack_data / 58.0 , "cm"
            detect_range = ack_data / 58.0 * 0.01 * 5.5  # according to the protocol
    
            #now code
            """
            ack_data = int(ack, 10)
            detect_range = ack_data
            """

            print "[%1d|%02x]" % (sid, saddr), detect_range, "m"

            ########################################################################
            if (detect_range <= 0.5):
                #print("wh2")
                if (sid == 0 or sid == 1):
                    alaString = "front"
                elif (sid == 7 or sid == 6):
                    alaString = "left"
                elif (sid == 2 or sid == 3):
                    alaString = "right"
                else:
                    alaString = "back"
                pub.publish(alaString)
            ########################################################################
            msg = sensor_.msg_template
            msg.range = detect_range
            if sensor_.msg_pub.get_num_connections():
                print("wh")
                sensor_.msg_pub.publish(msg)
            pass

            time.sleep(self.__time_to_next_sensor_after_read)
        pass

    def change_sensor_id(self, sid_before, sid_after):
        req = struct.pack("3B", sid_before, 0x02, 0x9a)
        self.__serial_port.write(data=req)
        time.sleep(0.001)
        req = struct.pack("3B", sid_before, 0x02, 0x92)
        self.__serial_port.write(data=req)
        time.sleep(0.001)
        req = struct.pack("3B", sid_before, 0x02, 0x9e)
        self.__serial_port.write(data=req)
        time.sleep(0.001)
        req = struct.pack("3B", sid_before, 0x02, sid_after)
        self.__serial_port.write(data=req)
        time.sleep(0.1)
        return
