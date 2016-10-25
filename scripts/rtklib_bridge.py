#!/usr/bin/env python
# vim:fileencoding=utf-8

import rospy
import socket
import time
import datetime
from sensor_msgs.msg import NavSatFix


class RtklibBridge:
    def __init__(self):
        rospy.init_node('RtklibBridge', anonymous=True)

        self.pub = rospy.Publisher('~gps_solution', NavSatFix, queue_size=10)

        self.server_address =  rospy.get_param('~rtklib_server_address', '127.0.0.1')
        self.server_port = rospy.get_param('~rtklib_server_port', 52001)

        self.covariance_table =[0, 3, 2, 0, 1]

    def connect(self):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client.connect((self.server_address, self.server_port))
        except socket.error:
            raise
        return True

    def gpst2time(self, week, sec):
        gpst0 =  t = time.mktime(datetime.datetime(1980, 1, 6, 0, 0, 0).timetuple())

        t += 86400 * 7 * int(week) + int(float(sec))

        return t

    def get_solution(self):
        try:
            receive = self.client.recv(4096)
        except socket.error:
            raise

        if not receive:
            raise ValueError("Not received")

        # receive example
        # 1918 352534.000   35.674540574  139.531064244    94.6605   5   9   3.3592   2.1315   7.6682  -0.8273   1.5609  -2.0968   0.00    0.0

        receive_split = receive.split()
        if len(receive_split) is not 15:
            raise ValueError("Receive data length is not 15")

        print receive_split

        t = self.gpst2time(receive_split[0], receive_split[1])
        latitude = float(receive_split[2])
        longtitude = float(receive_split[3])

        ret = NavSatFix()
        ret.header.stamp = rospy.Time(float(t))
        ret.latitude = latitude
        ret.longitude = longtitude

        ret.position_covariance_type = self.covariance_table[int(receive_split[5])-1]

        return ret

    def spin(self):
        rate = rospy.Rate(10)

        try:
            pub_nav_sat = self.get_solution()
        except:
            raise

        self.pub.publish(pub_nav_sat)

        rate.sleep()
        return True

    def close_connection(self):
        self.client.close()


if __name__ == '__main__':
    bridge = RtklibBridge()
    is_connection = False

    while not rospy.is_shutdown():
        if not is_connection:
            try:
                is_connection = bridge.connect()
            except socket.error as e:
                rospy.logerr(e)
                is_connection = False

        else:
            try:
                bridge.spin()
            except socket.error as e:
                rospy.logerr(e)
                bridge.close_connection()
                is_connection = False
            except ValueError as e:
                rospy.logerr(e)
                bridge.close_connection()
                is_connection = False
            except rospy.ROSInterruptException:
                bridge.close_connection()
                pass
