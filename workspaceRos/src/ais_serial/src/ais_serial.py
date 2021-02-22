#!/usr/bin/env python3

import rospy
from traj_generator.msg import TrajInfo
import serial
import ais
import time

SER_DEBUG = True

class Ais_Serial():

    def __init__(self, rosrate=10):


        self.pub_trajInfo = rospy.Publisher('/trajInfo', TrajInfo, queue_size=10)

        self.rate = rospy.Rate(rosrate)
        if not SER_DEBUG:
            self.ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1, parity=serial.PARITY_NONE)
            self.f = open("/home/corentin/Documents/Brave/workspaceRos/src/ais_serial/out.ais", 'w')
        else:
            self.f = open("/home/corentin/Documents/Brave/workspaceRos/src/ais_serial/out.ais", 'r')
            self.lines = self.f.readlines()
            self.i = 0

    def main(self):
        if not SER_DEBUG:
            line = self.ser.readline()
            if line != b'':
                rospy.loginfo(line.decode("ascii"))
                self.f.write(line.decode("ascii"))
                self.f.write("\n")
                try:
                    data = line.decode("ascii").split(",")[5]
                    parsed = ais.decode(data, 0)

                    trajectory_info = TrajInfo()

                    trajectory_info.nb_sec = int(rospy.get_time())
                    trajectory_info.latitude = parsed['x']
                    trajectory_info.longitude = parsed['y']
                    trajectory_info.vitesse_nd = parsed['sog']
                    trajectory_info.heading = parsed['true_heading']
                    trajectory_info.imo = parsed['mmsi']
                    self.pub_trajInfo.publish(trajectory_info)
                except Exception as e:
                    rospy.logerr(e)
        else:
            line = self.lines[self.i%len(self.lines)]
            self.i += 1
            if line != b'':
                try:
                    data = line.split(",")[5]
                    parsed = ais.decode(data, 0)
                    rospy.loginfo(parsed)

                    trajectory_info = TrajInfo()

                    trajectory_info.nb_sec = int(rospy.get_time())
                    trajectory_info.latitude = parsed['x']
                    trajectory_info.longitude = parsed['y']
                    trajectory_info.vitesse_nd = parsed['sog']
                    trajectory_info.heading = parsed['cog']
                    trajectory_info.imo = parsed['mmsi']
                    self.pub_trajInfo.publish(trajectory_info)
                    time.sleep(3)
                except Exception as e:
                    rospy.logerr(e)

if __name__ == "__main__":
    rospy.init_node('ais_serial', anonymous=True)

    ais_serial = Ais_Serial()

    while not rospy.is_shutdown():
        ais_serial.main()
        ais_serial.rate.sleep()