#!/usr/bin/env python

import rospy
import numpy as np
import utm

from numpy import cos, sin, arctan, arctan2, pi, cross, hstack, array, log, sign
from numpy.linalg import det, norm

from navigator_node.msg import Waypoints
from geometry_msgs.msg import Pose2D

from proj_tools import *


r, zeta = 10, pi/4
delta_rmax = pi/3
EARTH_RADIUS = 6371000.
# Origine repere Guerledan
#lat0, lon0 = (48.198427, -3.014750)

# Origine repere Ty Colo
lat0, lon0 = (48.431775, -4.615529)


class Navigator():

    def __init__(self, rosrate=10):
        # self.pub_pwm = rospy.Publisher('/Command', Command, queue_size=10)

        self.waypoints = [] # [(lat, long), ... ]

        rospy.Publisher('/Waypoints', Waypoints, queue_size=32)

        rospy.Subscriber('/State', Pose2D, self._callback_state)

        self.rate = rospy.Rate(rosrate)


    def _callback_state(self, msg):
        pass

    # Input: cartesian coordinates of points a and b to follow line a-b
    # 	cartesian coordinates of the boat, m.

    # xte = det(hstack((b-a,m-a)))/norm(b-a)

    def main(self):
        # Guerledan
        # lxa, lya = -3.015067, 48.198905
        # lxb, lyb = -3.015603, 48.198301
        # lxc, lyc = -3.016049, 48.198762

        # Ty Colo
        lya, lxa = 48.431640, -4.615234
        lyb, lxb = 48.431352, -4.614569
        lyc, lxc = 48.431220, -4.615272

        xa, ya = self.WGS84_to_cart(lya, lxa)
        xb, yb = self.WGS84_to_cart(lyb, lxb)
        xc, yc = self.WGS84_to_cart(lyc, lxc)

        xm, ym = self.WGS84_to_cart(self.lym, self.lxm)

        a = array([[xa], [ya]])
        b = array([[xb], [yb]])
        c = array([[xc], [yc]])

        m = array([[xm], [ym]])

        if abs(m[0]-b[0]) < 5 and abs(m[1]-b[1]) < 5:
            print("ligne bc")
            self.FirstLine = False
            self.pt1 = b
            self.pt2 = c

        elif abs(m[0]-c[0]) < 5 and abs(m[1]-c[1]) < 5:
            print("ligne ca")
            self.FirstLine = False
            self.pt1 = c
            self.pt2 = a

        if self.FirstLine:
            print("ligne ab")
            self.pt1 = a
            self.pt2 = b


if __name__ == "__main__":
    rospy.init_node('navigator', anonymous=True)
    navigator = Navigator()
    while not rospy.is_shutdown():
        navigator.main()
        navigator.rate.sleep()
