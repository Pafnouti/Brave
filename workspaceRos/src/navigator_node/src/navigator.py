#!/usr/bin/python3

import rospy
import numpy as np
import utm

from numpy import cos, sin, arctan, arctan2, pi, cross, hstack, array, log, sign
from numpy.linalg import det, norm

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64MultiArray, Int32

from controller.msg import Line


r, zeta = 10, pi/4
delta_rmax = pi/3
EARTH_RADIUS = 6371000.
# Origine repere Guerledan
# lat0, lon0 = (48.198427, -3.014750)

# Origine repere Ty Colo
lat0, lon0 = (48.431775, -4.615529)


def sawtooth(x):
    """Deal with 2*PI modulo

    Input:
    ------
    x: rad
    """
    return (x+pi) % (2*pi)-pi


def deg2rad(x):
    """
    Conversion deg to rad
    """
    return x*2*pi/360


def north2east(x):
    """
    Input:
    ------
    Wind direction in degres, 0 is pointing north
    clockwise rotation

    Return:
    -------
    Angle of the wind in rad, in trigonometric circle
    """

    x = deg2rad(x)
    return sawtooth(pi/2 - x)


def WGS84_to_cart(lat, lon):
    """
    Input: gps coord decimal lat, lon
    Return: cartesian coord x, y with (lat0, lon0) the origin
    """
    x = (pi/180.)*EARTH_RADIUS*(lon-lon0)*cos((pi/180.)*lat)
    y = (pi/180.)*EARTH_RADIUS*(lat-lat0)
    return x, y


def cart_to_WGS84(x, y):
    """
    Input: cartesian coord x, y with (lat0, lon0) the origin
    Return: gps coord decimal lat, lon
    """
    EPSILON = 0.00000000001
    lat = y*180./pi/EARTH_RADIUS+lat0
    if abs(lat-90.) < EPSILON or abs(lat+90.) < EPSILON:
        lon = 0
    else:
        lon = (x/EARTH_RADIUS)*(180./pi)/cos((pi/180.)*(lat))+lon0
    return lat, lon


class Navigator():

    def __init__(self, rosrate=10):
        self.waypoints = []  # [(lat, long), ... ]
        self.waypoint_index = 0

        self.a = [0, 0]
        self.b = [0, 10]
        self.line = Line()
        self.line.xa = self.a[0]
        self.line.ya = self.a[1]
        self.line.xb = self.b[0]
        self.line.yb = self.b[1]
        self.m = [0, 0]

        self.pub_line = rospy.Publisher('/Line', Line, queue_size=32)
        self.pub_currIndex = rospy.Publisher('/Current_Target', Int32, queue_size=8)

        rospy.Subscriber('/State', Pose2D, self._callback_state)
        rospy.Subscriber('/Waypoints', Float64MultiArray,
                         self._callback_waypoints)

        self.rate = rospy.Rate(rosrate)

    def validate_wp(self, m, a, b):
        if m.shape == (2,):
            return np.dot(b-a, m-b) > 0
        else:
            return False

    def _callback_state(self, msg):
        self.m = np.array([msg.x, msg.y])
        
        if self.validate_wp(self.m, np.array(self.a), np.array(self.b)):
            self.a = self.b
            self.waypoint_index += 1
            self.b = self.waypoints[self.waypoint_index]
            self.line = Line()
            self.line.xa = self.a[0]
            self.line.ya = self.a[1]
            self.line.xb = self.b[0]
            self.line.yb = self.b[1]
            self.pub_line.publish(self.line)

    def _callback_waypoints(self, msg):
        self.waypoints = []
        dt = np.array(msg.data)
        n = len(dt)
        for i in range(n//2):
            wp = WGS84_to_cart(dt[2*i], dt[2*i+1])
            self.waypoints.append(wp)
        self.a = self.m
        self.b = self.waypoints[0]
        self.waypoint_index = 0

        self.line = Line()
        self.line.xa = self.a[0]
        self.line.ya = self.a[1]
        self.line.xb = self.b[0]
        self.line.yb = self.b[1]
        self.pub_line.publish(self.line)

    def main(self):
        msg = Int32()
        msg.data = self.waypoint_index
        self.pub_currIndex.publish(msg)


if __name__ == "__main__":
    rospy.init_node('navigator', anonymous=True)
    navigator = Navigator()

    lat0 = rospy.get_param("/origin/lat")
    lon0 = rospy.get_param("/origin/lon")

    while not rospy.is_shutdown():
        navigator.main()
        navigator.rate.sleep()
        #navigator.pub_line.publish(navigator.line)
