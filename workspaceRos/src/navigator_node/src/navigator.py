#!/usr/bin/env python

import rospy
import numpy as np
import utm

from numpy import cos, sin, arctan, arctan2, pi, cross, hstack, array, log, sign
from numpy.linalg import det, norm

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64MultiArray

from controller.msg import Line


r, zeta = 10, pi/4
delta_rmax = pi/3
EARTH_RADIUS = 6371000.
# Origine repere Guerledan
#lat0, lon0 = (48.198427, -3.014750)

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

    x = self.deg2rad(x)
    return self.sawtooth(pi/2 - x)


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


def rad2pwm(x, sail_name):
    """
    Bijection de [0, pi/2] sur [pwm_min, pwm_max] pour les voiles.
    [-pi/3,pi/3] pour rudder
    """
    if sail_name == "main":
        return (2/pi)*(self.pwm_max_main_sail - self.pwm_min_main_sail)*x + self.pwm_min_main_sail
    elif sail_name == "fore":
        return (2/pi)*(self.pwm_max_fore_sail - self.pwm_min_fore_sail)*x + self.pwm_min_fore_sail
    elif sail_name == "rudder":
        x = x+pi/3
        return (3/(2*pi))*(self.pwm_max_rudder - self.pwm_min_rudder)*x + self.pwm_min_rudder


class Navigator():

    def __init__(self, rosrate=10):
        self.waypoints = [] # [(lat, long), ... ]

        self.a = 0, 0
        self.b = 0, 10

        rospy.Publisher('/Line', Line, queue_size=32)

        rospy.Subscriber('/State', Pose2D, self._callback_state)
        rospy.Subscriber('/Waypoints', Float64MultiArray, self._callback_waypoints)

        self.rate = rospy.Rate(rosrate)

    def validate_wp(self, pos, a, b):
        return np.dot(b-a, pos-b) > 0

    def _callback_state(self, msg):
        pos = np.array([msg.x, msg.y])
        if self.validate_wp(pos, np.array(self.a), np.array(self.b)):
            pass

    def _callback_waypoints(self, msg):
        self.waypoints = []
        dt = np.array(msg.data)
        n = len(dt)
        for i in range(n//2):
            wp = WGS84_to_cart(dt[2*i], dt[2*i+1])
            self.waypoints.append(wp)

        print(self.waypoints)

    def main(self):
        # Guerledan
        # lxa, lya = -3.015067, 48.198905
        # lxb, lyb = -3.015603, 48.198301
        # lxc, lyc = -3.016049, 48.198762

        # Ty Colo
        lya, lxa = 48.431640, -4.615234
        lyb, lxb = 48.431352, -4.614569
        lyc, lxc = 48.431220, -4.615272

        xa, ya = WGS84_to_cart(lya, lxa)
        xb, yb = WGS84_to_cart(lyb, lxb)
        xc, yc = WGS84_to_cart(lyc, lxc)

        xm, ym = WGS84_to_cart(self.lym, self.lxm)

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
        #navigator.main()
        navigator.rate.sleep()
