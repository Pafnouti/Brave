#!/usr/bin/env python

import rospy

from routeur import *

from controller.msg import Line
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64MultiArray, Int32



lon0, lat0 = rospy.get_param('origin')['lon'], rospy.get_param('origin')['lat']

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



class Router():
    def __init__(self, rosrate=0.1):
        self.routeur = Routeur()

        self.rate = rospy.Rate(rosrate)

        self.activated = False
        self.got_target = False
        self.target = (0, 0)
        self.state = (0, 0)

        
        self.pub_wps = rospy.Publisher('/Waypoints', Float64MultiArray, queue_size=32)

        rospy.Subscriber('/Routing', Bool, self._callback_routing)
        rospy.Subscriber('/Target', Pose2D, self._callback_target)
        rospy.Subscriber('/State', Pose2D, self._callback_state)


    def _callback_target(self, msg):
        self.target = (msg.x, msg.y)
        self.got_target = True
        

    def _callback_state(self, msg):
        self.state = np.array([msg.x, msg.y])

    def _callback_routing(self, msg):
        self.activated = msg.data
        print('cb routage')
        self.main()
    
    def main(self):
        if self.activated and self.got_target:
            print('routing...')
            traj, iso = self.routeur.run(self.state, self.target)
            self.wps = Float64MultiArray()
            wps_tmp = []
            for i in range(len(traj[0])):
                x, y = traj[0][i], traj[1][i]
                lat, lon = cart_to_WGS84(x, y)
                wps_tmp.append(lat)
                wps_tmp.append(lon)
            self.wps.data = wps_tmp
            self.pub_wps.publish(self.wps)
        else:
            print('not routing')

    


    

if __name__ == "__main__":
    rospy.init_node('router', anonymous=True)
    router = Router(rosrate=0.01)

    while not rospy.is_shutdown():
        router.main()
        router.rate.sleep()
