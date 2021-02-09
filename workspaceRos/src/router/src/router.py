#!/usr/bin/env python

import rospy

from routeur import *

from controller.msg import Line, Wind
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64MultiArray, Int32



lon0, lat0 = rospy.get_param('origin')['lon'], rospy.get_param('origin')['lat']

EARTH_RADIUS = 6371000.

def WGS84_to_cart(lat, lon):
    """
    Input: gps coord decimal lat, lon
    Return: cartesian coord x, y with (lat0, lon0) the origin
    """
    x = (np.pi/180.)*EARTH_RADIUS*(lon-lon0)*np.cos((np.pi/180.)*lat)
    y = (np.pi/180.)*EARTH_RADIUS*(lat-lat0)
    return x, y

def cart_to_WGS84(x, y):
    """
    Input: cartesian coord x, y with (lat0, lon0) the origin
    Return: gps coord decimal lat, lon
    """
    EPSILON = 0.00000000001
    lat = y*180./np.pi/EARTH_RADIUS+lat0
    if np.abs(lat-90.) < EPSILON or np.abs(lat+90.) < EPSILON:
        lon = 0
    else:
        lon = (x/EARTH_RADIUS)*(180./np.pi)/np.cos((np.pi/180.)*(lat))+lon0
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
        rospy.Subscriber('/Wind', Wind, self._callback_wind)


    def _callback_target(self, msg):
        self.target = (msg.x, msg.y)
        self.got_target = True
        

    def _callback_state(self, msg):
        self.state = np.array([msg.x, msg.y])

    def _callback_routing(self, msg):
        self.activated = msg.data
        print('cb routage')
        self.main()

    def _callback_wind(self, msg):
        self.routeur.set_weahter(msg.wind_speed, msg.wind_direction)
        #print('set weather to : ', msg.wind_speed, msg.wind_direction)
    
    def main(self):
        if self.activated and self.got_target:
            print('routing...')

            B = WGS84_to_cart(self.target[0], self.target[1])
            traj, iso = self.routeur.run(self.state, B)
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
