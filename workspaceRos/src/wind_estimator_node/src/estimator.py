#!/usr/bin/env python3

import rospy
import numpy as np

from controller.msg import Meteo
from controller.msg import Wind

from collections import deque
from statistics import mean

class WindEstimator():

    def __init__(self, rosrate=1):
        self.meanDuration = 600; # number of values, duration depends on the weather station's sampling rate; assuming 1Hz here
        self.wind_pub = rospy.Publisher('/Wind', Wind, queue_size=10)

        rospy.Subscriber('/ublox/WIMDA', Meteo, self._callback_meteo)


        self.rate = rospy.Rate(rosrate)

        self.TWD_list = deque()
        self.windSpeed_list = deque()
        self.initDeque = 0;

    def sawtooth(self, x):
        """Deal with 2*PI modulo

        Input:
        ------
        x: rad 
        """
        return (x+np.pi) % (2*np.pi)-np.pi

    def deg2rad(self, x):
        """
        Conversion deg to rad
        """
        return x*2*np.pi/360

    def north2east(self, x):
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
        return self.sawtooth(np.pi/2 - x)

    def _callback_meteo(self, msg):
        """
        float64 barometric_pressure_mercury
        float64 barometric_pressure_bars
        float64 temperature
        #angle of the wind en degre par rapport au nord dans le sens horaire
        float64 true_wind_direction
        float64 magnetic_wind_direction0.
        float64 wind_speed
        """
        self.TWD_list.append(self.north2east(msg.true_wind_direction))
        self.windSpeed_list.append(msg.wind_speed)
        if self.initDeque >= self.meanDuration:
            self.TWD_list.popleft()
            self.windSpeed_list.popleft()
        else :
            self.initDeque += 1

    def main(self):
        if self.initDeque >= 1:
            twd = mean(self.TWD_list)
            spd = mean(self.windSpeed_list)

            msg = Wind()
            msg.wind_direction = twd;
            msg.wind_speed = spd;
            self.wind_pub.publish(msg);



if __name__ == "__main__":
    rospy.init_node('controller', anonymous=True)
    WindEstimator = WindEstimator()

    #lat0 = rospy.get_param("/origin/lat")
    #lon0 = rospy.get_param("/origin/lon")

    while not rospy.is_shutdown():
        WindEstimator.main()
        WindEstimator.rate.sleep()
