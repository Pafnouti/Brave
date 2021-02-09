#!/usr/bin/env python3

import rospy
import numpy as np
import json
import yaml
from std_msgs.msg import String
from poly_generator.msg import UniquePolygon
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon


EARTH_RADIUS = 6371000.

# Origine repère Ty Colo
lat0 = 48.431775
lon0 = -4.615529


class PolyGenerator():

    def __init__(self, rosrate=10):

        rospy.Subscriber('/AIVDM', String, self._callback_aivdm)
        self.pub_poly = rospy.Publisher('/poly', Polygon, queue_size=10) 

        self.rate = rospy.Rate(rosrate) # fréquence à laquelle le main se répète : rosrate = 10Hz (boucle toutes les 100 ms)

        self.nb_sec = 0
        self.latitude = lat0
        self.longitude = lon0
        self.vitesse_nd = 0
        self.heading = 0
        self.imo = 0


    def _callback_aivdm(self, msg):
        """
        Lorsqu'une trame aivdm est reçue sur le topic '/AIVDM', la fonction 
        _callback_aivdm les récupère afin de calculer le polygone associé.
        """

        # On convertit le msg ROS (String) en dictionary pour récupérer les différentes informations utiles au calcul du polygone
        data = yaml.load(str(msg))
        aivdm_dictionary = json.loads(data["data"])
        print(aivdm_dictionary)

        self.nb_sec = aivdm_dictionary["second"]
        self.latitude = aivdm_dictionary["lat"]
        self.longitude = aivdm_dictionary["lon"]
        self.vitesse_nd = aivdm_dictionary["speed"]
        self.heading = aivdm_dictionary["heading"]
        self.imo = aivdm_dictionary["imo"]
    
    
    def WGS84_to_cart(self, lat, lon):
        """
        Input: gps coord decimal lat, lon
        Return: cartesian coord x, y with (lat0, lon0) the origin
        """
        x = (np.pi/180.)*EARTH_RADIUS*(lon-lon0)*np.cos((np.pi/180.)*lat)
        y = (np.pi/180.)*EARTH_RADIUS*(lat-lat0)
        return x, y


    def Deg2Rad(self, xDeg):
        #Fonction convertissant des Degrés en Radians
        xRad= (xDeg*np.pi)/180
        return xRad

    
    def poly_gen(self, nb_sec, lat, longi, v_nd, heading):

        x_nav, y_nav = self.WGS84_to_cart(longi, lat)
        v = v_nd*4.98/2.69/3.6 # vitesse du navire en m/s

        alpha = 10 # demi-angle d'ouverture du cone (en degrés)
        theta1 = self.Deg2Rad(heading - alpha)
        theta2 = self.Deg2Rad(heading + alpha)

        deltaT = 1
        eps = 0.5

        d1 = d4 = v * deltaT
        d2 = d3 = v * (deltaT + eps)

        k = (heading-alpha)//90

        pt1 = Point32()
        pt2 = Point32()
        pt3 = Point32()
        pt4 = Point32()

        pt1.z = pt2.z = pt3.z = pt4.z = 0

        # Coordonnées du 1er point du polygone
        pt1.x = d1 * np.cos(theta1 - k*np.pi/2)
        pt1.y = d1 * np.sin(theta1 - k*np.pi/2)

        # Coordonnées du 2nd point du polygone
        pt2.x = d2 * np.cos(theta1 - k*np.pi/2)
        pt2.y = d2 * np.sin(theta1 - k*np.pi/2)

        # Coordonnées du 3ème point du polygone
        pt3.x = d3 * np.cos(theta2 - k*np.pi/2)
        pt3.y = d3 * np.sin(theta2 - k*np.pi/2)

        # Coordonnées du 4ème point du polygone
        pt4.x = d4 * np.cos(theta2 - k*np.pi/2)
        pt4.y = d4 * np.sin(theta2 - k*np.pi/2)

        poly = Polygon()
        poly.points = [pt1, pt2, pt3, pt4]

        return poly


    def main(self):

        nb_sec = self.nb_sec
        lat = self.latitude
        longi = self.longitude
        v_nd = self.vitesse_nd
        heading = self.heading
        imo = self.imo

        poly = self.poly_gen(nb_sec, lat, longi, v_nd, heading)

        unique_poly = UniquePolygon()
        unique_poly.id = imo
        unique_poly.poly = poly

        self.pub_poly.publish(unique_poly)


if __name__ == "__main__":
    rospy.init_node('poly_generator', anonymous=True)
    poly_generator = PolyGenerator()
    while not rospy.is_shutdown():
        poly_generator.main()
        poly_generator.rate.sleep()
