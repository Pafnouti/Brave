#!/usr/bin/env python3

import rospy
import numpy as np
import time
from geometry_msgs.msg import Pose2D
from traj_generator.msg import TrajInfo

EARTH_RADIUS = 6371000.
SPEED_RATE = 20 # La vitesse réelle du navire est 20 fois plus rapide que la vitesse de la simulation

# Origine repère Guerlédan
# lat0, lon0 = (48.198427, -3.014750)

# VALEURS À MODIFIER SI BESOIN
# Origine repère Ty Colo
lat0 = 48.431775
lon0 = -4.615529
# Point final à atteindre
latf = 48.42
lonf = -4.62

class TrajGenerator():

    def __init__(self, rosrate=10):

        self.pub_trajInfo = rospy.Publisher('/trajInfo', TrajInfo, queue_size=10)
        self.pub_posNavire = rospy.Publisher('/posNavire', Pose2D, queue_size=10)

        self.rate = rospy.Rate(rosrate) # fréquence à laquelle le main se répète : rosrate = 10Hz (boucle toutes les 100 ms)

        self.lon0 = lat0
        self.lat0 = lon0
        self.lonf = latf
        self.latf = lonf

        # Coordonnées du point initial de la trajectoire du navire simulé en longitude, latitude
        self.a = np.array([[self.lon0], [self.lat0]])
        # Coordonnées du point final de la trajectoire du navire simulé en longitude, latitude
        self.b = np.array([[self.lonf], [self.latf]])

        # Coordonnées cartésiennes (x,y) du point final de la trajectoire du navire
        self.x0, self.y0 = self.WGS84_to_cart(self.lon0, self.lat0)
        # Coordonnées cartésiennes (x,y) du point final de la trajectoire du navire
        self.xf, self.yf = self.WGS84_to_cart(self.lonf, self.latf)

        # On initialise les variables du navire au point a (lon0, lat0)
        self.longitude = self.lon0
        self.latitude = self.lat0
        self.x = self.x0
        self.y = self.y0
        self.vitesse_nd = 0
        self.heading = 0
        self.imo = 9403815

        self.t0 = rospy.get_rostime() # on initialise le temps
        self.time_simu = 0
        self.t = rospy.get_rostime() # on initialise le temps
        self.T_res = 3600 # Temps de simulation restant (au début de la simulation, le temps restant est égal au temps total de la simu)
        self.DX_res = abs(self.xf - self.x0) # distance restant à parcourir suivant X
        self.DY_res = abs(self.yf - self.y0) # distance restant à parcourir suivant Y
        self.dT = 0.1
        # Etant donné que rosrate = 10, l'intervalle de temps entre 2 boucles pour suivre le temps réel
        # lors de la simulation devrait valoir 0.1 s. Pour suivre le temps réel d'exécution de l'algorithme, on utilise
        # ensuite rospy.get_rostime()
        self.N_res = int(self.T_res/self.dT) # Nombre d'échantillons de temps restant
        self.dx = self.DX_res/self.N_res
        self.dy = self.DY_res/self.N_res


    def Rad2Deg(self, xRad):
        #Fonction convertissant des Radians en Degrés
        xDeg = (xRad*180)/np.pi
        return xDeg


    def Deg2Rad(self, xDeg):
        #Fonction convertissant des Degrés en Radians
        xRad= (xDeg*np.pi)/180
        return xRad


    def WGS84_to_cart(self, lat, lon):
        """
        Input: gps coord decimal lat, lon
        Return: cartesian coord x, y with (lat0, lon0) the origin
        """
        x = (np.pi/180.)*EARTH_RADIUS*(lon-lon0)*np.cos((np.pi/180.)*lat)
        y = (np.pi/180.)*EARTH_RADIUS*(lat-lat0)
        return x, y


    def cart_to_WGS84(self, x, y):
        """
        Input: cartesian coord x, y with (lat0, lon0) the origin
        Return: gps coord decimal lat, lon
        """
        EPSILON = 0.00000000001
        lat = y*180./np.pi/EARTH_RADIUS+lat0
        if abs(lat-90.) < EPSILON or abs(lat+90.) < EPSILON:
            lon = 0
        else:
            lon = (x/EARTH_RADIUS)*(180./np.pi)/np.cos((np.pi/180.)*(lat))+lon0
        return lat, lon



    def calcul_distance(self, long_i, lat_i, long_f, lat_f):
        """
        Fonction calculant et retournant la distance en mètres entre 2 points de coordonnées GPS (long_i, lat_i) et (long_f, lat_f)
        """

        long_i = self.Deg2Rad(long_i)
        lat_i = self.Deg2Rad(lat_i)
        long_f = self.Deg2Rad(long_f)
        lat_f = self.Deg2Rad(lat_f)

        # Distance entre les 2 localisations
        d = EARTH_RADIUS*np.arccos(np.sin(lat_i)*np.sin(lat_f) + np.cos(lat_i)*np.cos(lat_f)*np.cos(long_i-long_f))

        return d


    def move_navire(self, dx, dy):
        """
        Fonction simulant le déplacement d'un navire suivant une trajectoire y = ax+b,
        et retournant à chaque instant les infos permettant de construire la trame AIVDM.
        Un appel à la fonction move_navire correspond à un déplacement (dx, dy) pendant dT,
        (que l'on a multiplié par le SPEED_RATE pour accélérer la simulation par rapport au temps réel).
        """

        dx = dx * SPEED_RATE
        dy = dy * SPEED_RATE

        x_next = self.x - dx
        y_next = self.y - dy
        print(dx, dy)

        # Longitude et latitude du bateau
        lon_next, lat_next = self.cart_to_WGS84(x_next, y_next)

        # Route sur le fond en degrés (le cap)
        # C'est l'angle exprimé en degrés (de 0 à 360°), dans le sens des aiguilles d'une montre, 
        # entre la direction du Nord et celle du navire.
        #heading = abs(self.Rad2Deg(np.arctan2(-dx, -dy) - np.arctan2(0, 1)))
        #heading = abs(self.Rad2Deg(np.arctan2(self.xf, self.yf) - np.arctan2(0, 1)))
        heading = self.Rad2Deg(np.arctan2(self.xf, self.yf) - np.arctan2(0, 1))
        if heading < 0 :
            heading = 360 - abs(heading)

        # Vitesse sur le fond en nœuds (2,69 nd = 4,98 km/h)
        long_i, lat_i = self.cart_to_WGS84(self.x, self.y)
        long_f, lat_f = self.cart_to_WGS84(x_next, y_next)
        d = self.calcul_distance(long_i, lat_i, long_f, lat_f)  # distance réelle parcourue par le navire pendant dT (en mètres)
        print("Distance parcourue pendant dT")
        print(d)

        v = (d/self.dT) # vitesse réelle du navire en m/s
        v_kmh = v*3.6 # conversion en km/h
        v_nd = v_kmh*2.69/4.98 # conversion en noeuds

        # On initialise les valeurs pour le prochain tour de boucle
        self.x = x_next
        self.y = y_next
        latitude, longitude = lat_next, lon_next

        return latitude, longitude, v_nd, heading


    def main(self):

        self.latitude, self.longitude, self.vitesse_nd, self.heading = self.move_navire(self.dx, self.dy)

        # Temps en secondes
        t_next = rospy.get_rostime() # on récupère le temps actuel
        delta_time = t_next - self.t
        self.dT = (delta_time).to_sec() #floating point
        self.time_simu = (t_next - self.t0).to_sec()

        # Temps restant
        self.T_res -= self.dT
        self.N_res = int(self.T_res/self.dT)

        # Distance restant à parcourir
        self.DX_res -= self.dx
        self.DY_res -= self.dy

        # Distance à parcourir au prochain tour de boucle
        self.dx = self.DX_res/self.N_res
        self.dy = self.DY_res/self.N_res

        trajectory_info = TrajInfo()

        trajectory_info.nb_sec = int(self.time_simu)
        trajectory_info.latitude = self.latitude
        trajectory_info.longitude = self.longitude
        trajectory_info.vitesse_nd = self.vitesse_nd
        trajectory_info.heading = self.heading
        trajectory_info.imo = self.imo

        print("Temps réel écoulé depuis le début de la simulation (secondes)")
        print(int(self.time_simu))
        print("Latitude (degrés)")
        print(self.latitude)
        print("Longitude (degrés)")
        print(self.longitude)
        print("Vitesse (noeuds)")
        print(self.vitesse_nd)
        print("Cap (degrés)")
        print(self.heading)
        print("ID")
        print(self.imo)

        pos_nav = Pose2D()
        pos_nav.x = self.x
        pos_nav.y = self.y
        pos_nav.theta = self.heading

        self.pub_trajInfo.publish(trajectory_info)
        self.pub_posNavire.publish(pos_nav)

        self.t = t_next



if __name__ == "__main__":
    rospy.init_node('traj_generator', anonymous=True)
    traj_generator = TrajGenerator()
    while not rospy.is_shutdown():
        traj_generator.main()
        traj_generator.rate.sleep()