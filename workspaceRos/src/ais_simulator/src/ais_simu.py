#!/usr/bin/env python3

import rospy
import numpy as np
import json
from traj_generator.msg import TrajInfo
from std_msgs.msg import String

# Origine repère Ty Colo
lat0 = 48.431775
lon0 = -4.615529

class AISSimulator():

    def __init__(self, rosrate=10):
                
        rospy.Subscriber('/trajInfo', TrajInfo, self._callback_traj_info)
        self.pub_aivdm = rospy.Publisher('/AIVDM', String, queue_size=10)

        self.rate = rospy.Rate(rosrate) # fréquence à laquelle le main se répète : rosrate = 10Hz (boucle toutes les 100 ms)

        self.nb_sec = 0
        self.latitude = lat0
        self.longitude = lon0
        self.vitesse_nd = 0
        self.heading = 0


    def _callback_traj_info(self, msg):
        """
        Lorsque les informations de trajectoire du navire à éviter sont envoyées sur le topic '/trajInfo', 
        la fonction _callback_traj_info les récupère pour générer la trame AIVDM associée.
        """

        self.nb_sec = msg.nb_sec
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.vitesse_nd = msg.vitesse_nd
        self.heading = msg.heading


    def AIVDM_gen(self, nb_sec, lat, longi, v_nd, heading):
        """
        Fonction simulant un AIS et retournant un dictionnaire contenant les informations 
        des trames AIVDM, en supposant qu'elles aient déjà été prétraitées par libais.
        """

        aivdm = { 
            "class": "AIS",
            "device": "stdin",
            "scaled": True,
            "status": 0,
            "status_text": "Under way using engine",
            "imo": 9290610,
            "shipname": "Cargo to avoid",
            "shiptype": 70,
            "to_bow": 0.45,
            "to_stern": 0.45,
            "to_port": 5,
            "to_starboard": 5,
            "heading": 0,
            "type": 1,
            "repeat": 0,
            "mmsi": 371798000,
            "turn": 0,
            "speed": 0., 
            "accuracy": True,
            "lon": 0.,
            "lat": 0., 
            "course": 0., 
            "second": 0, 
            "maneuver": 0,
            "spare": 0,
            "raim": False,
            "sync_state": 0
        }

        # HEURE (en secondes)
        aivdm["second"] = nb_sec
        # DIRECTION DE LA VITESSE PAR RAPPORT AU FOND (en degrés, course = heading pour simu)
        aivdm["course"] = heading
        # CAP (en degrés, de 0 à 359)
        aivdm["heading"] = heading
        # LATITUDE (en dégrés, North = positive, South = negative)
        aivdm["lat"] = lat
        # LONGITUDE (en degrés, East = positive, West = negative) 
        aivdm["lon"] = longi
        # VITESSE PAR RAPPORT AU FOND (en noeuds)
        aivdm["speed"] = v_nd

        return aivdm


    def main(self):

        nb_sec = self.nb_sec
        lat = self.latitude
        longi = self.longitude
        v_nd = self.vitesse_nd
        heading = self.heading
        aivdm_dictionary = self.AIVDM_gen(nb_sec, lat, longi, v_nd, heading)
        print(aivdm_dictionary)

        # On publie la trame AIVDM sous forme de String sur le topic '/AIVDM'
        aivdm_msg = json.dumps(aivdm_dictionary) # converts dictionary into str
        self.pub_aivdm.publish(aivdm_msg)


if __name__ == "__main__":
    rospy.init_node('traj_generator', anonymous=True)
    ais_simulator = AISSimulator()
    while not rospy.is_shutdown():
        ais_simulator.main()
        ais_simulator.rate.sleep()