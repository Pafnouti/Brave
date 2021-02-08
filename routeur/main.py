import numpy as np
import matplotlib.pyplot as plt 
from collision import *
from scipy import interpolate
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


class Routeur():
    def __init__(self):
        self.data_polaire = [[0*np.pi/180, 10*np.pi/180, 30*np.pi/180, 40*np.pi/180, 90*np.pi/180, 110*np.pi/180, 150*np.pi/180, 180*np.pi/180], [0, 0, 0.4, 0.8, 1, 0.9, 0.88, 0.8]]
        self.inter_polaire = interpolate.interp1d(self.data_polaire[0], self.data_polaire[1], kind='cubic')
        self.vitesse_max = 5
        self.vent_ideal = 10


    def polaire(self, alpha, vent_=1):
        vent = self.weather()
        a = alpha - vent[1]
        if a > np.pi: a = 2*np.pi - a
        if a < 0: a = - a
        return self.inter_polaire(a)*np.cos(abs(vent[0] - self.vent_ideal)/self.vent_ideal)*self.vitesse_max*np.cos(abs(vent_ - self.vent_ideal)/self.vent_ideal)
        #return 1

    def polaire_plot(self):
        ax = plt.subplot(111, polar=True)
        ax.set_theta_offset(np.pi/2)
        vent = [10]
        for v in vent:
            A = np.linspace(0, 2*np.pi, 100)
            V = []
            for a in A:
                V.append(self.polaire(a, vent_=v))

            ax.plot(A, V)
        plt.show()

    def weather(self, t=0):
        return 10, 0

    def show_result(self, A, B, isochrones):
        ax = plt.subplot(111)
        ax.scatter(A[0], A[1])
        ax.scatter(B[0], B[1])
        

        for traj in isochrones:
            X, Y = [], []
            for pt in traj:
                if pt != None:
                    X.append(pt[0])
                    Y.append(pt[1])
            plt.plot(X, Y)
            plt.scatter(X, Y)
        plt.show()
        plt.pause(10)

    def run(self, A, B, angle_calcul=rad(150), nb_traj=3, pas=1):
        angle_AB = getAngle(A, B)
        isochrones = [[A] for i in range(nb_traj)]
        for i in range(nb_traj):
            isochrones[i].append(pointAtDistance(A, self.polaire(angle_calcul*i/nb_traj - angle_calcul/2 + angle_AB)*5, angle_calcul*i/nb_traj - angle_calcul/2 + angle_AB))

        

        arrived = False
        it = 0
        while not arrived and it < 2:
            it += 1
            for traj in isochrones:
                if traj[-1] != None:
                    angle_next = getAngle(traj[-2], traj[-1])
                    dist = self.polaire(angle_next)
                    new_point = pointAtDistance(traj[i-1], dist*pas, angle_next)
                    traj.append(new_point)
        
        self.show_result(A, B, isochrones)


        
            
        





if __name__ == "__main__":
    routeur = Routeur()
    #routeur.polaire_plot()

    routeur.run((0, 0), (20, 20))
