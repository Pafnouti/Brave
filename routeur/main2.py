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
        for iso in isochrones:
            X = [p[0] for p in iso.exterior.coords]
            Y = [p[1] for p in iso.exterior.coords]  
            plt.plot(X, Y)

        X, Y = [], []
        for a in np.linspace(0, 2*np.pi):
            d = self.polaire(a)
            x, y = pointAtDistance(A, 10*d, a)
            X.append(x)
            Y.append(y)

        plt.legend(["iso n°" + str(i) for i in range(len(isochrones))])
    
        plt.show()
        plt.pause(10)

    def run(self, A, B, angle_calcul=180, nb_traj=10, pas=1):

        target = Point(B[0], B[1])
        iso = [(pointAtDistance(A, self.polaire(2*np.pi*i/nb_traj)*5, 2*np.pi*i/nb_traj)) for i in range(nb_traj)]
        trajs = {}
        for i in range(len(iso)):
            trajs[i] = [A, iso[i]] 
        isochrones = [Polygon(iso)]

        

        while(not isochrones[-1].contains(target) and len(isochrones)<30):
            bounds = list(isochrones[-1].exterior.coords)
            iso = []
            pol_angle = self.weather()[1]   
            O_pol = pointAtDistance()
            
            for i in range(len(bounds)):
                b_a, b_c = bounds[-1], bounds[(i+1)%len(bounds)]
                angle_next = np.arctan(b_a[0] - b_c[0], b_a[1] - b_c[1]) - np.pi/2
                dist = self.polaire(angle_next)
                new_point = pointAtDistance(bounds[i], dist*pas, angle_next)
                if getAngle()                      
                iso.append(new_point)

            print('---------')
            #test boucles : 
            ok = False
            while not ok:
                stop = False
                for i in range(len(iso) - 1):
                    if not stop:
                        s1 = (iso[i], iso[i+1])
                        for k in range(i+2, len(iso) - 1):
                            if not stop:
                                s2 = (iso[k], iso[k+1])
                                bool_intersec, I = intersectionSegments(s1, s2)
                                if bool_intersec:
                                    poly1 = iso[0:i] + [I] + iso[k+1:]
                                    poly2 = iso[i+1:k+1] + [I]
                                    #plotPoly([iso], [s1, s2], [I])
                                    if len(poly1) > len(poly2):
                                        iso = poly1
                                    else:
                                        iso = poly2
                                    #plotPoly([iso])
                                    #plotPoly([poly1, poly2])
                                    stop = True
                        if i == len(iso) - 2:
                            ok = True

            isochrones.append(Polygon(iso))
            
        self.show_result(A, B, isochrones)





if __name__ == "__main__":
    routeur = Routeur()
    routeur.polaire_plot()

    routeur.run((0, 0), (20, 20))
