import numpy as np
import matplotlib.pyplot as plt 
from collision import *
from scipy import interpolate
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from scipy.spatial.distance import cdist
import alphashape
from time import time
import pickle as pkl


class Routeur():
    def __init__(self):
        self.data_polaire = [[0*np.pi/180, 10*np.pi/180, 30*np.pi/180, 40*np.pi/180, 90*np.pi/180, 110*np.pi/180, 150*np.pi/180, 180*np.pi/180], [0, 0, 0.4, 0.8, 1, 0.9, 0.88, 0.8]]
        self.inter_polaire = interpolate.interp1d(self.data_polaire[0], self.data_polaire[1], kind='cubic')
        self.vitesse_max = 5
        self.vent_ideal = 10

        self.windpos, self.wind = pkl.load(open('wind/wind.bin', 'rb'))

    def polaire(self, alpha, vent_=1):
        vent = self.weather()
        a = alpha - vent[1]
        if a > np.pi: a = 2*np.pi - a
        if a < 0: a = - a
        return self.inter_polaire(a)*self.vitesse_max
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
        pass

    def compute_iso(self, points, zone, A, B, d_min=4, line=True, pas=1):

        

     
        a_s = np.array(list(alphashape.alphashape(points, 0.43*pas).exterior.coords))
        bnds = []

        i = 0
        bnd = []

        k = 0
        while zone.contains(Point(a_s[k, 0], a_s[k, 1])):
            k += 1

        for j in range(a_s.shape[0]):
            i = (j + k)%a_s.shape[0]
            if zone.contains(Point(a_s[i, 0], a_s[i, 1])):
                bnd.append((a_s[i, 0], a_s[i, 1]))
            else:
                if len(bnd) >= 2:
                    bnds.append(bnd)
                bnd = []
        if len(bnd) >= 2:
            bnds.append(bnd)

        d_max = 0
        j_max = 0
        for j in range(len(bnds)):
            d = distanceAB(A, bnds[j][0])
            if d > d_max:
                d_max = d
                j_max = j

        bnd = np.array(bnds[j_max])

        return bnd

    def run(self, A, B, angle_calcul=rad(170), nb_traj=150, pas_s=0.5):
        print('Starting...')

        

        angle_AB = np.arctan2((B[1] - A[1]), (B[0] - A[0]))

        points = [np.zeros((1, 2))]
        points[-1][0, 0] = A[0]
        points[-1][0, 1] = A[1]


        angles = np.linspace(0, 2*np.pi, 60)
        all_points = [A]
        hist_points = {0:-1}


        C = (A[0]+(A[0]+B[0])/2, A[1]+(A[1]+B[1])/2)
        cercle = []
        for a in angles:
            p = C[0]+distanceAB(A, B)/2*np.cos(a), C[1]+distanceAB(A, B)/2*np.sin(a)
            cercle.append(p)
        cercle_np = np.array(cercle)
        plt.plot(cercle_np[:, 0], cercle_np[:, 1])
        cercle = Polygon(cercle_np)

        i_iso = 0
        target_in = False


        while i_iso < 100 and not target_in:
            iso = []
            t0 = time()
            for j in range(points[-1].shape[0]):
                pt = points[-1][j, :]
                i_pt = all_points.index((pt[0], pt[1]))
                
                for a in angles:
                    x, y = polToCart(a, self.polaire(a)*pas_s)
                    x += pt[0]
                    y += pt[1]

                    iso.append((x, y))
                    all_points.append((x, y))
                    hist_points[len(all_points)-1] = i_pt

            t1 = time()
            #print('##########\ncalcul des points : {}'.format(t1 - t0))

            tmp_points = np.zeros((len(iso), 2))
            for i in range(len(iso)):
                tmp_points[i][0] = iso[i][0]
                tmp_points[i][1] = iso[i][1]

            print('computing iso : {}'.format(i_iso))

            pts = self.compute_iso(tmp_points, cercle, A, B, pas=pas_s)

            t3 = time()
            #print('calcul iso : {}'.format(t3 - t2))

            points.append(pts)
            i_iso += 1
            for i in range(pts.shape[0]):
                if distanceAB(B, pts[i, :]) < 2:
                    target_in = True
                    print("Target in !")

        plt.scatter([A[0], B[0]], [A[1], B[1]])


        pts_to_check = np.vstack((points[-1], points[-2]))
        pt = pts_to_check[cdist([np.array(((B[0], B[1])))], pts_to_check).argmin()]

        i_pt = all_points.index((pt[0], pt[1]))
        traj_x, traj_y = [all_points[i_pt][0]], [all_points[i_pt][1]]
        while hist_points[i_pt] != -1:
            i_pt = hist_points[i_pt]
            traj_x.append(all_points[i_pt][0])
            traj_y.append(all_points[i_pt][1])

        plt.plot(traj_x, traj_y)

        for pts in points:
            plt.plot(pts[:, 0], pts[:, 1])

        plt.show()
        
            
        





if __name__ == "__main__":
    routeur = Routeur()
    #routeur.polaire_plot()

    routeur.run((0, 0), (30, 2))
