#!/usr/bin/env python

import numpy as np
from scipy import interpolate
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from scipy.spatial.distance import cdist
import alphashape
import pickle as pkl
import matplotlib.pyplot as plt
import pandas as pd
import os

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)


def distanceAB(A, B):
    x1, y1 = A
    x2, y2 = B
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def polToCart(angle, d):
    angle = np.array(angle)
    d = np.array(d)
    return d*np.cos(angle), d*np.sin(angle)

def cartToPol(x, y):
    x = np.array(x)
    y = np.array(y)
    return np.arctan2(y, x), np.sqrt(x**2 + y**2)



class Routeur():
    def __init__(self, nb_iso=10):
        self.data_polaire = [[0*np.pi/180, 10*np.pi/180, 30*np.pi/180, 40*np.pi/180, 90*np.pi/180, 110*np.pi/180, 150*np.pi/180, 180*np.pi/180], [0, 0, 0.4, 0.8, 1, 0.9, 0.88, 0.8]]
        self.inter_polaire = interpolate.interp1d(self.data_polaire[0], self.data_polaire[1], kind='cubic')
        self.vitesse_max = 5
        self.vent_ideal = 10

        self.nb_iso = nb_iso

        polar = pd.read_csv("boat_Express.csv", sep=";")
        tws = polar.columns[1:].astype('float')
        twa = polar.values[:,0]
        self.polar_f = interpolate.interp2d(tws, twa, polar.values[:, 1:], kind='linear')


    def polaire(self, alpha, vent=(10, 0)):

        a = alpha - vent[1]
        if a > np.pi: a = 2*np.pi - a
        if a < 0: a = - a
        a = 180*a/np.pi
        return self.polar_f(vent[0], a)
        

    def polaire_plot(self):
        ax = plt.subplot(111, polar=True)
        ax.set_theta_offset(np.pi/2)
        vent = [10]
        for v in vent:
            A = np.linspace(0, 2*np.pi, 100)
            V = []
            for a in A:
                V.append(self.polaire(a, (10, 0)))
        ax.plot(A, V)
        plt.show()

    def weather(self, t=0, pos=(0, 0)):
        return (10, 0)

    def compute_iso(self, points, zone, A, B, d_min=4, line=True, pas=1):
        
        #alpha = alphashape.optimizealpha(points)
        #print(alpha)
        alpha = 0.6 - 0.05*pas
        if pas < 1:
            alpha = 0.99
        elif pas > 100:
            alpha = 0.01
        elif pas < 10:
            alpha = 0.9-0.09*pas
        else: 
            alpha = 0.086 - 7.7e-4*pas
        if pas < 0:
            pas = 0.001
        #print(pas, alpha)
        
        a_s = np.array(list(alphashape.alphashape(points, alpha).exterior.coords))
        bnds = []


        #plt.scatter(points[:, 0], points[:, 1])
        #plt.plot(a_s[:, 0], a_s[:, 1])
        #plt.show()

        i = 0
        bnd = []

        k = 0
        while zone.contains(Point(a_s[k, 0], a_s[k, 1])) and k < a_s.shape[0] - 1:
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

    def run(self, A, B, def_ang=60, pas_s=0.5, nb_iso=10):
        print('Starting...')


        A = [A[0], A[1]]
        B = [B[0], B[1]]

        dAB = distanceAB(A, B)
        pas_s = dAB/(nb_iso*self.polaire(45*np.pi/180, self.weather(pos=A)))

        angle_AB = np.arctan2((B[1] - A[1]), (B[0] - A[0]))

        points = [np.zeros((1, 2))]
        points[-1][0, 0] = A[0]
        points[-1][0, 1] = A[1]


        angles = np.linspace(0, 2*np.pi, def_ang)
        all_points = [A]
        hist_points = {0:-1}


        C = (A[0]+(A[0]+B[0])/2, A[1]+(A[1]+B[1])/2)
        cercle = []
        for a in angles:
            p = C[0]+distanceAB(A, B)/2*np.cos(a), C[1]+distanceAB(A, B)/2*np.sin(a)
            cercle.append(p)
        cercle_np = np.array(cercle)
        cercle = Polygon(cercle_np)

        i_iso = 0
        target_in = False


        while i_iso < 3 and not target_in:

            iso = []

            for j in range(points[-1].shape[0]):
                pt = points[-1][j, :].tolist()
                i_pt = all_points.index(pt)

                for a in angles:
                    x, y = polToCart(a, self.polaire(a, self.weather(pos = pt))*pas_s)
                    x += pt[0]
                    y += pt[1]

                    iso.append((x, y))
                    all_points.append([x, y])
                    hist_points[len(all_points)-1] = i_pt

            tmp_points = np.zeros((len(iso), 2))
            for i in range(len(iso)):
                tmp_points[i][0] = iso[i][0]
                tmp_points[i][1] = iso[i][1]

            print('computing iso : {}'.format(i_iso))
            
            if i_iso > 0:
                pts = self.compute_iso(tmp_points, cercle, A, B, pas=pas_s)
            else:
                pts = tmp_points

            points.append(pts)
            i_iso += 1
            for i in range(pts.shape[0]):
                if distanceAB(B, pts[i, :]) < 10*pas_s:
                    target_in = True
                    print("Target in !")

   

        pts_to_check = np.vstack((points[-1], points[-2]))
        pt = pts_to_check[cdist([np.array(((B[0], B[1])))], pts_to_check).argmin()].tolist()

        i_pt = all_points.index(pt)
        traj_x, traj_y = [all_points[i_pt][0]], [all_points[i_pt][1]]
        while hist_points[i_pt] != -1:
            i_pt = hist_points[i_pt]
            traj_x.append(all_points[i_pt][0])
            traj_y.append(all_points[i_pt][1])

        return (traj_x, traj_y), points, all_points
    



if __name__ == "__main__":
    
    routeur = Routeur()

    #routeur.polaire_plot()
    A, B = (0, 0), (354, 0)

    C = (A[0]+(A[0]+B[0])/2, A[1]+(A[1]+B[1])/2)
    cercle = []
    for a in np.linspace(0, 2*np.pi, 50):
        p = C[0]+distanceAB(A, B)/2*np.cos(a), C[1]+distanceAB(A, B)/2*np.sin(a)
        cercle.append(p)
    cercle_np = np.array(cercle)

    traj, iso, all_points = routeur.run(A, B)
    for i in iso:
        plt.plot(i[:, 0], i[:, 1])
    plt.plot(traj[0], traj[1])
    plt.plot(cercle_np[:, 0], cercle_np[:, 1])
    plt.show()


