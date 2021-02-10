#!/usr/bin/env python

import numpy as np
from scipy import interpolate
from shapely.geometry import Point, MultiPoint, MultiPolygon, LineString
from shapely.geometry.polygon import Polygon
from scipy.spatial.distance import cdist
import alphashape
import pickle as pkl
import matplotlib.pyplot as plt
import pandas as pd
import os
import time
import rospy

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)

DEBUG = True
deep_DEBUG = False


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

def cout(msg, *args):
    s = '{} '.format(msg)
    for m in args:
        s += '{} '.format(m)
    if DEBUG:
        print(s)
    else:
        rospy.loginfo(s)

def getAngleABC(A, B, C):
    ba = np.arctan2(A[1]-B[1], A[0] - B[0])
    bc = np.arctan2(C[1]-B[1], C[0]-B[0])
    return ba - bc



class Routeur():
    def __init__(self):
        self.data_polaire = [[0*np.pi/180, 10*np.pi/180, 30*np.pi/180, 40*np.pi/180, 90*np.pi/180, 110*np.pi/180, 150*np.pi/180, 180*np.pi/180], [0, 0, 0.4, 0.8, 1, 0.9, 0.88, 0.8]]
        self.inter_polaire = interpolate.interp1d(self.data_polaire[0], self.data_polaire[1], kind='cubic')
        self.vitesse_max = 5
        self.vent_ideal = 10
    
        self.wind = (10, 0)


        polar = pd.read_csv("boat_Express.csv", sep=";")
        tws = polar.columns[1:].astype('float')
        twa = polar.values[:,0]
        self.polar_f = interpolate.interp2d(tws, twa, polar.values[:, 1:], kind='linear')
        pas = np.array([0, 0.2, 0.664, 0.996, 2, 2.65, 3, 4, 5.3, 6, 7.3, 8.63, 10, 12.62, 15.28, 17.266, 20.92, 23.244, 27.89, 40, 80, 100])
        alpha = np.array([1, 1, 0.99, 0.99, 0.9, 0.7, 0.6, 0.45, 0.35, 0.3, 0.25, 0.21, 0.17, 0.15, 0.12, 0.105, 0.09, 0.08, 0.068, 0.045, 0.02, 0.015])*0.9
        self.set_alpha = interpolate.interp1d(pas, alpha, kind='cubic')

        if DEBUG:
            self.poly_cargos = []


    def polaire(self, alpha, vent=(10, 0)):

        a = alpha - vent[1]
        if a > np.pi: a = 2*np.pi - a
        if a < 0: a = - a
        a = 180*a/np.pi
        return self.polar_f(vent[0], a)
        
    def show_alpha(self):
        x, y = [], []
        for pas in np.linspace(0, 100, 400):
            x.append(pas)
            y.append(self.set_alpha(pas))
        plt.plot(x, y)
        plt.show()

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

    def set_weahter(self, vitesse, angle):
        self.wind = (vitesse, angle)

    def weather(self, t=0, pos=(0, 0)):
        return self.wind

    def zones_cargos(self, cargos, t, dt, coeff_s = 3, alpha = np.pi/4):
        zones = []
        for c in cargos:
            v = cargos[c]['v']
            cap = cargos[c]['cap']
            x, y = cargos[c]['pos']
            x, y = (x + v*t*np.cos(cap), y + v*t*np.sin(cap))
            d = coeff_s * v * dt

            A, B = Point(x + d*np.cos(cap+alpha/2), y + d*np.sin(cap+alpha/2)), Point(x + d*np.cos(cap-alpha/2), y + d*np.sin(cap-alpha/2))
            zones.append(Polygon([Point(x, y), A, B]))
            if DEBUG:
                self.poly_cargos.append(([x, A.x, B.x, x], [y, A.y, B.y, y]))
            if deep_DEBUG : plt.plot([x, A.x, B.x, x], [y, A.y, B.y, y])
        return MultiPolygon(zones)

    def compute_iso(self, points, zone, zones_cargos, A, B, d_min=4, line=True, pas=1):
                
        #alpha = alphashape.optimizealpha(points)
        
        # réglage d'alpha avec une fonction empirique
        alpha = self.set_alpha(pas)*0.9

        #cout('Alpha/pas ', alpha, pas)
        #a_s = np.array(list(alphashape.alphashape(points, alpha).exterior.coords))
        
        # Calcul de l'alphashape
        ok = False
        while not ok:
            try:
                a_s = np.array(list(alphashape.alphashape(points, alpha).exterior.coords))
                ok = True
            except AttributeError:
                cout('error : Alpha:{}/pas:{}, trying with {}'.format(alpha, pas, 0.8*alpha))
                alpha = 0.9*alpha
        bnds = []
        bnds_m = []
       
        #plt.scatter(points[:, 0], points[:, 1])

        # idée d'optimisation -> problème : on obtient les points à l'intérieur de la zone, mais comment séléctionner la bonne ligne ?
        # Comment avoir la notion de ligne ?
        # pts_coll = MultiPoint(a_s)
        # inter = zone.intersection(pts_coll)
        
        # on se place au début d'une ligne
        i = 0
        bnd = []
        k = 0
        while zone.contains(Point(a_s[k, 0], a_s[k, 1])) and k < a_s.shape[0] - 1:
            k += 1
        
        # on regroupe par ligne
        for j in range(a_s.shape[0]):
            i = (j + k)%a_s.shape[0]
            P = Point(a_s[i, 0], a_s[i, 1])
            if zone.contains(P):
                i_pt = self.all_points.index([a_s[i, 0], a_s[i, 1]])
                i_pt = self.hist_points[i_pt]
                ligne = LineString([(a_s[i, 0], a_s[i, 1]), self.all_points[i_pt]])
                if not ligne.crosses(zones_cargos):
                    bnd.append((a_s[i, 0], a_s[i, 1]))
            else:
                if len(bnd) >= 2:
                    bnds.append(bnd)
                bnd = []

        if len(bnd) >= 2:
            bnds.append(bnd)

        # on séléctionne celle dont la moyenne des distances à A est la plus grande
        for bnd in bnds:
            bnd = np.array(bnd)
            XB = np.array(((0, 0))).reshape(1,-1)
            bnds_m.append(np.mean(cdist(bnd, XB, 'euclidean')))
        bnd = np.array(bnds[bnds_m.index(max(bnds_m))])

        if deep_DEBUG:
            plt.plot(bnd[:, 0], bnd[:, 1])
            plt.show()

        return bnd


    def run(self, A, B, def_ang=50, pas_t=0.5, nb_iso=10, cargos={'0':{'pos':(-100, -200), 'v':5, 'cap':0}}):
        cout('Starting...')

        #pour debug : 
        self.cargos = cargos

        # départ et arrivé
        A = [A[0], A[1]]
        B = [B[0], B[1]]

        # temps
        t = 0

        # réglage du pas un peu empirique
        dAB = distanceAB(A, B)
        v_moy = self.polaire(45*np.pi/180, self.weather(pos=A))*0.7
        pas_t = dAB/(nb_iso*v_moy)

        cout('Distance à parcourir : {}, vitesse estimée : {}, eta : {}'.format(dAB, v_moy, dAB/v_moy))

        # la variables points stocke les points des isochrones 
        points = [np.zeros((1, 2))]
        points[-1][0, 0] = A[0]
        points[-1][0, 1] = A[1]

        # angles donne une discrétisation de def_ang angles sur 2pi ou 3/4 de 2pi
        angles = np.linspace(0, 2*np.pi, def_ang)
        angles_2 = np.linspace(0, 3/2*np.pi, int(3/4*def_ang))

        # self.all_points permet de retrouver le chemin
        self.all_points = [A]
        self.hist_points = {0:-1}

        # c'est le cercle dans lequelle on peut faire les calculs, pour optimiser et que le voilier ne cherche pas à aller dans la direction opposée du point d'arrivée
        C = (A[0]+(A[0]+B[0])/2, A[1]+(A[1]+B[1])/2)
        cercle = []
        r = distanceAB(A, B)/2*1.2
        for a in angles:
            p = C[0]+r*np.cos(a), C[1]+r*np.sin(a)
            cercle.append(p)
        cercle_np = np.array(cercle)
        cercle = Polygon(cercle_np)

        # le deuxième cercle sert pour généré les points, le premier, plus petit pour selectionner les lignes (cf alphashape)
        cercle_2 = []
        r = r*1.2
        for a in angles:
            p = C[0]+r*np.cos(a), C[1]+r*np.sin(a)
            cercle_2.append(p)
        cercle2_np = np.array(cercle_2)
        cercle2 = Polygon(cercle2_np)



        i_iso = 0
        target_in = False
        target_near = False
        pas2 = pas_t #stockage du pas pour le réduire quand on s'approche de l'arrivée


        # boucle principale
        while i_iso < 2*nb_iso and not target_in:

            if target_near:
                pas_t = pas2/3
            iso = []

            # on update la pos des cargos :
            zones_cargos = self.zones_cargos(cargos, t, pas_t) 

            # pour tous les points de l'isochrone précédente 
            for j in range(points[-1].shape[0]):
                pt = points[-1][j, :].tolist()
                i_pt = self.all_points.index(pt)

                # pour tous les angles 
                for a in angles:
                    x, y = polToCart(a, self.polaire(a, self.weather(pos = pt))*pas_t)
                    x += pt[0]
                    y += pt[1]

                                             
                    # ou pour 3/4 de cercle plutot ;)
                    if (abs(getAngleABC(A, pt, (x, y))) > np.pi/4) or i_iso < 1:
                            iso.append((x, y))
                            self.all_points.append([x, y])
                            self.hist_points[len(self.all_points)-1] = i_pt
            
            # passage en numpy
            tmp_points = np.zeros((len(iso), 2))
            for i in range(len(iso)):
                tmp_points[i][0] = iso[i][0]
                tmp_points[i][1] = iso[i][1]
            
            # pour toutes les iso sauf la première
            if i_iso > 0:
                cout('computing iso : {}'.format(i_iso))
                tmp_points = np.array(cercle2.intersection(MultiPoint(tmp_points))) # on enlève les points zen dehors du plus grand cerlce
                if deep_DEBUG : plt.plot(cercle_np[:, 0], cercle_np[:, 1])
                pts = self.compute_iso(tmp_points, cercle, zones_cargos, A, B, pas=pas_t)
            else:
                pts = tmp_points

            points.append(pts)
            i_iso += 1

            # conditions d'arrêt
            for i in range(pts.shape[0]):
                if distanceAB(B, pts[i, :]) < 0.2*dAB/nb_iso:
                    target_in = True
                elif distanceAB(B, pts[i, :]) < 1.2*dAB/nb_iso:
                    target_near = True
            if target_in:
                cout("Target in !")
            elif target_near:
                cout('Target near !')
                
            t += pas_t
   

        pts_to_check = np.vstack((points[-1], points[-2]))
        pt = pts_to_check[cdist([np.array(((B[0], B[1])))], pts_to_check).argmin()].tolist()

        # on retrace la trajectoire
        i_pt = self.all_points.index(pt)
        traj_x, traj_y = [self.all_points[i_pt][0]], [self.all_points[i_pt][1]]
        while self.hist_points[i_pt] != -1:
            i_pt = self.hist_points[i_pt]
            traj_x.insert(0, self.all_points[i_pt][0])
            traj_y.insert(0, self.all_points[i_pt][1])
        traj_x.append(B[0])
        traj_y.append(B[1])

        return (traj_x, traj_y), points
    



if __name__ == "__main__":

    
    routeur = Routeur()
    #routeur.show_alpha()
    #routeur.polaire_plot()
    A, B = (0, 0), (420, -325)
    plt.scatter((A[0], B[0]), (A[1], B[1]))
    C = (A[0]+(A[0]+B[0])/2, A[1]+(A[1]+B[1])/2)
    cercle = []
    r = distanceAB(A, B)/2*1.2
    angles = np.linspace(0, 2*np.pi, 60)
    for a in angles:
        p = C[0]+r*np.cos(a), C[1]+r*np.sin(a)
        cercle.append(p)
    cercle_np = np.array(cercle)

    traj, iso = routeur.run(A, B)
    for i in iso:
        plt.plot(i[:, 0], i[:, 1])
    for p_c in routeur.poly_cargos[:]:
        plt.plot(p_c[0], p_c[1])
    
    plt.plot(traj[0], traj[1])
    plt.plot(cercle_np[:, 0], cercle_np[:, 1])
    plt.show()


