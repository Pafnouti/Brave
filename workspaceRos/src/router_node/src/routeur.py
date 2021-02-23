#!/usr/bin/env python

import numpy as np
from scipy import interpolate
from shapely.geometry import Point, MultiPoint, MultiPolygon, LineString
from shapely.geometry.polygon import Polygon
from scipy.spatial.distance import cdist
import pickle as pkl
import matplotlib.pyplot as plt
import pandas as pd
import os
import time
import rospy

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)

DEBUG = False
deep_DEBUG = False
d_min_ENABLED = True


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
        
        t0 = time.time()
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

        t1 = time.time() - t0
       
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

        t2 = time.time() - t0

        print("alphashape : ", (t1/t2))


        if deep_DEBUG:
            plt.plot(bnd[:, 0], bnd[:, 1])
            plt.show()

        return bnd

    def sort_dist(self, points, A, B, coeff_min=0.8):
        #distances = np.dot(points[:, :2]-A, (B-A).T).flatten()
        #distances = cdist(A, points[:, :2]).flatten() - cdist(B, points[:, :2]).flatten()
        #distances = cdist(B, points[:, :2]).flatten()
        distances = cdist(A, points[:, :2]).flatten()
        #distances = cdist(A, points[:, :2]).flatten() - 0.5*cdist(B, points[:, :2]).flatten() + 0*np.dot(points[:, :2]-A, (B-A).T).flatten()

        i_d = np.argsort(distances)

        d_min = distances.mean()*coeff_min
        

        if d_min_ENABLED:
            i = 0
            while i < len(distances) and distances[i_d[i]] < d_min:
                i += 1
                # print(d_min, distances[i_d[i]] )
            return points[i_d[i:], :]

        

        return points[i_d, :]
    
    def in_poly(self, points, poly):
        pts = np.zeros((1, 3))
        for i in range(points.shape[0]):
            if poly.contains(Point(points[i, 0], points[i, 1])):
                if pts.shape[0] == 1:
                    pts = points[i, :]
                else:
                    pts.vstack((pts, points[i, :]))
        return pts

    def compute_iso2(self, points, lst_points, A, B, no_go_zones, safe_zones, nb_secteurs=200):


        A = np.array((A[0], A[1])).reshape(1, -1)
        B = np.array((B[0], B[1])).reshape(1, -1)
           
        points = self.sort_dist(points, A, B, coeff_min=1)

        # on determine tous les angles entre les pts et A
        angles = np.arctan2(points[:, 1] - A[0, 1], points[:, 0] - A[0, 0]).reshape(-1, 1)%(2*np.pi)
        #angles = np.arctan2(points[:, 1] - B[0, 1], points[:, 0] - B[0, 0]).reshape(-1, 1)%(2*np.pi)
        points = np.hstack((points, angles))
        

        points2 = np.zeros((1, 3))

        angleAB = np.arctan2(B[0, 1] - A[0, 1], B[0, 0] - A[0, 0])
        angles2 = (angles+np.pi-angleAB)%(2*np.pi)
        angle_min, angle_m, angle_max = min(angles2), np.mean(angles), max(angles2)
        # print(angle_min, angle_m, angle_max)
        angle_min, angle_max = (angle_min-np.pi+angleAB), (angle_max-np.pi+angleAB)
        # if not angle_min < angle_m < angle_max: 
        #     angle_max = angle_max - 2*np.pi
        # print(angle_min, angle_m, angle_max)

        secteurs = np.linspace(angle_min, angle_max, nb_secteurs)
        secteurs = secteurs%(2*np.pi)


        for i in range(nb_secteurs - 1):

            pts = points[secteurs[i] < points[:, 3], :]
            pts = pts[pts[:, 3] < secteurs[i+1]]

            # plt.scatter(points[:, 0], points[:, 1])
            # plt.scatter(pts[:, 0], pts[:, 1])
            # plt.scatter(A[0], A[1])
        

            #pts = self.sort_dist(pts[:, :-1], A, d_min=d_mean*0.5)


            if len(pts) != 0:
                
                #if cdist(C, pts[-1, :2].reshape(1, -1)) < r :
                ok = True
                
                cur_pt = (pts[-1, 0], pts[-1, 1])
                j = int(pts[-1, 2])
                lst_pt = (lst_points[j, 0], lst_points[j, 1])

                if deep_DEBUG : plt.plot([cur_pt[0], lst_pt[0]], [cur_pt[1], lst_pt[1]])

                if LineString((cur_pt, lst_pt)).crosses(no_go_zones) or ((not LineString((cur_pt, lst_pt)).within(safe_zones)) and not safe_zones.is_empty):
                    ok = False
                    
                if ok:
                    pts = pts[-1, :]

                    if points2.shape[0] == 1 :
                        points2 = pts
                    else:
                        points2 = np.vstack((points2, pts))

        if deep_DEBUG:
            plt.scatter(points[:, 0], points[:, 1])
            plt.scatter(points2[:, 0], points2[:, 1])
            plt.scatter(A[0, 0], A[0, 1])
            plt.scatter(B[0, 0], B[0, 1])
            plt.plot(*safe_zones.exterior.xy)
            r = 500
            # angles = [angles.min(), angles.max()]
            for a in secteurs:
                plt.plot((A[0, 0], A[0, 0]+r*np.cos(a)), (A[0, 1], A[0, 1]+r*np.sin(a)))
            plt.show()
            
        points = points2

    
        return points


    def run(self, A, B, no_go_zones=MultiPolygon(), safe_zones=Polygon(), def_ang=30, nb_iso=10, nb_secteurs=40, cargos={'0':{'pos':(-100, -200), 'v':0, 'cap':0}}, ):
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
        v_moy = self.polaire(45*np.pi/180, self.weather(pos=A))*0.7 # on suppose qu'on va a 70% de la VMG en moyenne
        pas_t = dAB/(nb_iso*v_moy)

        cout('Distance à parcourir : {}, vitesse estimée : {}, eta : {}'.format(dAB, v_moy, dAB/v_moy))

        # la variables points stocke les points des isochrones 
        points = [np.zeros((1, 3))]
        points[-1][0, 0] = A[0]
        points[-1][0, 1] = A[1]
        points[-1][0, 2] = 0

        # angles donne une discrétisation de def_ang angles sur 2pi ou 3/4 de 2pi
        angles = np.linspace(0, 2*np.pi, def_ang)
        angles_2 = np.linspace(0, 3/2*np.pi, int(3/4*def_ang))


        # c'est le cercle dans lequelle on peut faire les calculs, pour optimiser et que le voilier ne cherche pas à aller dans la direction opposée du point d'arrivée
        C = (A[0]+(A[0]+B[0])/2, A[1]+(A[1]+B[1])/2)
        cercle = []
        r = distanceAB(A, B)
        for a in angles:
            p = C[0]+r*np.cos(a), C[1]+r*np.sin(a)
            cercle.append(p)
        cercle_np = np.array(cercle)
        cercle = Polygon(cercle_np)
        if safe_zones.is_empty:
            safe_zones = cercle
        safe_zones = safe_zones.intersection(cercle)
        # plt.plot(*safe_zones.exterior.xy)
        # le deuxième cercle sert pour généré les points, le premier, plus petit pour selectionner les lignes (cf alphashape)
        cercle_2 = []
        r2 = r*1.2
        for a in angles:
            p = C[0]+r2*np.cos(a), C[1]+r2*np.sin(a)
            cercle_2.append(p)
        cercle2_np = np.array(cercle_2)
        cercle2 = Polygon(cercle2_np)



        i_iso = 0
        target_in = False
        target_near = False
        pas2 = pas_t #stockage du pas pour le réduire quand on s'approche de l'arrivée
        d_min = 1

        t_1 = []
        t_2 = []
        t_3 = []

        if deep_DEBUG:
            if not no_go_zones.is_empty:
                plt.plot(*no_go_zones.exterior.xy)
            if not safe_zones.is_empty:
                pass#plt.plot(*safe_zones.exterior.xy)


        # boucle principale
        while i_iso < 3*nb_iso and not target_in:

            t0 = time.time()

            if target_near:
                pas_t = pas2*d_min/dAB + 0.1*pas2
            iso = np.zeros((1, 3))

            # on update la pos des cargos :
            #no_go_zones = no_go_zones.union(self.zones_cargos(cargos, t, pas_t))
            no_go_zones = no_go_zones.union(self.zones_cargos(cargos, t, pas_t))

            # pour tous les points de l'isochrone précédente 
            for j in range(points[-1].shape[0]):
                pt = points[-1][j, :]

                # pour tous les angles 
                for a in angles:
                    x, y = polToCart(a, self.polaire(a, self.weather(pos = pt))*pas_t)
                    x += pt[0]
                    y += pt[1]
                                             
                    # ou pour 3/4 de cercle plutot ;)
                    #if (abs(getAngleABC(A, pt[:2], (x, y))) > np.pi/4 and cercle2.intersection(Point(x, y))) or i_iso < 1:
                    if iso.shape[0] == 1:
                        iso = np.array(((x[0], y[0], j)))
                    else:
                        iso = np.vstack((iso, np.array(((x[0], y[0], j)))))
            
                       
            t1 = time.time()
            t_1.append(t1 - t0)

            
            # pour toutes les iso sauf la première
            if i_iso > 0:
                cout('computing iso ({}): {}'.format(iso.shape[0], i_iso))
                if deep_DEBUG : plt.plot(cercle_np[:, 0], cercle_np[:, 1])
                pts = self.compute_iso2(iso, points[-1], A, B, no_go_zones, safe_zones, nb_secteurs)
            else:
                pts = iso

            points.append(pts)
            i_iso += 1

            t2 = time.time()
            t_2.append(t2 - t1)


            # conditions d'arrêt
            distancesB = cdist(np.array((B)).reshape(1, -1), pts[:, :2])
            d_min = distancesB.min()
            if d_min < 0.2*dAB/nb_iso:
                target_in = True
            elif d_min < 1.1*dAB/nb_iso:
                target_near = True
            if deep_DEBUG:
                if target_in:
                    cout("Target in !")
                elif target_near:
                    cout('Target near !')
                
            t += pas_t
   
        cout('temps : {}, {}'.format(np.mean(t_1), np.mean(t_2)))

        #pts_to_check = np.vstack((points[-1], points[-2]))
        pt = points[-1][cdist([np.array(((B[0], B[1])))], points[-1][:, :2]).argmin()]


        i = len(points) - 1
        j = int(pt[2])

        traj_x, traj_y = [pt[0], B[0]], [pt[1], B[1]]

        while i != 0:
            i -= 1
            #print(j)
            # print(points[i][:, :])            
            traj_x.insert(0, points[i][j, 0])
            traj_y.insert(0, points[i][j, 1])
            j = int(points[i][j, 2])
            

        return (traj_x, traj_y), points
    



if __name__ == "__main__":


    DEBUG = True
    deep_DEBUG = False
    d_min_ENABLED = False

    #s_z = Polygon(((-50, 50), (-50, -400), (450, -400), (450, 50), (350, 50), (350, -300), (50, -300), (50, 50)))

    
    routeur = Routeur()
    #routeur.show_alpha()
    routeur.polaire_plot()
    A, B = (0, 0), (420, 20)
    plt.scatter((A[0], B[0]), (A[1], B[1]))
    # C = (A[0]+(A[0]+B[0])/2, A[1]+(A[1]+B[1])/2)
    # cercle = []
    # r = distanceAB(A, B)/2*1.2
    # angles = np.linspace(0, 2*np.pi, 60)
    # for a in angles:
    #     p = C[0]+r*np.cos(a), C[1]+r*np.sin(a)
    #     cercle.append(p)
    # cercle_np = np.array(cercle)

    traj, iso = routeur.run(A, B)#, safe_zones=s_z)
    j = len(iso)-1
    for i in iso:
        plt.plot(i[:, 0], i[:, 1], color=[(0.8/len(iso)*j +0.2),(0.8/len(iso)*j+0.2), (0.8/len(iso)*j+0.2)])
        j -= 1
    for p_c in routeur.poly_cargos[:]:
        plt.plot(p_c[0], p_c[1])

    
    
    plt.plot(traj[0], traj[1], c='red')
    #plt.plot(cercle_np[:, 0], cercle_np[:, 1])
    plt.show()


