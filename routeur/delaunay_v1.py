import numpy as np
import matplotlib.pyplot as plt 
from collision import *
from scipy import interpolate
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from scipy.spatial.distance import cdist
import alphashape


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

    def compute_iso(self, points, d_min=4):
     
        alpha_shape = np.array(list(alphashape.alphashape(points, 0.43).exterior.coords))


        return alpha_shape

    def run(self, A, B, angle_calcul=rad(170), nb_traj=150, pas=1):
        print('Starting...')

        angle_AB = np.arctan2((B[1] - A[1]), (B[0] - A[0]))
        a_min, a_max = angle_AB-angle_calcul/2, angle_AB+angle_calcul/2
        print(a_min, a_max)
        plt.plot([A[0]+30*np.cos(a_max), A[0], A[0]+30*np.cos(a_min)], [A[1]+30*np.sin(a_max), A[1], A[1]+30*np.sin(a_min)])

        '''
        angles = np.linspace(0, 2*np.pi, nb_traj)
        points_tmp = [polToCart(angle, self.polaire(angle)) for angle in angles]
        points = [np.zeros((nb_traj, 2))]
        all_points = [A]
        hist_points = {0:-1}
        for i in range(nb_traj):
            x = points_tmp[i][0]+A[0]
            y = points_tmp[i][1]+A[1]
            points[0][i][0] = x
            points[0][i][1] = y
            if a_min < np.arctan2(y-A[1], x-A[0]) < a_max:
                all_points.append((x, y))
                hist_points[i+1] = 0
        pts = self.compute_iso(points[0])
        points[0] = pts
        angles = np.linspace(0, 2*np.pi, 40)'''

        points = [np.array(((A[0], A[1])))]
        print(points)
        angles = np.linspace(0, 2*np.pi, 40)
        all_points = [A]
        hist_points = {0:-1}

        i_iso = 0
        target_in = False
        while i_iso < 2 and not target_in:
            iso = []
            for pt in points[-1]:
                print(points[-1])
                i_pt = all_points.index((pt[0], pt[1]))
                if all_points[i_pt] != (pt[0], pt[1]):
                    print('no good')
                for a in angles:
                    x, y = polToCart(a, self.polaire(a))
                    x += pt[0]
                    y += pt[1]
                    iso.append((x, y))
                    all_points.append((x, y))
                    hist_points[len(all_points)-1] = i_pt
                

            tmp_points = np.zeros((len(iso), 2))
            for i in range(len(iso)):
                tmp_points[i][0] = iso[i][0]
                tmp_points[i][1] = iso[i][1]
            print('computing iso : {}'.format(i_iso))
            pts = self.compute_iso(tmp_points)
            points.append(pts)
            i_iso += 1
            if Polygon(pts).contains(Point(B[0], B[1])):
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
            plt.plot(pts[:,0], pts[:,1])
            #plt.scatter(pts[:,0], pts[:,1])
        plt.show()
        
            
        





if __name__ == "__main__":
    routeur = Routeur()
    #routeur.polaire_plot()

    routeur.run((0, 0), (30, 2))
