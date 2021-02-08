import numpy as np
import matplotlib.pyplot as plt 
from collision import *
from scipy import interpolate
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
import networkx as nx

class Routeur():
    def __init__(self):
        self.data_polaire = [[0*np.pi/180, 10*np.pi/180, 30*np.pi/180, 40*np.pi/180, 90*np.pi/180, 110*np.pi/180, 150*np.pi/180, 180*np.pi/180], [0.01, 0.05, 0.4, 0.8, 1, 0.9, 0.88, 0.8]]
        self.inter_polaire = interpolate.interp1d(self.data_polaire[0], self.data_polaire[1], kind='cubic')
        self.vitesse_max = 5
        self.vent_ideal = 10


    def polaire(self, alpha, vent_=1):
        vent = self.weather()
        a = alpha - vent[1]
        if a > np.pi: a = 2*np.pi - a
        if a < 0: a = - a
        #return self.inter_polaire(a)*np.cos(abs(vent[0] - self.vent_ideal)/self.vent_ideal)*self.vitesse_max
        return self.inter_polaire(a)*self.vitesse_max

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

    def get_path(self, Pr, i, j):
        path = [j]
        k = j
        while Pr[i, k] != -9999:
            path.append(Pr[i, k])
            k = Pr[i, k]
        return path[::-1]

    def test(self):
        pass



    def run(self, A, B, pas=1):

        d = distanceAB(A, B)
        angleAB = getAngle(A, B)
        theta = angleAB + np.pi/2
        R = np.array(((np.cos(theta), -np.sin(theta)), (np.sin(theta), np.cos(theta))))

        i_max = int(d/pas)
        graph = np.zeros((i_max**2, i_max**2))

        for i in range(i_max**2):
            for j in range(i):
                pos_i = R@np.array(((i%i_max), (i//i_max)))
                pos_j = R@np.array(((j%i_max), (j//i_max)))
                angle_i_j = getAngle(pos_i, pos_j)
                v = self.polaire(angle_i_j)
                if v == 0 : v = 0.0001
                p = 1/v
                graph[i][j] = p
                graph[j][i] = p

  
        
        G = nx.from_numpy_array(graph)
        print(nx.nodes(G))
        path = nx.shortest_path(G, 0, i_max*2-1, weight='weight')


        X, Y = [], []
        for node in path:
            pos_i = R@np.array(((node%i_max), (node//i_max)))
            print(pos_i)
            X.append(pos_i[0])
            Y.append(pos_i[1])

        plt.plot(X, Y)
        plt.scatter(A[0], A[1])
        plt.scatter(B[0], B[1])
        plt.show()
        '''
        graph = csr_matrix(graph)
        D, Pr = dijkstra(csgraph=graph, directed=False, indices=0, return_predecessors=True)
        print(Pr)
        print(self.get_path(Pr, 0, i_max))'''



if __name__ == "__main__":
    routeur = Routeur()
    #routeur.polaire_plot()

    routeur.run((0, 0), (4, 4))
