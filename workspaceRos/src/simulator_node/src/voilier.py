import numpy as np
from numpy.core.function_base import linspace
import pandas as pd
from scipy import interpolate
from scipy import signal
import sys

def sawtooth(x):
    return signal.sawtooth(x + np.pi) * np.pi

class Sailboat(object):
    """
    Defines a sailboat.
    """
    def __init__(self) -> None:
        self.x = np.array([0, 0]).astype('float64')
        self.v = .02
        self.theta = 0 # RollPitchYaw / phi, _, theta
        self.theta_dot = 0
        self.twd = 0
        self.twa = 0
        self.sailAngle = 0
        self.rudderAngle = 0
        self.q = 1
        polar = pd.read_csv("boat_Express.csv", sep=';')
        tws = polar.columns[1:].astype("float")
        twa = polar.values[:,0]
        self.polar_f = interpolate.interp2d(tws, twa, polar.values[:, 1:], kind='linear')
        
        self.beta = .05
        self.J = 3000.0
        self.rg = 4.0

        self.alphatheta = 2000.0
        self.m = 1000.0
        self.alphaf = 100.0
        self.rv = 2.0
        self.alphag = 250.0
        self.l = 2.0
        self.alphav = 500.0
        self.V = 100.0
        self.Jv = 3000.0 #moment d'inertie par rapport � l'axe (ox) du voilier
    
    
    def step(self, u_sail : float, u_rudder : float, dT : float, wind : tuple) -> float:
        """
            Input in radians
        """
        yaw = self.theta

        self.sailAngle = u_sail
        self.rudderAngle = u_rudder

        #force_sail = self.alphav * self.v * np.cos(yaw + u_sail) - self.alphav * self.v * np.sin(u_sail)
        #force_rudder = self.alphag * self.v * np.sin(u_rudder)
        dx = np.array([
            self.v * np.cos(yaw) * dT,
            self.v * np.sin(yaw) * dT
        ]).astype('float64').flatten()
        self.x += dx

        #self.v += (1 / self.m) * (np.sin(u_sail) * force_sail - np.sin(u_rudder) * force_rudder - self.alphaf * self.v) * dT
        wspd = 4*np.linalg.norm(wind)#knots
        self.twd = np.arctan2(wind[1], wind[0])
        twa = sawtooth(self.theta - self.twd)
        if(np.abs(twa) < np.pi/3):
            self.sailAngle = 0
        elif np.abs(twa) < 3*np.pi/4:
            self.sailAngle = -np.sign(twa) * np.pi/4
        else:
            self.sailAngle = -np.sign(twa) * np.pi/2


        self.v = 1*self.polar_f(wspd, np.abs(twa*180/np.pi))#Symmetry
        """ if float(self.v) < 0:
            self.v = 0 #Cheating for test purposes """
        """ print("Speed : ", self.v)
        print("Theta : ", self.theta) """
        self.theta += self.theta_dot * dT
        self.theta_dot = u_rudder

        self.twa = twa
        #self.theta_dot += (1 / self.J) * ((self.l - self.rv * np.cos(u_sail)) * force_sail - self.rg * np.cos(u_rudder) * force_rudder - self.alphatheta * self.theta) * dT

    def control(self, a, b):
        r = 20
        zeta = 0.3*np.pi #close hauled angle

        theta = self.theta  # cap robot
        phi = np.arctan2(b[1, 0]-a[1, 0], b[0, 0]-a[0, 0])
        cap = sawtooth(theta-phi)

        #delta_r = (1/np.pi)*cap 
        m = self.x.reshape((2,1))
        # ------------ Ajout ecart a la ligne --------
        ke = 0.5
        e = np.linalg.det(np.hstack((b-a, m-a)))/np.linalg.norm(b-a)
        thetaBar = phi - ke*np.arctan(e/r)
        #print("Ecart ligne : ", e)

        # ----------- Ajout "remonte au vent" ---------
        # zeta = angle a partir du quel on considere qu'on remonte au vent (pi/4 un peu fort --- pi/6 mieux ?)

        if abs(e) > r:  # on est en dehors du chenal
            self.q = np.sign(e)  # de quel cote on est de la ligne
        # si mon angle vent - angle boat < zeta
        print(" TWD : {:.2f} Theta : {:.2f} TWA : {:.2f} ThetaBar : {:.2f}".format(self.twd, sawtooth(theta), self.twa, thetaBar))
        if sawtooth(self.twd-thetaBar) < zeta:
            print("Je remonte le vent")
            thetaBar = self.twd - self.q*zeta
        
        delta_rmax = 1
        delta_r = (delta_rmax/np.pi) * \
            sawtooth(thetaBar-theta)  # de -pi/3 a pi/3
        return delta_r


if __name__=='__main__':
    v = Sailboat()

    import matplotlib.pyplot as plt

    x = linspace(0, 180, 500)
    y = linspace(0, 25, 200)
    
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    Z = np.empty((500, 200))
    for idx, i in enumerate(x):
        for idy, j in enumerate(y):
            Z[idx, idy] = v.polar_f(j, i)


    Y, X = np.meshgrid(y, x)
    print(X.shape, Y.shape, Z.shape)
    # Plot the surface.
    surf = ax.plot_surface(X, Y, Z,
                       linewidth=0, antialiased=False)
    plt.show()