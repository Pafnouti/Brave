import numpy as np
from numpy.core.function_base import linspace
import pandas as pd
from scipy import interpolate
from scipy import signal

def sawtooth(x):
    return signal.sawtooth(x + np.pi) * np.pi

class Sailboat(object):
    """
    Defines a sailboat.
    """
    def __init__(self) -> None:
        self.x = np.array([500, 500]).astype('float64')
        self.v = .02
        self.theta = 0 # RollPitchYaw / phi, _, theta
        self.theta_dot = 0

        self.sailAngle = 0
        self.rudderAngle = 0

        polar = pd.read_csv("boat_Hermione.csv", sep=';')
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

        force_sail = self.alphav * self.v * np.cos(yaw + u_sail) - self.alphav * self.v * np.sin(u_sail)
        force_rudder = self.alphag * self.v * np.sin(u_rudder)
    
        dx = np.array([
            self.v * np.cos(-np.pi/2 + yaw) * dT,
            self.v * np.sin(-np.pi/2 + yaw) * dT
        ]).astype('float64').flatten()
        self.x += dx

        #self.v += (1 / self.m) * (np.sin(u_sail) * force_sail - np.sin(u_rudder) * force_rudder - self.alphaf * self.v) * dT
        wspd = 4*np.linalg.norm(wind)#knots
        
        twa = sawtooth(self.theta - np.arctan2(wind[1], wind[0]))
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

        return twa
        #self.theta_dot += (1 / self.J) * ((self.l - self.rv * np.cos(u_sail)) * force_sail - self.rg * np.cos(u_rudder) * force_rudder - self.alphatheta * self.theta) * dT

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