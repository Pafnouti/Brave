import numpy as np
from scipy import interpolate
import pandas as pd



def grey(r, D):
    v = 255 - 255*(r/(1+D))
    return (v > 0)*v


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    img = np.empty((400, 400))
    cx, cy = c = np.array([200, 200])

    polar = pd.read_csv("boat_Hermione.csv", sep=';')
    tws = polar.columns[1:].astype("float")
    twa = polar.values[:,0]
    polar_f = interpolate.interp2d(tws, twa, polar.values[:, 1:], kind='cubic')

    for ix in range(400):
        for iy in range(400):
            print('Generating background image [%d%%]\r'%(100*ix/400), end="")
            px = np.array([ix, iy])

            dx = c-px
            d = np.linalg.norm(dx)
            ang = np.arctan2(dx[1], dx[0])

            D = polar_f(15, np.abs(180*ang/np.pi))
            img[ix, iy] = grey(d, 20*max(0, float(D)))
            
    
    
    plt.imshow(img)
    plt.show()
    
    plt.show()