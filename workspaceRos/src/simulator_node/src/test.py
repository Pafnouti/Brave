import numpy as np
from numpy.core.defchararray import index
from scipy import interpolate
import pandas as pd
from scipy.ndimage import correlate


def grey(r, D):
    v = 255 - 255*(r/(1+D))
    return (v > 0)*v

a = 0

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    mask = np.zeros((2000, 2000))
    for x in range(1000):
        mask[x, x:2000-x] += 1
    imgPolar = np.empty((110, 120)).astype('int32')
    cx, cy = c = np.array([55, 60])

    polar = pd.read_csv("boat_Express.csv", sep=';')
    tws = polar.columns[1:].astype("float")
    twa = polar.values[:,0]
    polar_f = interpolate.interp2d(tws, twa, polar.values[:, 1:], kind='cubic')

    for ix in range(110):
        a += 1
        for iy in range(120):
            print('Generating background image [%d%%]\r'%(100*ix/200), end="")
            px = np.array([ix, iy])

            dx = c-px
            d = np.linalg.norm(dx)
            ang = np.arctan2(dx[1], dx[0])

            spd = polar_f(15, np.abs(180*ang/np.pi))
            t = d/spd
            imgPolar[ix, iy] = t < 8

    img = np.zeros((2000, 2000)).astype('int32')

    def drawPolarAt(x, y):
        img[x-cx:x+cx, y-cy:y+cy] =  img[x-cx:x+cx, y-cy:y+cy] | imgPolar
    
    drawPolarAt(1000, 1000)
    isos = []

    for t in range(6):
        print(t)
        isos.append(img.copy())
        border = np.linalg.norm(np.array(np.gradient(img)), axis=0) * mask
        indexes = np.where(border > 0)
        print(len(indexes[0]))
        for x, y in zip(indexes[0], indexes[1]):
            drawPolarAt(x, y)
            img = (img > 0).astype('int32')



    borders = np.zeros(img.shape)
    for i in isos:
        border = np.linalg.norm(np.array(np.gradient(i)), axis=0) * mask
        borders += border


    plt.imshow(correlate(isos[-1], imgPolar))
    plt.show()