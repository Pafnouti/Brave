import numpy as np
import random
import matplotlib.pyplot as plt 



def polyGenerator(n=17, r_min=100, r_max=300, size=(700, 700)):
    '''
    génération du polygone à découvrir
    :param n: nombre de points
    :param r_min: rayon min, tous les points seront a une distance supérieur du centre
    :param r_max: rayon max, tous les points seront a une distance inférieur du centre
    :return: matrice 2*n, coordonnées des points du polygone
    '''

    angles = sorted([random.random()*2*np.pi for i in range(n)])
    rayons = [random.random()*(r_max - r_min)+r_min for i in range(n)]

    points = np.zeros((2, n))

    for i in range(n):
        x = size[0]/2 + rayons[i]*np.cos(angles[i])
        y = size[1]/2 + rayons[i]*np.sin(angles[i])

        points[0, i] = x
        points[1, i] = y

    return points

def distanceAB(A, B):
    x1, y1 = A
    x2, y2 = B
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def pointsToPara(A, B):
    a, b = 0, 0
    xa, ya = A
    xb, yb = B

    if xa - xb != 0:
        a = (yb - ya)/(xb - xa)
        b = ya - a*xa

    return a, b

def intersection(d1, d2):
    '''
    retourne le point d'intersection entre d1 et d2
    :param d1: (a1, b1) tuple des paramètres de la droite
    :param d2: (a2, b2)
    :return: bool d'intersection, coords intersection
    '''

    a1, b1 = d1
    a2, b2 = d2

    if a1 == a2:
        return False, (0, 0)

    else:
        x = (b2 - b1)/(a1 - a2)
        y = a1*x + b1
        return True, (x, y)

def estEntreAetB(C, A, B):
    xa, ya = A
    xb, yb = B
    xc, yc = C

    if xc <= max(xa, xb) and xc >= min(xa, xb):
        if yc <= max(ya, yb) and yc >= min(ya, yb):
            return True
    return False

def intersectionSegments(s1, s2):
    A, B = s1
    C, D = s2
    b, I = intersection(pointsToPara(A, B), pointsToPara(C, D))
    if b :
        if estEntreAetB(I, A, B) and estEntreAetB(I, C, D):
            return True, I
        else:
            return False, (0, 0)
    else:
        return False, (0, 0)


def distanceDroite(A, droite):
    '''
    Retourne le point de la droite le plus proche de A
    :param A: point sous forme de tuple (x, y)
    :param droite: paramèrtes a et b
    :return: le point de la droite le plus proche de A
    '''



    x1, y1 = A
    a, b = droite
    x2 = (x1 + a*y1 - a*b)/(1 + a**2)
    y2 = a*x2 + b

    return x2, y2


def intersectionCercleDroite(C, d, r):
    '''
    Donne les points d'intersection d'une droite et d'un cercle. Retourne une liste de points
    Le centre du cercle doit appartenir à la droite !!
    :param C: centre du cercle (x, y)
    :param d: paramètres droite (a, b)
    :param r: rayon du cercle
    :return: A, B = (xA, yA), (xB, yB)
    '''

    x0, y0 = C
    a, b = d
    points = []

    X = np.roots([1, -2*x0, x0**2 - r**2/(1 + a**2)])


    for x in X:
        y = a*x + b
        points.append((x, y))



    return points

def getAngle(A, B):
    '''
    retourne l'angle entre A, B et l'horizontale
    '''

    a, b, s = 0, 0, 0
    x, y = A
    dx, dy = B[0] - x, B[1] - y

    if dx != 0 and dy != 0:
        s_1 = dx / abs(dx)
        s_2 = (dy) / abs(dy)
        alpha = np.arctan(dy / dx)
        if s_1 < 0:
            alpha += np.pi
        elif s_2 < 0:
            alpha += 2*np.pi
    elif dx == 0:
        if dy < 0:
            return 3*np.pi/2
        else:
            return np.pi/2
    else:
        if dx < 0:
            return np.pi
        else:
            return 0

    return alpha

def getAngleABC(A, B, C):
    return getAngle(B, A) - getAngle(B, C)

def pointAtDistance(A, d, angle):
    return (A[0] + d*np.cos(angle), A[1] + d*np.sin(angle))

def rad(deg):
    return deg*np.pi/180

def deg(rad):
    return rad*180/np.pi

def plotPoly(polys, segments=[],points=[]):
    for poly in polys:
        X = [p[0] for p in poly]
        Y = [p[1] for p in poly]  
        plt.plot(X, Y)
    for s in segments:
        A, B = s
        plt.scatter([A[0], B[0]], [A[1], B[1]])
    for p in points:
        plt.scatter(p[0], p[1])
    plt.title("poly")
    plt.show()


def polToCart(angle, d):
    angle = np.array(angle)
    d = np.array(d)
    return d*np.cos(angle), d*np.sin(angle)

def cartToPol(x, y):
    x = np.array(x)
    y = np.array(y)
    return np.arctan2(y, x), np.sqrt(x**2 + y**2)

if __name__ == "__main__":
    x, y = [0, 1, 2, 3], [4, 5, 6, 7]
    a, d = cartToPol(x, y)
    z, t = polToCart(a, d)
    print(z, t)

