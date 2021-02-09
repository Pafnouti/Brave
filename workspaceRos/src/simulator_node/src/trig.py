import numpy as np

def sawtooth(x):
    """Deal with 2*PI modulo

    Input:
    ------
    x: rad 
    """
    return (x+np.pi) % (2*np.pi)-np.pi

def deg2rad(x):
    """
    Conversion deg to rad
    """
    return x*np.pi/180

def rad2deg(x):
    return 180*x/np.pi

def north2east(x):
    """
    Input:
    ------
    Wind direction in degres, 0 is pointing north
    clockwise rotation

    Return:
    -------
    Angle of the wind in rad, in trigonometric circle
    """

    x = deg2rad(x)
    return sawtooth(np.pi/2 - x)

def east2north(x):
    return 90 - rad2deg(x)