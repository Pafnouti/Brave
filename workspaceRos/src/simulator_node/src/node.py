#!/usr/bin/python3
import numpy as np
import rospy
from controller.msg import Line
from geometry_msgs.msg import Pose2D

from display import *
import pygame
from voilier import Sailboat, sawtooth
from wind import Wind
import pickle
from scipy import interpolate
import os

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)

sailboat = Sailboat()
windGridSize = 100

try:
    f = open("wind.bin", 'rb')
    windpos, wind = pickle.load(f)
    # Do something with the file
except:
    print("No wind file found, generating one")
    windpos, wind = Wind(width, height, roughness=.8, noise_amp=16, constant_wind=(0, -10)).getWindGrid(shape=(windGridSize, windGridSize))

    nf = open("wind.bin", 'wb')
    pickle.dump((windpos, wind), nf)

target = (500 , 500)
print(windpos.shape)
clock = pygame.time.Clock()

ws = wind[0].shape

wind_f = interpolate.interp2d(np.linspace(0, 1000, ws[0]), np.linspace(0, 1000, ws[1]), np.linalg.norm(np.array(wind), axis=0), kind='linear')
try:
    back_file = open("background.png", 'r')
    background_image = pygame.image.load(back_file)
except:
    print("Background image not found, generating one...")
    background_image = pygame.Surface(size)
    for x in range(width):
        for y in range(height):
            print('Generating background image [%d%%]\r'%(100*x/width), end="")
            background_image.set_at((x, y), (np.clip(100 + int(100*wind_f(x, y)), 0, 255), np.clip(100 + int(100*wind_f(x, y)), 0, 255), 200))
    back_file = open("background.png", 'w')
    pygame.image.save(background_image, "background.png")

    rospy.init_node('navigator', anonymous=True)
    rospy.init_node('navigator', anonymous=True)

a = np.zeros((2,1))
b = np.ones((2,1))
m = np.zeros((2,1))

def line_callback(data):
    rospy.loginfo("Got new line")
    a[0, 0] = data.xa
    a[1, 0] = data.ya
    b[0, 0] = data.xb
    b[1, 0] = data.yb


rospy.init_node("simulator")
rospy.Subscriber("/Line", Line, line_callback)
state_pub = rospy.Publisher("/State", Pose2D)
counter = 0
while 1:
    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()
    dt = clock.tick(60)

    screen.blit(background_image, (0,0))
    drawSailboat(sailboat.x, 180*(np.pi/2 - sailboat.theta)/np.pi, 180*sailboat.sailAngle/np.pi, 180*sailboat.rudderAngle/np.pi)

    sx, sy = sailboat.x

    cx = sx + width/2
    cy = height/2-sy

    local_wind_x= wind[0][int(cx/width * windGridSize), int(cy/height * windGridSize)]
    local_wind_y= wind[1][int(cx/width * windGridSize), int(cy/height * windGridSize)]
    u = sailboat.control(a, b)
    drawBuoy(a.flatten(), color=(255,0,0))
    drawBuoy(b.flatten(), color=(255,100,100))
    sailboat.step(0, u, .1, np.array([local_wind_x, local_wind_y]))
    if counter >= 30:
        p = Pose2D()
        p.x = float(sx)
        p.y = float(sy)
        p.theta = float(-sailboat.theta)
        state_pub.publish(p)
        counter = 0
    counter += 1
    drawArrow(180 + 180/np.pi * np.arctan2(local_wind_y, local_wind_x), sailboat.x + np.array([50, 0]), scale=.1)
    pygame.display.flip()