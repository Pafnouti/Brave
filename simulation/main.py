from display import *
import pygame
from voilier import Sailboat
from wind import Wind
import pickle
from scipy import interpolate

sailboat = Sailboat()
windGridSize = 100

try:
    f = open("wind.bin", 'rb')
    windpos, wind = pickle.load(f)
    # Do something with the file
except:
    print("No wind file found, generating one")
    windpos, wind = Wind(width, height).getWindGrid(shape=(windGridSize, windGridSize))

    nf = open("wind.bin", 'wb')
    pickle.dump((windpos, wind), nf)

target = (500 , 500)

clock = pygame.time.Clock()

ws = wind[0].shape

wind_f = interpolate.interp2d(np.linspace(0, 1000, ws[0]), np.linspace(0, 1000, ws[1]), np.linalg.norm(np.array(wind), axis=0), kind='linear')
try:
    back_file = open("background.png", 'r')
    s = pygame.image.load(back_file)
except:
    print("Background image not found, generating one...")
    s = pygame.Surface(size)
    for x in range(width):
        for y in range(height):
            print('Generating background image [%d%%]\r'%(100*x/width), end="")
            s.set_at((x, y), (np.clip(100 + int(100*wind_f(x, y)), 0, 255), np.clip(100 + int(100*wind_f(x, y)), 0, 255), 200))
    back_file = open("background.png", 'w')
    pygame.image.save(s, "background.png")


while 1:
    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()
    dt = clock.tick(60)

    screen.blit(s, (0,0))
    drawSailboat(sailboat.x, 180*sailboat.theta/np.pi, 180*sailboat.sailAngle/np.pi, 180*sailboat.rudderAngle/np.pi)

    sx, sy = sailboat.x
    local_wind_x= wind[0][int(sx/width * windGridSize), int(sy/height * windGridSize)]
    local_wind_y= wind[1][int(sx/width * windGridSize), int(sy/height * windGridSize)]
    twa = sailboat.step(0, 0, .1, np.array([local_wind_x, local_wind_y]))

    drawArrow(180 + 180/np.pi * np.arctan2(local_wind_y, local_wind_x), sailboat.x + np.array([50, 0]), scale=.1)
    pygame.display.flip()