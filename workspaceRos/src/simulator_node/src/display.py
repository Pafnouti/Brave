import sys, pygame
import numpy as np
import os

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)

pygame.init()

size = width, height = 1000, 1000
black = 0, 0, 0
blue = 0, 0, 255

screen = pygame.display.set_mode(size)

boat = pygame.image.load("boat.png")
boat = pygame.transform.scale(boat, (50, 25))
sailLength = 19
rudderOffsetFromBoatCentre = 23
rudderLength = 8


def showImage(image : pygame.Surface, pos : tuple, angleDeg : int):
    image_rot = pygame.transform.rotate(image, 90- angleDeg)
    image_rect = image_rot.get_rect()
    x, y = pos
    mvt = [
            x - image_rect.centerx,
            y - image_rect.centery
        ]
    image_rect = image_rect.move(
        mvt
    )

    screen.blit(image_rot, image_rect)

def drawArrow(angle : int, pos : tuple, color=(0, 0, 0), scale = 1.):
    surface = pygame.Surface((300, 300))
    surface.set_colorkey(black)
    if color == (0, 0, 0):
        color = 1, 1, 1
    pygame.draw.polygon(surface, color, ((0, 100), (0, 200), (200, 200), (200, 300), (300, 150), (200, 0), (200, 100)))
    
    ns = (int(300 * scale), int(300 * scale))
    surface = pygame.transform.scale(surface, ns)
    showImage(surface, pos, angle)

def drawSailboat(pos:tuple, heading:int, sailAngle:int, rudderAngle:int):
    """
        Inputs are in degrees
    """
    showImage(boat, pos, heading)
    cx, cy = pos
    
    absZeroAngle = (heading - 90)*np.pi/180 + np.pi
    absSailAngle = (sailAngle+heading - 90)*np.pi/180 + np.pi
    absRudderAngle = (rudderAngle+heading-90)*np.pi/180 + np.pi

    sailEndPos = int(cx + sailLength*np.cos(absSailAngle)), int(cy + sailLength*np.sin(absSailAngle))

    rudderStartPos = int(cx + rudderOffsetFromBoatCentre*np.cos(absZeroAngle)), int(cy + rudderOffsetFromBoatCentre*np.sin(absZeroAngle))
    rudderEndPos = int(rudderStartPos[0] + rudderLength*np.cos(absRudderAngle)), int(rudderStartPos[1] + rudderLength*np.sin(absRudderAngle))
    pygame.draw.line(screen, (255, 255, 0), pos, sailEndPos, width=3)
    pygame.draw.line(screen, (255, 0, 255), rudderStartPos, rudderEndPos, width=3)
