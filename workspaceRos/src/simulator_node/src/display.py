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
    scx, scy = pos
    cx = scx + width/2
    cy = height/2-scy
    
    surface = pygame.Surface((300, 300))
    surface.set_colorkey(black)
    if color == (0, 0, 0):
        color = 1, 1, 1
    pygame.draw.polygon(surface, color, ((0, 100), (0, 200), (200, 200), (200, 300), (300, 150), (200, 0), (200, 100)))
    
    ns = (int(300 * scale), int(300 * scale))
    surface = pygame.transform.scale(surface, ns)
    showImage(surface, (cx, cy), angle)

def drawBuoy(point, color=(255,0,0)):
    scx, scy = point
    cx = scx + width/2
    cy = height/2-scy
    pygame.draw.circle(screen, color, (cx, cy), 15)

def drawSailboat(pos:tuple, heading:int, sailAngle:int, rudderAngle:int):
    """
        Inputs are in degrees
    """
    scx, scy = pos
    cx = scx + width/2
    cy = height/2-scy
    showImage(boat, (cx, cy), heading)
    absZeroAngle = (heading - 90)*np.pi/180 + np.pi
    absSailAngle = (sailAngle+heading - 90)*np.pi/180 + np.pi
    absRudderAngle = (rudderAngle+heading-90)*np.pi/180 + np.pi

    sailEndPos = int(cx + sailLength*np.cos(absSailAngle)), int(cy + sailLength*np.sin(absSailAngle))

    rudderStartPos = int(cx + rudderOffsetFromBoatCentre*np.cos(absZeroAngle)), int(cy + rudderOffsetFromBoatCentre*np.sin(absZeroAngle))
    rudderEndPos = int(rudderStartPos[0] + rudderLength*np.cos(absRudderAngle)), int(rudderStartPos[1] + rudderLength*np.sin(absRudderAngle))
    pygame.draw.line(screen, (255, 255, 0), (cx, cy), sailEndPos, width=3)
    pygame.draw.line(screen, (255, 0, 255), rudderStartPos, rudderEndPos, width=3)
