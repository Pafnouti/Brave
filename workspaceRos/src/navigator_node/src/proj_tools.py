def sawtooth(self, x):
    """Deal with 2*PI modulo

    Input:
    ------
    x: rad 
    """
    return (x+pi) % (2*pi)-pi


def deg2rad(self, x):
    """
    Conversion deg to rad
    """
    return x*2*pi/360


def north2east(self, x):
    """
    Input:
    ------
    Wind direction in degres, 0 is pointing north
    clockwise rotation

    Return:
    -------
    Angle of the wind in rad, in trigonometric circle
    """

    x = self.deg2rad(x)
    return self.sawtooth(pi/2 - x)


def WGS84_to_cart(self, lat, lon):
    """
    Input: gps coord decimal lat, lon
    Return: cartesian coord x, y with (lat0, lon0) the origin
    """
    x = (pi/180.)*EARTH_RADIUS*(lon-lon0)*cos((pi/180.)*lat)
    y = (pi/180.)*EARTH_RADIUS*(lat-lat0)
    return x, y


def cart_to_WGS84(self, x, y):
    """
    Input: cartesian coord x, y with (lat0, lon0) the origin
    Return: gps coord decimal lat, lon
    """
    EPSILON = 0.00000000001
    lat = y*180./pi/EARTH_RADIUS+lat0
    if abs(lat-90.) < EPSILON or abs(lat+90.) < EPSILON:
        lon = 0
    else:
        lon = (x/EARTH_RADIUS)*(180./pi)/cos((pi/180.)*(lat))+lon0
    return lat, lon


def rad2pwm(self, x, sail_name):
    """
    Bijection de [0, pi/2] sur [pwm_min, pwm_max] pour les voiles.
    [-pi/3,pi/3] pour rudder
    """
    if sail_name == "main":
        return (2/pi)*(self.pwm_max_main_sail - self.pwm_min_main_sail)*x + self.pwm_min_main_sail
    elif sail_name == "fore":
        return (2/pi)*(self.pwm_max_fore_sail - self.pwm_min_fore_sail)*x + self.pwm_min_fore_sail
    elif sail_name == "rudder":
        x = x+pi/3
        return (3/(2*pi))*(self.pwm_max_rudder - self.pwm_min_rudder)*x + self.pwm_min_rudder
