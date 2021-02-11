from matplotlib import pyplot
from shapely.geometry import Polygon, MultiPoint, Point
from descartes.patch import PolygonPatch
import numpy as np

b = np.array([
    [0, 0],
    [1, 0],
    [1, 1],
    [0, 1]
])

poly = Polygon(b)

point = MultiPoint([
    [.5, .5, 17],
    [1.5, .5, 12]
])
print(poly.intersection(point))
pyplot.plot(*poly.exterior.xy)
pyplot.show()
