import matplotlib.pyplot as plt
from perlin_noise import PerlinNoise
import numpy as np

class Wind():
    def __init__(self, width:int, height:int, noise_freq=0.006, noise_amp=2, constant_wind=(0, 0), roughness=0.5):
        self.t = 0
        self.width = width
        self.height = height
        self.noiseRough = PerlinNoise(octaves=noise_freq/15)
        self.noiseFine = PerlinNoise(octaves=noise_freq)
        self.noise_amp = noise_amp
        self.constant_wind = constant_wind
        self.roughness = roughness

    def getWind(self, x: int, y: int):  
        k1 = 5*(1 - self.roughness)
        k2 = 50 * self.roughness

        w = self.noise_amp * (k1 * (2*self.noiseFine([x, y, self.t]) - 1) + k2 * (2*self.noiseRough([x, y, self.t]) - 1))
        return w
    
    def timeIncrement(self):
        self.t += 1

    def getWindGrid(self, shape=(100, 100)):

        gx, gy = shape

        x = np.linspace(0, 1000, gx)
        y = np.linspace(0, 1000, gy)
        potential_grid = np.empty(shape)
        positions = np.empty((gx, gy, 2))

        for ind_x, pos_x in enumerate(x):
            print('Generating wind grid [%d%%]\r'%(100*pos_x/10), end="")
            for ind_y, pos_y in enumerate(y):
                w = self.getWind(pos_x, pos_y)
                positions[ind_x, ind_y] = np.array([pos_x, pos_y])
                potential_grid[ind_x, ind_y] = w
        
        return positions, np.gradient(potential_grid)

if __name__ == "__main__":
    import time
    t = time.time()
    w = Wind(100, 100, roughness=.8)
    print(w.getWind(10, 10))
    print(time.time() - t)

    x = np.arange(0, 100, 1)
    y = np.arange(0, 100, 1)

    _, u = w.getWindGrid()
    print(len(u))
    plt.quiver(x, y, u[0], u[1], angles='uv', scale=1, scale_units='xy')
    plt.show()

