import numpy as np


class Simulator(object):
    def __init__(self, period=0.1):
        self.period = period
        self.orientation = None

    def get_position(self, position, velocity):
        alfa = self.orientation[2][0]
        R = np.array([[np.cos(alfa), -np.sin(alfa), 0],
                      [np.sin(alfa),  np.cos(alfa), 0],
                      [0,             0,            1]])
        position = position + np.dot(R, velocity[0:3]) * self.period
        return position

    def get_orientation(self, orientation, velocity):
        self.orientation = orientation + velocity[3:] * self.period
        return self.orientation
