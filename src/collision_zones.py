import numpy as np


class CollisionZone(object):
    def __init__(self, r_inner):
        self.r_inner = r_inner
        self.phi = np.arange(0, 2 * np.pi + 0.1, 0.1)


class InnerCollisionZone(CollisionZone):
    def __init__(self, r_inner):
        super(InnerCollisionZone, self).__init__(r_inner)

    def get_contour(self, position):
        x = self.r_inner * np.cos(self.phi)[None, :]  # [1, :]
        y = self.r_inner * np.sin(self.phi)[None, :]
        z = np.zeros_like(x)
        contour = np.concatenate((x, y, z)) + position  # [3, :]

        return contour


class OuterCollisionZone(CollisionZone):
    def __init__(self, r_inner, b, k):
        super(OuterCollisionZone, self).__init__(r_inner)
        self.a = b
        self.b = b
        self.p = b
        self.e = 0
        self.c = 0  # focal distance
        self.k = k

    def get_contour(self, position, alpha, velocity):
        self.a = self.b + (velocity[0, :] ** 2 + velocity[1, :] ** 2 + velocity[2, :] ** 2) ** 0.5 * self.k
        self.e = np.sqrt(1 - (self.b / self.a) ** 2)
        self.p = self.b ** 2 / self.a
        self.c = self.a * self.e
        rho = self.p / (1 + self.e * np.cos(np.pi - self.phi + alpha))
        x = rho * np.cos(self.phi)[None, :]  # [1, :]
        y = rho * np.sin(self.phi)[None, :]
        z = np.zeros_like(x)
        contour = np.concatenate((x, y, z)) + position  # [3, :]

        return contour

    def calculate_distance(self, point_cloud, position, alpha):
        R = np.array([[np.cos(alpha), -np.sin(alpha), 0],
                      [np.sin(alpha), np.cos(alpha), 0],
                      [0, 0, 1]])
        # point_cloud_transformed = np.dot(R.T, point_cloud) - position  # [3, :]
        point_cloud_transformed = point_cloud - position

        cropped_point_cloud = point_cloud[:, (point_cloud[0, :] >= -self.a) * \
                                             (point_cloud[0, :] <= self.a) * \
                                             (point_cloud[1, :] >= - 2 * self.b) * \
                                             (point_cloud[1, :] <= 2 * self.b)]
        cropped_point_cloud = point_cloud_transformed

        if cropped_point_cloud.size == 0:
            distance = np.array([self.a])
            is_obstacle = False
            in_inner = False
        else:
            x = cropped_point_cloud[0, :]
            y = cropped_point_cloud[1, :]
            distance = np.sqrt(x ** 2 + y ** 2)
            phi = np.arctan2(y, x)
            rho = self.p / (1 + self.e * np.cos(np.pi - phi + alpha))

            in_ellipse = np.where(distance <= rho)
            in_inner = np.where(distance <= self.r_inner)[0].size > 0

            distance = np.append(self.a, distance[in_ellipse])
            is_obstacle = in_ellipse[0].size > 0

        distance = distance.min()

        return distance, is_obstacle, in_inner

    def get_velocity(self, distance, velocity):
        # delta_velocity = velocity[:3] * (self.a - distance) / (self.a - self.r_inner)
        delta_velocity = velocity * (self.a - distance) / (self.a - self.r_inner)
        # velocity[:3] -= delta_velocity
        # delta_velocity = velocity * (self.a - distance) ** 0.5 / ((self.a - self.r_inner) ** 0.5)
        velocity -= delta_velocity

        return velocity
