import numpy as np
import matplotlib.pyplot as plt

from .collision_zones import InnerCollisionZone, OuterCollisionZone


class CollisionZonesManager(object):
    def __init__(self):
        self.inner_collision_zone_first_type = InnerCollisionZone(r_inner=0.21)
        self.inner_collision_zone_second_type = InnerCollisionZone(r_inner=0.052)
        self.outer_collision_zone_first_type = OuterCollisionZone(r_inner=0.21, b=0.4, k=0.3)
        self.outer_collision_zone_second_type = OuterCollisionZone(r_inner=0.052, b=0.1, k=0.1)  # k=0.1

        self.common_velocity = None
        self.linear_velocities = []
        self.angular_velocities = []
        self.linear_velocity_initial = []
        self.angular_velocity_initial = []

    def get_contours_and_velocity(self,
                                  point_cloud_first_type,
                                  point_cloud_second_type,
                                  position,
                                  alpha,
                                  velocity):
        self.linear_velocity_initial.append((velocity[0:3] ** 2).sum() ** 0.5)
        self.angular_velocity_initial.append((velocity[3:] ** 2).sum() ** 0.5)

        # Calculation of velocities
        # First type
        distance_first_type, is_obstacle_1, in_inner_1 = \
            self.outer_collision_zone_first_type.calculate_distance(point_cloud_first_type,
                                                                    position,
                                                                    alpha)
        velocity_first_type = self.outer_collision_zone_first_type.get_velocity(distance_first_type, velocity)

        # Second type
        R = np.array([[np.cos(alpha), -np.sin(alpha), 0],
                      [np.sin(alpha), np.cos(alpha), 0],
                      [0, 0, 1]])
        shift = np.array([[0., 0.19, 0.]]).T
        position_second_type_1 = position + np.dot(R, shift)
        position_second_type_2 = position + np.dot(R, -shift)
        distance_second_type_1, is_obstacle_2, in_inner_2 = \
            self.outer_collision_zone_second_type.calculate_distance(point_cloud_second_type,
                                                                     position_second_type_1,
                                                                     alpha)
        distance_second_type_2, is_obstacle_3, in_inner_3 = \
            self.outer_collision_zone_second_type.calculate_distance(point_cloud_second_type,
                                                                     position_second_type_2,
                                                                     alpha)
        velocity_second_type_1 = self.outer_collision_zone_second_type.get_velocity(distance_second_type_1, velocity)
        velocity_second_type_2 = self.outer_collision_zone_second_type.get_velocity(distance_second_type_2, velocity)

        # Calculation of contours
        inner_contour_first_type = self.inner_collision_zone_first_type.get_contour(position)
        outer_contour_first_type = self.outer_collision_zone_first_type.get_contour(position, alpha, velocity_first_type)

        inner_contour_second_type_1 = self.inner_collision_zone_second_type.get_contour(position_second_type_1)
        inner_contour_second_type_2 = self.inner_collision_zone_second_type.get_contour(position_second_type_2)
        outer_contour_second_type_1 = self.outer_collision_zone_second_type.get_contour(position_second_type_1,
                                                                                        alpha,
                                                                                        velocity_second_type_1)
        outer_contour_second_type_2 = self.outer_collision_zone_second_type.get_contour(position_second_type_2,
                                                                                        alpha,
                                                                                        velocity_second_type_2)

        # Calculation common velocity
        # product = velocity_first_type[:3] * velocity_second_type_1[:3] * velocity_second_type_2[:3]
        product = velocity_first_type * velocity_second_type_1 * velocity_second_type_2
        self.common_velocity = np.sign(product) * (np.abs(product) ** (1/3))
        # common_velocity = np.vstack((common_velocity, velocity[3:]))
        # common_velocity = np.vstack((common_velocity, velocity))

        # Line names list
        names = ["inner_contour_line_first_type",
                 "outer_contour_line_first_type",
                 "inner_contour_line_second_type_1",
                 "inner_contour_line_second_type_2",
                 "outer_contour_line_second_type_1",
                 "outer_contour_line_second_type_2"]

        # Is obstacles list
        is_obstacles = [in_inner_1, is_obstacle_1, in_inner_2, in_inner_3, is_obstacle_2, is_obstacle_3]

        # Contours list
        contours = [inner_contour_first_type,
                    outer_contour_first_type,
                    inner_contour_second_type_1,
                    inner_contour_second_type_2,
                    outer_contour_second_type_1,
                    outer_contour_second_type_2]
        return contours, is_obstacles, names, self.common_velocity

    def save_velocity_in_plot(self):
        self.linear_velocities.append((self.common_velocity[0:3] ** 2).sum() ** 0.5)
        self.angular_velocities.append((self.common_velocity[3:] ** 2).sum() ** 0.5)

        t = 0.1 * np.arange(len(self.linear_velocities))
        plt.plot(t, self.linear_velocity_initial, '--', label='linear initial')
        plt.plot(t, self.linear_velocities, label='linear updated')
        plt.plot(t, self.angular_velocity_initial, '--', label='angular initial')
        plt.plot(t, self.angular_velocities, label='angular updated')
        plt.legend()
        plt.xlabel('Time, s')
        plt.ylabel('Velocity, m/s, rad/s')
        plt.title('Velocities change')
        plt.savefig('/home/kirill/catkin_ws/src/collision-avoidance/velocities_change.png')
        plt.clf()
