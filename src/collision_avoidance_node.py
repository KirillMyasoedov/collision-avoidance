import numpy as np
import rospy
from time import time


class CollisionAvoidanceNode(object):
    def __init__(self,
                 input_data_adapter,
                 collision_zones_manager,
                 output_data_adapter,
                 simulator,
                 period=0.1,
                 position=np.array([0, 0, 0]).reshape(-1, 1),
                 orientation=np.array([0, 0, 0]).reshape(-1, 1)):
        self.__input_data_adapter = input_data_adapter
        self.__collision_zones_manager = collision_zones_manager
        self.__output_data_adapter = output_data_adapter
        self.__simulator = simulator
        self.__timer = rospy.Timer(rospy.Duration(period), self.timer_callback)

        self.position = position
        self.orientation = orientation

        simulator_parameter_name = rospy.search_param("simulation")
        self.simulation = rospy.get_param(simulator_parameter_name)

        plot_parameter_name = rospy.search_param("output_plot")
        self.output_plot = rospy.get_param(plot_parameter_name)

    def timer_callback(self, _):
        start_time = time()
        point_cloud_first_type, point_cloud_second_type = self.__input_data_adapter.get_grid_point_cloud()
        if not self.simulation:
            self.position, self.orientation = self.__input_data_adapter.get_position_orientation()
        velocity = self.__input_data_adapter.get_velocity_vector()

        if velocity is None or point_cloud_first_type is None:
            return None

        contours, is_obstacles, names, updated_velocity = \
            self.__collision_zones_manager.get_contours_and_velocity(point_cloud_first_type,
                                                                     point_cloud_second_type,
                                                                     self.position,
                                                                     self.orientation[2][0],
                                                                     velocity)
        if self.output_plot:
            self.__collision_zones_manager.save_velocity_in_plot()

        if self.simulation:
            self.orientation = self.__simulator.get_orientation(self.orientation, updated_velocity)
            self.position = self.__simulator.get_position(self.position, updated_velocity)

        self.__output_data_adapter.publish_messages(contours, is_obstacles, updated_velocity, names)
        # print("Collision avoidance rate", time() - start_time)
