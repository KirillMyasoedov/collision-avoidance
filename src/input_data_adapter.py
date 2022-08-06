import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
import numpy as np
import threading


class InputDataAdapter(object):
    def __init__(self,
                 non_passable_grid_topic="/non_passable_grid",
                 passable_grid_topic="/passable_grid",
                 velocity_topic="/cmd_vel",
                 odometry_topic="/odom_proj"):
        self.__non_passable_grid_subscriber = rospy.Subscriber(non_passable_grid_topic,
                                                               OccupancyGrid,
                                                               self.__non_passable_grid_callback)
        self.__passable_grid_subscriber = rospy.Subscriber(passable_grid_topic,
                                                           OccupancyGrid,
                                                           self.__passable_grid_callback)
        self.__velocity_subscriber = rospy.Subscriber(velocity_topic, Twist, self.__velocity_callback)
        self.__odometry_subscriber = rospy.Subscriber(odometry_topic, Odometry, self.__odometry_callback)

        self.__non_passable_grid_data = None
        self.__passable_grid_data = None
        self.__velocity_data = None
        self.__odometry_data = None

    def __non_passable_grid_callback(self, non_passable_grid_data):
        self.__non_passable_grid_data = non_passable_grid_data

    def __passable_grid_callback(self, passable_grid_data):
        self.__passable_grid_data = passable_grid_data

    def __velocity_callback(self, velocity_data):
        self.__velocity_data = velocity_data

    def __odometry_callback(self, odometry_data):
        self.__odometry_data = odometry_data

    def get_grid_point_cloud(self):
        if self.__non_passable_grid_data is None or self.__passable_grid_data is None:
            return None, None

        non_passable_width = self.__non_passable_grid_data.info.width
        non_passable_height = self.__non_passable_grid_data.info.height
        non_passable_resolution = self.__non_passable_grid_data.info.resolution
        non_passable_origin = self.__non_passable_grid_data.info.origin

        non_passable_data = np.array(self.__non_passable_grid_data.data)
        non_passable_data = non_passable_data.reshape((non_passable_height, non_passable_width))

        passable_width = self.__passable_grid_data.info.width
        passable_height = self.__passable_grid_data.info.height
        passable_resolution = self.__passable_grid_data.info.resolution
        passable_origin = self.__passable_grid_data.info.origin

        passable_data = np.array(self.__passable_grid_data.data)
        passable_data = passable_data.reshape((passable_height, passable_width))

        x_first_type = np.where(non_passable_data == 100)[1].reshape(1, -1) * non_passable_resolution \
                       + non_passable_origin.position.x
        y_first_type = np.where(non_passable_data == 100)[0].reshape(1, -1) * non_passable_resolution \
                       + non_passable_origin.position.y
        z_first_type = np.zeros_like(x_first_type)

        point_cloud_first_type = np.concatenate((x_first_type, y_first_type, z_first_type))

        x_second_type = np.where(passable_data == 100)[1].reshape(1, -1) * passable_resolution \
                        + passable_origin.position.x
        y_second_type = np.where(passable_data == 100)[0].reshape(1, -1) * passable_resolution \
                        + passable_origin.position.y
        z_second_type = np.zeros_like(x_second_type)

        point_cloud_second_type = np.concatenate((x_second_type, y_second_type, z_second_type))

        return point_cloud_first_type, point_cloud_second_type

    def get_velocity_vector(self):
        if self.__velocity_data is None:
            return None

        v_x = self.__velocity_data.linear.x
        v_y = self.__velocity_data.linear.y
        v_z = self.__velocity_data.linear.z
        w_x = self.__velocity_data.angular.x
        w_y = self.__velocity_data.angular.y
        w_z = self.__velocity_data.angular.z
        velocity = np.array([[v_x, v_y, v_z, w_x, w_y, w_z]]).T

        return velocity

    def get_position_orientation(self):
        if self.__odometry_data is None:
            return None, None
        x = self.__odometry_data.pose.pose.position.x
        y = self.__odometry_data.pose.pose.position.y
        z = self.__odometry_data.pose.pose.position.z
        position = np.array([[x, y, z]]).T

        v_x = self.__odometry_data.twist.twist.linear.x
        v_y = self.__odometry_data.twist.twist.linear.y
        angle = np.arctan2(v_y, v_x)
        return position, angle
