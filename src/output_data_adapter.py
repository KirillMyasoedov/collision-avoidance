import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header


class OutputDataAdapter(object):
    def __init__(self, contour_topic='/contours', velocity_topic='/velocity', point_cloud_topic='/point_cloud'):
        self.__contours_publisher = rospy.Publisher(contour_topic, MarkerArray, queue_size=10)
        self.__velocity_publisher = rospy.Publisher(velocity_topic, Twist, queue_size=10)
        self.__point_cloud_publisher = rospy.Publisher(point_cloud_topic, PointCloud2, queue_size=10)

    def create_contours_message(self, contours, is_obstacles, names):
        p = Point()
        contours_message = MarkerArray()
        for contour, is_obstacle, name in zip(contours, is_obstacles, names):
            line = []
            for i in range(contour.shape[1]):
                p = Point()
                p.x = contour[0, i]
                p.y = contour[1, i]
                p.z = contour[2, i]
                line.append(p)

            marker = Marker()
            marker.lifetime = rospy.Duration(1)
            marker.id = 0
            marker.ns = "line_" + name
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.action = Marker.ADD
            marker.scale.x = 0.02
            marker.pose.orientation.w = 1.
            marker.pose.position.x = 0
            marker.pose.position.y = 0
            marker.pose.position.z = 0
            marker.type = Marker.LINE_STRIP

            if is_obstacle:
                marker.color.r = 1.
            else:
                marker.color.b = 1.

            marker.color.a = 1.
            marker.points = line
            contours_message.markers.append(marker)

        return contours_message

    def create_velocity_message(self, velocity):
        velocity_message = Twist()
        velocity_message.linear.x = velocity[0]
        velocity_message.linear.y = velocity[1]
        velocity_message.linear.z = velocity[2]
        velocity_message.angular.x = velocity[3]
        velocity_message.angular.y = velocity[4]
        velocity_message.angular.z = velocity[5]

        return velocity_message

    def create_point_cloud_message(self, point_cloud):
        point_cloud = point_cloud.T.tolist()

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]

        header = Header()
        header.frame_id = "map"

        point_cloud_message = point_cloud2.create_cloud(header, fields, point_cloud)

        return point_cloud_message

    def publish_messages(self, contours, is_obstacles, velocity, names):
        contours_message = self.create_contours_message(contours, is_obstacles, names)
        velocity_message = self.create_velocity_message(velocity)
        # point_cloud_message = self.create_point_cloud_message(point_cloud)

        self.__contours_publisher.publish(contours_message)
        self.__velocity_publisher.publish(velocity_message)
        # self.__point_cloud_publisher.publish(point_cloud_message)
