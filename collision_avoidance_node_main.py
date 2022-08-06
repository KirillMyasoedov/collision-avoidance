#!/usr/bin/env python3
import rospy
from src import CollisionAvoidanceNodeFactory

if __name__ == '__main__':
    try:
        rospy.init_node("collision_avoidance_node")
        point_projector = CollisionAvoidanceNodeFactory.make_collision_avoidance_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
