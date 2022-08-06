from .input_data_adapter import InputDataAdapter
from .collision_zones_manager import CollisionZonesManager
from .output_data_adapter import OutputDataAdapter
from .collision_avoidance_node import CollisionAvoidanceNode
from .simulator import Simulator


class CollisionAvoidanceNodeFactory(object):
    @staticmethod
    def make_collision_avoidance_node():
        input_data_adapter = InputDataAdapter()
        collision_zones_manager = CollisionZonesManager()
        output_data_adapter = OutputDataAdapter()
        simulator = Simulator()
        collision_avoidance_node = CollisionAvoidanceNode(input_data_adapter,
                                                          collision_zones_manager,
                                                          output_data_adapter,
                                                          simulator)
        return collision_avoidance_node
