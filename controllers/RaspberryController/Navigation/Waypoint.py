import sys
from typing import List
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from Navigation.Angle import Angle

class Waypoint:
    def __init__(self, id: str):
        self.status = WaypointStatus.UNKNOWN
        self.id = id
        self.angles: List[Angle] = []
        self.incoming_angle: float = 0.0
        self.weight_to_target: int = sys.maxsize
        self.dijkstra_visied = False

    def get_id(self):
        return self.id

    def set_angles(self, angles):
        self.angles = angles

    def set_status(self, status):
        self.status = status

    def set_incoming_angle_by_id(self, waypoint_id):
        angle = [a for a in self.angles if a.get_waypoint().get_id() == waypoint_id][0]
        self.incoming_angle = angle.get_value()

    def set_weight_to_target(self, weight: int):
        self.weight_to_target = weight

    def set_dijkstra_visited(self, visited: bool):
        self.dijkstra_visied = visited

    def get_dijkstra_visited(self):
        return self.dijkstra_visied

    def get_weight_to_target(self):
        return self.weight_to_target

    def get_angles(self):
        return self.angles
    
    def update_angle(self, value: float, waypoint_status: WaypointStatus, edge_status: EdgeStatus):
        angle = self.get_angle_from_value(value)
        angle.get_waypoint().set_status(waypoint_status)
        angle.get_edge().set_status(edge_status)
        return angle

    def get_angle_from_value(self, value: float):
        calculated_angle = self.calculate_angle_from_value(value)
        return min(self.angles, key=lambda a: abs(a.get_value() - calculated_angle))

    def calculate_angle(self, value: float):
        return (self.incoming_angle - 180.0) % 360 + value