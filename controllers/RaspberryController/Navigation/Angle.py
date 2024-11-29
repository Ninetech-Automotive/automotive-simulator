from Navigation.Edge import Edge
from Navigation.Waypoint import Waypoint

class Angle:
    def __init__(self, waypoint: Waypoint, value: float, edge: Edge):
        self.waypoint = waypoint
        self.value = value
        self.edge = edge

    def get_value(self):
        return self.value
    
    def get_edge(self):
        return self.edge

    def get_waypoint(self):
        return self.waypoint