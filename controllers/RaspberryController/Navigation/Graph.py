import json
from Navigation.Waypoint import Waypoint
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from Navigation.Angle import Angle
from Navigation.Edge import Edge

class Graph:
    def __init__(self):
        self.waypoints = []
        self.initialize_waypoints()
        self.current_waypoint: Waypoint = None
        self.target_waypoint: Waypoint = None
        self.previous_waypoint: Waypoint = None

    def initialize_waypoints(self):
        s = Waypoint("S")
        h = Waypoint("H")
        g = Waypoint("G")
        f = Waypoint("F")
        i = Waypoint("I")
        a = Waypoint("A")
        c = Waypoint("C")
        b = Waypoint("B")
        self.waypoints = [s,h,g,f,i,a,c,b]

        with open('angles.json', 'r') as file:
            data = json.load(file)
        
        for waypoint_id, angles in data.items():
            waypoint = self.get_waypoint_by_id(waypoint_id)
            angles = []
            for outgoing_waypoint_id, angle in angles.items():
                edge = Edge()
                outgoing_waypoint = self.get_waypoint_by_id(outgoing_waypoint_id)
                angle = Angle(outgoing_waypoint, angle, edge)
                angles.append(angle)
            waypoint.set_angles(angles)
                
    def get_waypoint_by_id(self, id):
        return [w for w in self.waypoints if w.get_id() == id][0]

    def set_target_waypoint(self, target_waypoint_id):
        self.target_waypoint = self.get_waypoint_by_id(target_waypoint_id)

    def go_to_next_best_waypoint(self):
        next_best_waypoint = self.get_next_best_waypoint()
        next_best_waypoint.set_incoming_angle_by_id(self.current_waypoint.get_id())
        self.previous_waypoint = self.current_waypoint
        self.current_waypoint = next_best_waypoint
        return next_best_waypoint.get_id()
    
    def get_next_best_waypoint(self):
        self.calculate_shortest_path()
        angles = self.current_waypoint.get_angles()
        return min(angles, key=lambda a: a.get_waypoint().get_weight_to_target())

    def go_back_to_previous_waypoint(self):
        self.current_waypoint = self.previous_waypoint
        self.previous_waypoint = None

    def has_reached_target_waypoint(self):
        return self.current_waypoint == self.target_waypoint

    def update_waypoint(self, angle: float, waypoint_status: WaypointStatus, edge_status: EdgeStatus):
        angle = self.current_waypoint.update_angle(angle, waypoint_status, edge_status)
        return angle

    def calculate_shortest_path(self):
        # dijkstra
        self.current_waypoint.set_weight_to_target(0)
        while(self.has_next_unvisited_node()):
            current_node = self.get_next_unvisited_node()
            for angle in self.current_waypoint.get_angles():
                waypoint = angle.get_waypoint()
                edge = angle.get_edge()
                weight_to_target = self.current_waypoint.get_weight_to_target() + edge.get_weight()
                if (weight_to_target < waypoint.get_weight_to_target()):
                    waypoint.set_weight_to_target(weight_to_target)
            current_node.set_dijkstra_visited(True)

    def get_next_unvisited_node(self):
        unvisited_nodes = [w for w in self.waypoints if w.get_dijkstra_visited() == False]
        return min(unvisited_nodes, key=lambda n: n.get_weight_to_target())
    
    def has_next_unvisited_node(self):
        unvisited_nodes = [w for w in self.waypoints if w.get_dijkstra_visited() == False]
        return len(unvisited_nodes) > 0
