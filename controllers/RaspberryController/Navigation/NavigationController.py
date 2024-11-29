from Navigation.Graph import Graph
from Communicator import Communicator
from ObjectDetection.ObjectDetector import ObjectDetector

class NavigationController:
    def __init__(self, communicator: Communicator, object_detector: ObjectDetector):
        self.graph = Graph()
        self.communicator = communicator
        self.object_detector = object_detector
        self.angle_ids = []

    def startup_procedure(self):
        # TODO
        pass

    def start(self, target_waypoint: str):
        self.graph.set_target_waypoint(target_waypoint)
        self.communicator.emit("follow_line")
    
    def on_waypoint(self):
        self.communicator.emit("scan_point")

    def on_angle(self, angle: float):
        waypoint_status, edge_status = self.object_detector.detect_color()
        angle = self.graph.update_waypoint(angle, waypoint_status, edge_status)
        self.angle_ids.append(angle.get_waypoint().get_id())
        
    def on_point_scanning_finished(self):
        if (not self.graph.has_reached_target_waypoint()):
            next_best_waypoint_id = self.graph.go_to_next_best_waypoint()
            next_best_waypoint_index = self.angle_ids.index(next_best_waypoint_id)
            self.emit(f"target_line:{next_best_waypoint_index}")

    def on_turned_to_target_line(self):
        self.communicator.emit("follow_line")
