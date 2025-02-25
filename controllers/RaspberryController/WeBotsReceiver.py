from Communication.Receiver import Receiver
from Navigation.NavigationController import NavigationController

class WeBotsReceiver(Receiver):
    def __init__(self, timestep, robot, controller: NavigationController):
        self.receiver = robot.getDevice("ChildParentReceiver")
        self.receiver.enable(timestep)
        self.controller = controller
        self.messageHandlers = {
            "pong": self.controller.on_pong,
            "on_waypoint": self.controller.on_waypoint,
            "angle": self.controller.on_angle,
            "point_scanning_finished": self.controller.on_point_scanning_finished,
            "turned_to_target_line": self.controller.on_turned_to_target_line,
            "cone_detected": self.controller.on_cone_detected,
            "obstacle_detected": self.controller.on_obstacle_detected
        }

    def on_receive(self, message):
        if ":" in message:
            message, value = message.split(":")
            self.messageHandlers[message](float(value))
        else:
            self.messageHandlers[message]()

    def receive(self):
        if self.receiver.getQueueLength() != 0:
            data = self.receiver.getBytes()
            self.receiver.nextPacket()
            message = data.decode("utf-8")
            message = message.rstrip("\x00")
            self.on_receive(message)