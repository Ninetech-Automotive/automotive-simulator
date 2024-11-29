import random

class Communicator:

    def __init__(self, emitter, receiver, navigation_controller):
        self.emitter = emitter
        self.receiver = receiver
        self.navigation_controller = navigation_controller

    def emit(self, message):
        self.emitter.send(message)
        print("parent sent message " + message)

    def onReceive(self, message):
        if message == "pong":
            self.handlePong()
        elif message == "on_waypoint":
            self.navigation_controller.on_waypoint()
        elif message.startswith("angle:"):
            angle = message[6:]
            self.navigation_controller.on_angle(angle)
        elif message == "point_scanning_finished":
            self.navigation_controller.on_point_scanning_finished()
        elif message == "turned_to_target_line":
            self.navigation_controller.on_turned_to_target_line()

    def receive(self):
        if self.receiver.getQueueLength() != 0:
            data = self.receiver.getBytes()
            self.receiver.nextPacket()
            message = data.decode("utf-8")
            message = message.rstrip("\x00")
            print("parent received message " + message)
            self.onReceive(message)
            
    def ping(self):
        self.emit("ping")

    def handlePong(self):
        print("Ping answer received")
