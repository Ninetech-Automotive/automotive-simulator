from CommunicationReceiver import CommunicationReceiver

class Communicator:

    def __init__(self, emitter, receiver):
        self.emitter = emitter
        self.receiver = receiver
        self.communication_receiver = None

    def set_communication_receiver(self, communication_receiver: CommunicationReceiver):
        self.communication_receiver = communication_receiver

    def emit(self, message):
        print(f"[pi->µC] {message}")
        self.emitter.send(message)

    def onReceive(self, message):
        if message == "pong":
            self.handlePong()
        elif message == "on_waypoint":
            self.communication_receiver.on_waypoint()
        elif message.startswith("angle:"):
            angle = float(message[6:])
            self.communication_receiver.on_angle(angle)
        elif message == "point_scanning_finished":
            self.communication_receiver.on_point_scanning_finished()
        elif message == "turned_to_target_line":
            self.communication_receiver.on_turned_to_target_line()

    def receive(self):
        if self.receiver.getQueueLength() != 0:
            data = self.receiver.getBytes()
            self.receiver.nextPacket()
            message = data.decode("utf-8")
            message = message.rstrip("\x00")
            self.onReceive(message)
            
    def ping(self):
        self.emit("ping")

    def handlePong(self):
        pass
