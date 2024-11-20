import random

class Communicator:

    def __init__(self, emitter, receiver, camera,timestep):
        self.emitter = emitter
        self.receiver = receiver
        self.camera = camera
        self.camera_enabled = False
        self.timestep = timestep
        self.angles = []

    def emit(self, message):
        self.emitter.send(message)
        print("parent sent message " + message)

    def onReceive(self, message):
        if message == "pong":
            self.handlePong()
        elif message == "line_detected":
            self.on_line_detected()
        elif message == "on_waypoint":
            self.start_point_scanning()
        elif message.startswith("angle:"):
            angle = message[6:]
            self.angles.append(angle)
            print(self.angles)
        elif message == "point_scanning_finished":
            self.choose_line()

    def receive(self):
        if self.receiver.getQueueLength() != 0:
            data = self.receiver.getBytes()
            self.receiver.nextPacket()
            message = data.decode("utf-8")
            message = message.rstrip("\x00")
            print("parent received message " + message)
            self.onReceive(message)
            
    def on_line_detected(self):
        print("Line detected! Enabling the camera...")
        if not self.camera_enabled:
            self.camera.enable(self.timestep)  # Aktivieren der Kamera
            self.camera_enabled = True
            print("Camera enabled.")
            

    def ping(self):
        self.emit("ping")
        
    def start_line_following(self):
        self.emit("follow_line")
        
    def start_point_scanning(self):
        self.emit("scan_point")
        
    def choose_line(self):
        number_of_angles = len(self.angles)
        line_choosen = str(random.randint(1, number_of_angles)) #choose a random line for now
        self.emit(f"target_line:{line_choosen}")
        print(line_choosen)
        self.angles.clear()

    def handlePong(self):
        print("Ping answer received")
