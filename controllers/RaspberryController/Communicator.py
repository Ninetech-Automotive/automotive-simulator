class Communicator:

    def __init__(self, emitter, receiver):
        self.emitter = emitter
        self.receiver = receiver

    def emit(self, message):
        self.emitter.send(message)
        print("parent sent message " + message)

    def onReceive(self, message):
        if message == "pong":
            self.handlePong()

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
