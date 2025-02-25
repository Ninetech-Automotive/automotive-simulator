from Communication.Emitter import Emitter

class WeBotsEmitter(Emitter):
    def __init__(self, robot):
        self.emitter = robot.getDevice("ParentChildEmitter")

    def emit(self, message):
        print(f"[pi->µC] {message}")
        self.emitter.send(message)