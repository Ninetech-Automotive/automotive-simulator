import sys
sys.path.insert(1, './Core/src')
from controller import Robot
from Communication.Emitter import Emitter
from Communication.Receiver import Receiver
from ObjectDetection.ObjectDetector import ObjectDetector
from ObjectDetection.ColorDetector import ColorDetector
from Navigation.NavigationController import NavigationController
from WeBotsEmitter import WeBotsEmitter
from WeBotsReceiver import WeBotsReceiver
from ObjectDetection.Camera import Camera
from WeBotsCamera import WeBotsCamera
from Configuration.Configurator import Configurator
from pathlib import Path

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    Configurator.initialize(Path("config.json"))
    camera: Camera = WeBotsCamera(timestep, robot)
    object_detector: ObjectDetector = ColorDetector(camera)
    emitter: Emitter = WeBotsEmitter(robot)
    navigation_controller = NavigationController(emitter, object_detector)
    receiver: Receiver = WeBotsReceiver(timestep, robot, navigation_controller)

    target = sys.argv[1]
    navigation_controller.start(target)

    while robot.step(timestep) != -1:
        receiver.receive()


if __name__ == "__main__":
    main()
