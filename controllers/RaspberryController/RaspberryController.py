import sys
from controller import Robot
from Communicator import Communicator
from ObjectDetection.ObjectDetector import ObjectDetector
from ObjectDetection.ColorDetector import ColorDetector
from Navigation.NavigationController import NavigationController
from CommunicationReceiver import CommunicationReceiver
from WeBotsCommunicator import WeBotsCommunicator
from ObjectDetection.Camera import Camera
from ObjectDetection.WeBotsCamera import WeBotsCamera

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    camera: Camera = WeBotsCamera(timestep, robot)
    object_detector: ObjectDetector = ColorDetector(camera)
    communicator: Communicator = WeBotsCommunicator(timestep, robot)
    navigation_controller: CommunicationReceiver = NavigationController(communicator, object_detector)
    communicator.set_communication_receiver(navigation_controller)

    communicator.ping()
    target = sys.argv[1]
    navigation_controller.start(target)

    while robot.step(timestep) != -1:
        communicator.receive()


if __name__ == "__main__":
    main()
