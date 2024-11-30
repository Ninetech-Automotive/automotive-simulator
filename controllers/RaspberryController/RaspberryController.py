from controller import Robot
from Communicator import Communicator
from ObjectDetection.ObjectDetector import ObjectDetector
from Navigation.NavigationController import NavigationController

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    emitter = robot.getDevice("ParentChildEmitter")
    receiver = robot.getDevice("ChildParentReceiver")
    receiver.enable(timestep)
    camera = robot.getDevice("CAM")

    object_detector = ObjectDetector(camera, timestep, robot)
    communicator = Communicator(emitter, receiver)
    navigation_controller = NavigationController(communicator, object_detector)
    communicator.set_communication_receiver(navigation_controller)

    communicator.ping()
    navigation_controller.start("A")

    while robot.step(timestep) != -1:
        communicator.receive()


if __name__ == "__main__":
    main()
