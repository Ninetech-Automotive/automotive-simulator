from controller import Robot
from Communicator import Communicator
from ObjectDetection.ObjectDetector import ObjectDetector
from Navigation.NavigationController import NavigationController


def main():
    print("Starting RaspberryController")
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    emitter = robot.getDevice("ParentChildEmitter")
    receiver = robot.getDevice("ChildParentReceiver")
    receiver.enable(timestep)
    camera = robot.getDevice("CAM")
    #camera.enable(timestep)

    object_detector = ObjectDetector(camera, timestep)
    navigation_controller = NavigationController(communicator, object_detector)
    communicator = Communicator(emitter, receiver, navigation_controller)

    communicator.ping()
    navigation_controller.start()

    while robot.step(timestep) != -1:
        robot.step(timestep)
        communicator.receive()
        robot.step(timestep)


if __name__ == "__main__":
    main()
