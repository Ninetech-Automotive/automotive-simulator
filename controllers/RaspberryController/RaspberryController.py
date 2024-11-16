from controller import Robot
from Communicator import Communicator
from ObjectDetector import ObjectDetector


def main():
    print("Starting RaspberryController")
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    emitter = robot.getDevice("ParentChildEmitter")
    receiver = robot.getDevice("ChildParentReceiver")
    receiver.enable(timestep)
    camera = robot.getDevice("CAM")
    camera.enable(timestep)

    communicator = Communicator(emitter, receiver)
    object_detector = ObjectDetector(camera)

    communicator.ping()

    while robot.step(timestep) != -1:
        communicator.receive()
        object_detector.detect_color()


if __name__ == "__main__":
    main()
