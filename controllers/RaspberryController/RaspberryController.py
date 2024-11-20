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
    #camera.enable(timestep)

    communicator = Communicator(emitter, receiver, camera, timestep)
    object_detector = ObjectDetector(camera)

    communicator.ping()
    communicator.start_line_following()

    while robot.step(timestep) != -1:
        communicator.receive()
        robot.step(timestep)
        if communicator.camera_enabled:
            object_detector.detect_color()
            robot.step(timestep)

            camera.disable()
            communicator.camera_enabled = False
            #if object_detector.detect_color():
                #camera.disable()
                #communicator.camera_enabled = False
            print("Camera disabled after color detection.")


if __name__ == "__main__":
    main()
