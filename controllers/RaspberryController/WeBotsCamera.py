from Core.src.ObjectDetection.Camera import Camera

class WeBotsCamera(Camera):

    def __init__(self, timestep, robot):
        self.camera = robot.getDevice("CAM")
        self.timestep = timestep
        self.robot = robot

    def enable(self):
        self.camera.enable(self.timestep)
        self.robot.step(self.timestep)

    def disable(self):
        self.camera.disable()

    def get_width(self):
        return self.camera.getWidth()

    def get_height(self):
        return self.camera.getHeight()

    def get_image_array(self):
        return self.camera.getImageArray()