from controller import Robot
import sys

TIME_STEP = 32

def run_robot(robot):
    # Initialize Camera
    camera = robot.getDevice('CAM')
    camera.enable(TIME_STEP * 2)  # Adjust sampling period (in ms), e.g., 64 ms

    # Get and print the camera resolution
    width = camera.getWidth()
    height = camera.getHeight()
    print(f"Camera resolution: {width}x{height}")

    while robot.step(TIME_STEP) != -1:
        # Capture camera image (optional)
        if camera.getSamplingPeriod() > 0:
            image = camera.getImage()

   

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)