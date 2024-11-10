from controller import Robot, Camera

# Constants
TIME_STEP = 32
MAX_SPEED = 6.28
BLACK_LINE_VALUE = 153
THRESHOLD = 20
TURN_DURATION = 0.5  # seconds, adjusted for stronger turns
SLOW_SPEED = MAX_SPEED * 0.3

def run_robot(robot):
    # Initialize motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    for motor in (left_motor, right_motor):
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)

    # Initialize 7 IR sensors (IR0 to IR6)
    ir_sensors = [robot.getDevice(f'IR{i}') for i in range(7)]
    for sensor in ir_sensors:
        sensor.enable(TIME_STEP)

    # Initialize Camera
    camera = robot.getDevice('CAM')
    camera.enable(TIME_STEP * 2)  # Adjust sampling period (in ms), e.g., 64 ms

    # Get and print the camera resolution
    width = camera.getWidth()
    height = camera.getHeight()
    print(f"Camera resolution: {width}x{height}")

    turn_end_time = 0

    while robot.step(TIME_STEP) != -1:
        current_time = robot.getTime()

        # Read IR sensor values
        ir_values = [sensor.getValue() for sensor in ir_sensors]
        print(f"IR values: {[f'{v:.2f}' for v in ir_values]}")

        # Capture camera image (optional)
        if camera.getSamplingPeriod() > 0:
            image = camera.getImage()
            print("Camera image captured")

        # Initialize motor speeds
        left_speed, right_speed = MAX_SPEED, MAX_SPEED

        if current_time < turn_end_time:
            # Continue the ongoing turn
            continue

        # Check if all sensors detect the line (waypoint detection)
        if all(is_on_line(value) for value in ir_values):
            print("Waypoint detected, looking for next line")
            left_speed = SLOW_SPEED * 0.5
            right_speed = SLOW_SPEED * 0.9
        elif is_on_line(ir_values[3]):
            print("Center sensor on line, moving fast")
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED
        elif is_on_line(ir_values[2]):  # Slight adjustment (left)
            print("Left inner sensor on line, steering slightly right")
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED * 0.7
        elif is_on_line(ir_values[4]):  # Slight adjustment (right)
            print("Right inner sensor on line, steering slightly left")
            left_speed = MAX_SPEED * 0.7
            right_speed = MAX_SPEED
        elif is_on_line(ir_values[0]) or is_on_line(ir_values[1]):  # Strong left turn
            print("Left edge sensor on line, turning sharply right")
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED * 0.5
            turn_end_time = current_time + TURN_DURATION
        elif is_on_line(ir_values[5]) or is_on_line(ir_values[6]):  # Strong right turn
            print("Right edge sensor on line, turning sharply left")
            left_speed = MAX_SPEED * 0.5
            right_speed = MAX_SPEED
            turn_end_time = current_time + TURN_DURATION
        else:
            print("No sensor on line, searching...")
            left_speed = MAX_SPEED * 0.3
            right_speed = MAX_SPEED * 0.6

        # Set motor velocities
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

def is_on_line(value):
    return BLACK_LINE_VALUE - THRESHOLD <= value <= BLACK_LINE_VALUE + THRESHOLD

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
