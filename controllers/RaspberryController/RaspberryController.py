from controller import Robot
import time

TIME_STEP = 32          

def emit(emitter, message):
    emitter.send(message)
    print("parent sent message " + message)
    
    
def onReceive(message):
    if (message == "pong"):
        print("Ping answer received")
    
    
def receive(receiver):    
    if receiver.getQueueLength() != 0:
        data = receiver.getBytes()
        receiver.nextPacket()
        message = data.decode("utf-8")
        message = message.rstrip('\x00')
        print("parent received message " + message)
        onReceive(message)


def ping(emitter):
    emit(emitter, "ping")
    
    
def main():
    robot = Robot()
    
    print("Starting RaspberryController")
    
    emitter = robot.getDevice("ParentChildEmitter")
    receiver = robot.getDevice("ChildParentReceiver")
    receiver.enable(TIME_STEP)

    camera = robot.getDevice('CAM')
    camera.enable(TIME_STEP * 2)

    width = camera.getWidth()
    height = camera.getHeight()
    print(f"Camera resolution: {width}x{height}")
    
    ping(emitter)

    while robot.step(TIME_STEP) != -1:
        receive(receiver)
    
        if camera.getSamplingPeriod() > 0:
            image = camera.getImage()
    

if __name__ == "__main__":
    main()