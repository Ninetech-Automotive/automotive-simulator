#include <webots/Robot.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <unistd.h>
#include <string>
#include <iostream>

using namespace webots;
using namespace std;

// Konstanten
const int TIME_STEP = 32;
const double MAX_SPEED = 6.28;
const double BLACK_LINE_VALUE = 157.0;
const double THRESHOLD = 20.0;
const double TURN_DURATION = 0.5;  // Sekunden für stärkere Kurven
const double SLOW_SPEED = MAX_SPEED * 0.3;

// Hilfsfunktion, um zu prüfen, ob der Sensor auf der Linie ist
bool isOnLine(double value) {
  return (BLACK_LINE_VALUE - THRESHOLD <= value) && (value <= BLACK_LINE_VALUE + THRESHOLD);
}

void emit(Emitter *emitter, string message) {
  emitter->send(message.c_str(), message.size() + 1);
  cout << "child sent message " << message << endl;
}

void onReceive(Emitter *emitter, string message) {
  if (message == "ping") {
      emit(emitter, "pong");
  }
}

void receive(Receiver *receiver, Emitter *emitter) {
  if (receiver->getQueueLength() > 0) {
    const void* data = receiver->getData();
    const char* messageChars = static_cast<const char*>(data);
    std::string message(messageChars);
    receiver->nextPacket();
    
    cout << "child received message " << message << endl;
    
    onReceive(emitter, message);
  }
}

void pointScanning(Robot* robot, DistanceSensor* ir[], Motor* leftMotor, Motor* rightMotor, Emitter* emitter, const int TIME_STEP) {
    double irValues[7];
    for (int i = 0; i < 7; ++i) {
        irValues[i] = ir[i]->getValue();
    }

    cout << "Sensorwert IR3: " << irValues[3] << endl;

    if (isOnLine(irValues[3])) {
        leftMotor->setVelocity(0.0);  // Motoren stoppen
        rightMotor->setVelocity(0.0);
        emit(emitter, "line_detected");
        cout << "Linie erkannt, stoppe für 1 Sekunde..." << endl;
        
        // Warten für 1 Sekunde, während der Roboter angehalten bleibt
        double stopTime = robot->getTime() + 1.0;
        while (robot->getTime() < stopTime) {
            robot->step(TIME_STEP);
        }

        // Nach der Pause, den Roboter wieder drehen lassen
        leftMotor->setVelocity(1.0);
        rightMotor->setVelocity(-1.0);
        cout << "Roboter dreht nach der Pause..." << endl;
        
        stopTime = robot->getTime() + 1.0;
        while (robot->getTime() < stopTime) {
            robot->step(TIME_STEP);
        }
    }
}


int main(int argc, char **argv) {
  cout << "Starting AutomotiveController" << endl;
  Robot *robot = new Robot();
  

  
  // IR-Sensoren initialisieren
  DistanceSensor *ir[7];
  string ir_sensors[7] = {"IR0", "IR1", "IR2", "IR3", "IR4", "IR5", "IR6"};
  for (int i = 0; i < 7; ++i) {
    ir[i] = robot->getDistanceSensor(ir_sensors[i]);
    ir[i]->enable(TIME_STEP);
  }
  
  // Motoren initialisieren
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  
  Receiver *receiver = robot->getReceiver("ParentChildReceiver");
  receiver->enable(32);
  Emitter *emitter = robot->getEmitter("ChildParentEmitter");
  
  int timeStep = (int)robot->getBasicTimeStep();
  
  while (robot->step(timeStep) != -1) {
  receive(receiver, emitter);
  leftMotor->setVelocity(1.0);
  rightMotor->setVelocity(-1.0);
  pointScanning(robot, ir, leftMotor, rightMotor, emitter, TIME_STEP);

} // Hier endet der while-Block

delete robot;
return 0;

}


