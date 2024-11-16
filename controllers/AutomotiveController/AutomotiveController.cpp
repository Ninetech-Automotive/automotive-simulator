#include <webots/Robot.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <string>
#include <iostream>

using namespace webots;
using namespace std;

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

int main(int argc, char **argv) {
  cout << "Starting AutomotiveController" << endl;
  Robot *robot = new Robot();
  Receiver *receiver = robot->getReceiver("ParentChildReceiver");
  receiver->enable(32);
  Emitter *emitter = robot->getEmitter("ChildParentEmitter");
  int timeStep = (int)robot->getBasicTimeStep();
  
  while (robot->step(timeStep) != -1) {
    receive(receiver, emitter);
  };

  delete robot;
  return 0;
}


