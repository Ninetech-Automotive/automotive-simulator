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
const double TURN_DURATION = 0.5; // Sekunden für stärkere Kurven
const double SLOW_SPEED = MAX_SPEED * 0.3;
const double WHEEL_RADIUS = 22.5; // in mm
const double AXLE_LENGTH = 52;    // in mm

// Funktionsprototyp für followLine
void followLine(Robot *robot, DistanceSensor *ir[], DistanceSensor *coneSensor, DistanceSensor *obstacleSensor, Motor *leftMotor, Motor *rightMotor, double &leftSpeed, double &rightSpeed, double currentTime, double &turnEndTime, Emitter *emitter);

// Funktionsprototyp für followLine
void pointScanning(Robot *robot, DistanceSensor *ir[], Motor *leftMotor, Motor *rightMotor, Emitter *emitter, const int TIME_STEP);

// Funktionsprototyp für turnToLineX
void turnToLineX(Robot *robot, DistanceSensor *ir[], Motor *leftMotor, Motor *rightMotor, Emitter *emitter, const int TIME_STEP, const int targetLine);

// Hilfsfunktion, um zu prüfen, ob der Sensor auf der Linie ist
bool isOnLine(double value)
{
    return (BLACK_LINE_VALUE - THRESHOLD <= value) && (value <= BLACK_LINE_VALUE + THRESHOLD);
}

void emit(Emitter *emitter, string message)
{
    cout << "[µC->pi] " << message << endl;
    emitter->send(message.c_str(), message.size() + 1);
}

void onReceive(Emitter *emitter, string message, Robot *robot, DistanceSensor *ir[], DistanceSensor *coneSensor, DistanceSensor *obstacleSensor, Motor *leftMotor, Motor *rightMotor, double &leftSpeed, double &rightSpeed, double currentTime, double &turnEndTime)
{
    if (message == "ping")
    {
        emit(emitter, "pong");
    }
    else if (message == "follow_line")
    {
        // Wenn "follow_line" empfangen wird, rufe die followLine-Funktion auf
        followLine(robot, ir, coneSensor, obstacleSensor, leftMotor, rightMotor, leftSpeed, rightSpeed, currentTime, turnEndTime, emitter);

    }
    else if (message == "scan_point")
    {
        // Wenn "scan_point" empfangen wird, rufe die pointScanning-Funktion auf
        pointScanning(robot, ir, leftMotor, rightMotor, emitter, TIME_STEP);
    }
    else if (message.rfind("target_line:", 0) == 0)
    { // Checks if message starts with "target_line:"
        // Code to handle "target_line:" message
        string targetLineStr = message.substr(12); // Extract the part after "target_line:"
        int targetLine = std::stoi(targetLineStr);
        turnToLineX(robot, ir, leftMotor, rightMotor, emitter, TIME_STEP, targetLine);
    }
}

void receive(Receiver *receiver, Emitter *emitter, Robot *robot, DistanceSensor *ir[], DistanceSensor *coneSensor, DistanceSensor *obstacleSensor, Motor *leftMotor, Motor *rightMotor, double &leftSpeed, double &rightSpeed, double currentTime, double &turnEndTime)
{
    if (receiver->getQueueLength() > 0)
    {
        const void *data = receiver->getData();
        const char *messageChars = static_cast<const char *>(data);
        std::string message(messageChars);
        receiver->nextPacket();

        onReceive(emitter, message, robot, ir, coneSensor, obstacleSensor, leftMotor, rightMotor, leftSpeed, rightSpeed, currentTime, turnEndTime);
    }
}

void followLine(Robot *robot, DistanceSensor *ir[], DistanceSensor *coneSensor, DistanceSensor *obstacleSensor, Motor *leftMotor, Motor *rightMotor, double &leftSpeed, double &rightSpeed, double currentTime, double &turnEndTime, Emitter *emitter)
{
    double irValues[7];
    
    // coneSensor aktivieren
    coneSensor->enable(TIME_STEP);
    
    // obstacleSensor aktivieren
    obstacleSensor->enable(TIME_STEP);
    
        // Continue following the line until a waypoint is detected
    while (true)
    {
        // Get the coneSensor data
        double pyloneDistance = coneSensor->getValue();
        
        // Get the obstacleSensor data
        double obstacleDistance = obstacleSensor->getValue();

        // Get the IRsensor data
        for (int i = 0; i < 7; ++i)
        {
            irValues[i] = ir[i]->getValue();
        }
        
        // Check if there's a cone
        if (pyloneDistance < 980)
        {
            // Signal to RPI "cone_detected"
            emit(emitter, "cone_detected");
            
            //turn arround
            leftMotor->setVelocity(1.0);
            rightMotor->setVelocity(-1.0);
            
            // Wait for 4.5 seconds, while robot continues to drive
            double stopTime = robot->getTime() + 4.5; // 4.5 Sekunden
            while (robot->getTime() < stopTime)
            {
                robot->step(TIME_STEP);
            }

            // Stop motors after 1 second
            leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);
            continue;
        }
        
        if (obstacleDistance < 500)
        {
            
            // Signal to RPI "cone_detected"
            emit(emitter, "obstacle_detected");
            
            //stop for a second when obstacle is detected
            leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);
            
            // Wait for 1 second, while robot has stopped to signalize that a obstacle was detected
            double stopTime = robot->getTime() + 1; // 1 Sekunde
            while (robot->getTime() < stopTime)
            {
                robot->step(TIME_STEP);
            }
            
            // Drive "through" obstacle, since we don't remove it in the simulator
            leftMotor->setVelocity(MAX_SPEED);
            rightMotor->setVelocity(MAX_SPEED);
            
            // Wait for 3 seconds while robot continues to drive
            stopTime = robot->getTime() + 3; // 3 Sekunde
            while (robot->getTime() < stopTime)
            {
                robot->step(TIME_STEP);
            }
            
            continue;
        }
        
        // Wait till Robot has turned
        if (currentTime < turnEndTime)
        {
            robot->step(TIME_STEP); // Continue Simulation, while wating on the turn
            continue;
        }

        // Check if all Sensors are on the Waypoint (Waypoint detection)
        if (std::all_of(std::begin(irValues), std::end(irValues), isOnLine))
        {
            // Set Velocity
            leftMotor->setVelocity(leftSpeed * 0.7);
            rightMotor->setVelocity(rightSpeed * 0.7);

            // Wait for 1 second, while robot continues to drive
            double stopTime = robot->getTime() + 1.0; // 1 Sekunde
            while (robot->getTime() < stopTime)
            {
                robot->step(TIME_STEP);
            }

            // Stop motors after 1 second
            leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);

            // Signal to RPI "on_waypoint"
            emit(emitter, "on_waypoint");
            break;
        }
        else if (isOnLine(irValues[3]))
        { // Center sensor on line
            leftSpeed = MAX_SPEED;
            rightSpeed = MAX_SPEED;
        }
        else if (isOnLine(irValues[2]))
        { // Slight adjustment (left)
            leftSpeed = MAX_SPEED;
            rightSpeed = MAX_SPEED * 0.7;
        }
        else if (isOnLine(irValues[4]))
        { // Slight adjustment (right)
            leftSpeed = MAX_SPEED * 0.7;
            rightSpeed = MAX_SPEED;
        }
        else if (isOnLine(irValues[0]) || isOnLine(irValues[1]))
        { // Strong left turn
            leftSpeed = MAX_SPEED;
            rightSpeed = MAX_SPEED * 0.5;
            turnEndTime = currentTime + TURN_DURATION;
        }
        else if (isOnLine(irValues[5]) || isOnLine(irValues[6]))
        { // Strong right turn
            leftSpeed = MAX_SPEED * 0.5;
            rightSpeed = MAX_SPEED;
            turnEndTime = currentTime + TURN_DURATION;
        }
        else
        {
            leftSpeed = MAX_SPEED * 0.3;
            rightSpeed = MAX_SPEED * 0.6;
        }

        // Set velocity
        leftMotor->setVelocity(leftSpeed);
        rightMotor->setVelocity(rightSpeed);

        // next step for next cycle
        robot->step(TIME_STEP);
    }
}

// Function to calculate the turn angle
double calculateTurnAngle(double motorSpeed, double wheelRadius, double axleLength, double turnDuration)
{
    // linear velocity
    double linearSpeed = motorSpeed * wheelRadius;

    // Winkelgeschw.
    double omega = (2 * linearSpeed) / axleLength;

    // Angle in rad
    double theta = omega * turnDuration;

    // Angle in degrees
    double thetaDegrees = theta * (180.0 / M_PI);
    return thetaDegrees;
}

void pointScanning(Robot *robot, DistanceSensor *ir[], Motor *leftMotor, Motor *rightMotor, Emitter *emitter, const int TIME_STEP)
{
    double irValues[7];
    double startTurnTime = robot->getTime();
    double stopTurnTime = 0;
    double turnDuration = 0;
    double totalAngle = 0;
    double angleArray[5]; // Array to store the angles of the lines
    int angleIndex = 0;

    leftMotor->setVelocity(1.0);
    rightMotor->setVelocity(-1.0);
    while (true)
    {
        // Sensorwerte aktualisieren
        for (int i = 0; i < 7; ++i)
        {
            irValues[i] = ir[i]->getValue();
        }

        // Wenn der mittlere Sensor (IR3) auf der Linie ist
        if (isOnLine(irValues[3]))
        {
            leftMotor->setVelocity(0.0); // Motoren stoppen
            rightMotor->setVelocity(0.0);
            stopTurnTime = robot->getTime();
            turnDuration = turnDuration + stopTurnTime - startTurnTime;

            // Berechne den Drehwinkel in Grad
            double angle = calculateTurnAngle(0.8, WHEEL_RADIUS, AXLE_LENGTH, turnDuration);

            // Überprüfen, ob der Roboter 360° gedreht hat
            if (angle >= 360.0)
            {
                emit(emitter, "point_scanning_finished");
                break; // Schleife beenden, wenn 360° erreicht wurden
            }

            // Verhindert, dass eine Linie bei 0° und 360° doppel gezählt wird
            if (angle >= 340.0)
            {
                continue;
            }

            string message = "angle:" + to_string(angle);
            emit(emitter, message);

            angleArray[angleIndex] = angle;
            angleIndex++;

            // Warten für 1 Sekunde, während der Roboter angehalten bleibt
            double stopTime = robot->getTime() + 1.0;
            while (robot->getTime() < stopTime)
            {
                robot->step(TIME_STEP);
            }

            // Nach der Pause, den Roboter wieder drehen lassen
            leftMotor->setVelocity(1.0);
            rightMotor->setVelocity(-1.0);
            startTurnTime = robot->getTime();

            stopTime = robot->getTime() + 0.5;
            while (robot->getTime() < stopTime)
            {
                robot->step(TIME_STEP);
            }

            continue;
        }

        // Schritte fortsetzen
        robot->step(TIME_STEP);
    }
}

void turnToLineX(Robot *robot, DistanceSensor *ir[], Motor *leftMotor, Motor *rightMotor, Emitter *emitter, const int TIME_STEP, const int targetLine)
{
    double irValues[7];
    int lineIndex = 0;

    leftMotor->setVelocity(1.0);
    rightMotor->setVelocity(-1.0);
    while (true)
    {
        // Sensorwerte aktualisieren
        for (int i = 0; i < 7; ++i)
        {
            irValues[i] = ir[i]->getValue();
        }

        // Wenn der mittlere Sensor (IR3) auf der Linie ist
        if (isOnLine(irValues[3]))
        {

            if (lineIndex == targetLine)
            {
                leftMotor->setVelocity(0.0); // Motoren stoppen
                rightMotor->setVelocity(0.0);
                emit(emitter, "turned_to_target_line");
                break;
            }

            double stopTime = robot->getTime() + 0.5;
            while (robot->getTime() < stopTime)
            {
                robot->step(TIME_STEP);
            }

            lineIndex++;
            continue;
        }

        // Schritte fortsetzen
        robot->step(TIME_STEP);
    }
}

int main(int argc, char **argv)
{
    Robot *robot = new Robot();

    // IR-Sensoren initialisieren
    DistanceSensor *ir[7];
    string ir_sensors[7] = {"IR0", "IR1", "IR2", "IR3", "IR4", "IR5", "IR6"};
    for (int i = 0; i < 7; ++i)
    {
        ir[i] = robot->getDistanceSensor(ir_sensors[i]);
        ir[i]->enable(TIME_STEP);
    }
    
    // Sensor für Hinderniss initialisieren
    DistanceSensor *obstacleSensor = robot->getDistanceSensor("obstacle_sensor");
    
    // Sensor für Pylone initalisieren
    DistanceSensor *coneSensor = robot->getDistanceSensor("cone_sensor");

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

    double leftSpeed = 0.0, rightSpeed = 0.0;
    double turnEndTime = 0;

    while (robot->step(timeStep) != -1)
    {
        double currentTime = robot->getTime();
        receive(receiver, emitter, robot, ir, coneSensor, obstacleSensor, leftMotor, rightMotor, leftSpeed, rightSpeed, currentTime, turnEndTime);
        // pointScanning(robot, ir, leftMotor, rightMotor, emitter, TIME_STEP);

        // followLine(robot, ir, leftMotor, rightMotor, leftSpeed, rightSpeed, currentTime, turnEndTime);

    } // Hier endet der while-Block

    delete robot;
    return 0;
}
