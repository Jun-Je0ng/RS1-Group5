#include "controller.h"
#include "skidsteer.h"

Controller::Controller() {
    // Initialize SkidSteer
    skidSteer = SkidSteer();  // Create the SkidSteer object
}

void Controller::sendVelocityCommand(double linear_x, double angular_z) {
    // Use SkidSteer to move the robot based on velocity commands
    skidSteer.move(linear_x, angular_z);
}
