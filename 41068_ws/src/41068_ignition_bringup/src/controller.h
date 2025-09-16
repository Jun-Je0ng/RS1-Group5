#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "skidsteer.h"  // Include SkidSteer

class Controller {
public:
    Controller();  // Constructor
    void sendVelocityCommand(double linear_x, double angular_z);  // Send velocity commands to SkidSteer

private:
    SkidSteer skidSteer;  // SkidSteer object
};

#endif // CONTROLLER_H
