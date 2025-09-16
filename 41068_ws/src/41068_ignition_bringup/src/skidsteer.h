#ifndef SKIDSTEER_H
#define SKIDSTEER_H

class SkidSteer {
public:
    SkidSteer();
    void move(double linear_x, double angular_z);  // Handle movement based on linear and angular velocity
};

#endif // SKIDSTEER_H
