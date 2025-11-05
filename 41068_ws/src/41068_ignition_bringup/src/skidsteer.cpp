#include "skidsteer.h"
#include <iostream>  // For debugging (you can remove this later)

SkidSteer::SkidSteer() {
    // Initialize any parameters if needed
}

void SkidSteer::move(double linear_x, double angular_z) {
    // Calculate left and right wheel speeds based on skidsteer model
    double left_speed = linear_x - angular_z;  // Left wheel speed
    double right_speed = linear_x + angular_z; // Right wheel speed

    // Output the speeds (for debugging)
    std::cout << "Left Speed: " << left_speed << ", Right Speed: " << right_speed << std::endl;

    // Send these speeds to actuators (publish to /cmd_vel or control robot hardware)
}