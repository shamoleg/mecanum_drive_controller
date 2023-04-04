//
// Created by sham on 03.04.23.
//

#ifndef SRC_MECANUM_DRIVE_KINEMATIC_H
#define SRC_MECANUM_DRIVE_KINEMATIC_H

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <array>

namespace mecanum_drive_controller {

class MecanumDriveKinematic {
public:
    inline explicit MecanumDriveKinematic();

    void init(double wheel_radius, double wheel_separation_y, double wheel_separation_x);

    std_msgs::Float64MultiArray cartesian_to_wheels_velocities(const geometry_msgs::Twist& vel) const;

    geometry_msgs::Twist wheel_to_cartesian_velocities(const std_msgs::Float64MultiArray& vel) const;

private:
    double wheel_radius_;
    double wheel_separation_x_;
    double wheel_separation_y_;
};


}

#endif //SRC_MECANUM_DRIVE_KINEMATIC_H
