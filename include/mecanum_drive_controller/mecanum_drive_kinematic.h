//
// Created by sham on 03.04.23.
//

#ifndef SRC_MECANUM_DRIVE_KINEMATIC_H
#define SRC_MECANUM_DRIVE_KINEMATIC_H

#define BASEWHELLS ((int)(4))

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>


#include <vector>
#include <array>

namespace mecanum_drive_controller {

using Wheels = std::array<double, BASEWHELLS>;

class MecanumDriveKinematic {
public:
    MecanumDriveKinematic();

    void init(ros::NodeHandle& controller_nh);

    Wheels cartesian_to_wheels_velocities(const geometry_msgs::Twist& vel) const;

    geometry_msgs::Twist wheel_to_cartesian_velocities(const Wheels& vel) const;

private:
    double wheel_radius_;
    double wheel_separation_coef_;
};


}

#endif //SRC_MECANUM_DRIVE_KINEMATIC_H
