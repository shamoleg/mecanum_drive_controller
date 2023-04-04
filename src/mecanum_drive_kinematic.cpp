//
// Created by sham on 03.04.23.
//

#include "mecanum_drive_controller/mecanum_drive_kinematic.h"
namespace mecanum_drive_controller {

MecanumDriveKinematic::MecanumDriveKinematic()  {}


void MecanumDriveKinematic::init(double wheel_radius, double wheel_separation_y, double wheel_separation_x) {
    wheel_radius_ = wheel_radius,
    wheel_separation_x_ = wheel_separation_y,
    wheel_separation_y_ = wheel_separation_x;
}


std_msgs::Float64MultiArray MecanumDriveKinematic::cartesian_to_wheels_velocities(const geometry_msgs::Twist& vel) const {
    double vx = vel.linear.x / wheel_radius_;
    double vy = vel.linear.y / wheel_radius_;
    double w  = ((wheel_separation_x_ + wheel_separation_y_) / wheel_radius_) * vel.angular.z;

    std_msgs::Float64MultiArray wheels_velocity;
    wheels_velocity.data[0] = +vx + vy - w;
    wheels_velocity.data[1] = -vx + vy + w;
    wheels_velocity.data[2] = -vx + vy - w;
    wheels_velocity.data[3] = +vx + vy + w;

    return wheels_velocity;
}

geometry_msgs::Twist MecanumDriveKinematic::wheel_to_cartesian_velocities(const std_msgs::Float64MultiArray& vel) const{
    static double wheel_radius_per4 = wheel_radius_ / 4;

    geometry_msgs::Twist cartesian_vel;
    cartesian_vel.linear.x  = (-vel.data[0] + vel.data[1] - vel.data[2] + vel.data[3]) / wheel_radius_per4;
    cartesian_vel.linear.y  = (-vel.data[0] + vel.data[1] - vel.data[2] + vel.data[3]) / wheel_radius_per4;
    cartesian_vel.angular.z = (-vel.data[0] + vel.data[1] - vel.data[2] + vel.data[3]) / wheel_radius_per4;

    return cartesian_vel;
}

}
