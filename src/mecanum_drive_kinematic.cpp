//
// Created by sham on 03.04.23.
//

#include "mecanum_drive_controller/mecanum_drive_kinematic.h"
namespace mecanum_drive_controller {

MecanumDriveKinematic::MecanumDriveKinematic()  {}


void MecanumDriveKinematic::init(ros::NodeHandle& controller_nh) {
    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of('/');
    std::string name_ = complete_ns.substr(id + 1);

    double wheel_radius = 0;
    double wheel_separation_x = 0;
    double wheel_separation_y  = 0;

    controller_nh.param("wheel radius", wheel_radius, wheel_radius);
    controller_nh.param("wheel separation_x", wheel_separation_x, wheel_separation_x);
    controller_nh.param("wheel separation_y", wheel_separation_y, wheel_separation_y);

    ROS_INFO_STREAM_NAMED(name_, "Wheel radius set to " << wheel_radius);
    ROS_INFO_STREAM_NAMED(name_, "Wheel separation x set to " << wheel_separation_x);
    ROS_INFO_STREAM_NAMED(name_, "Wheel separation y set to " << wheel_separation_y);

    wheel_radius_ = wheel_radius,
    wheel_separation_coef_ = wheel_separation_y + wheel_separation_x;
}


Wheels MecanumDriveKinematic::cartesian_to_wheels_velocities(const geometry_msgs::Twist& vel) const {
    double vx_per_R = vel.linear.x / wheel_radius_;
    double vy_per_R = vel.linear.y / wheel_radius_;
    double w_with_coef  = ((wheel_separation_coef_) / wheel_radius_) * vel.angular.z;

    Wheels wheels_velocity;
    wheels_velocity[0] = + vx_per_R + vy_per_R - w_with_coef;
    wheels_velocity[1] = - vx_per_R + vy_per_R + w_with_coef;
    wheels_velocity[2] = - vx_per_R + vy_per_R - w_with_coef;
    wheels_velocity[3] = + vx_per_R + vy_per_R + w_with_coef;

    return wheels_velocity;
}

geometry_msgs::Twist MecanumDriveKinematic::wheel_to_cartesian_velocities(const Wheels& vel) const{
    static double wheel_radius_per4 = wheel_radius_ / 4;

    geometry_msgs::Twist cartesian_vel;
    cartesian_vel.linear.x  = (-vel[0] + vel[1] - vel[2] + vel[3]) * wheel_radius_per4;
    cartesian_vel.linear.y  = (-vel[0] + vel[1] - vel[2] + vel[3]) * wheel_radius_per4;
    cartesian_vel.angular.z = (-vel[0] + vel[1] - vel[2] + vel[3]) * (wheel_radius_per4 / wheel_separation_coef_);

    return cartesian_vel;
}

}
