//
// Created by sham on 03.04.23.
//

#include "mecanum_drive_controller/mecanum_drive_kinematic.h"
namespace mecanum_drive_controller {

MecanumDriveKinematic::MecanumDriveKinematic() :
        wheel_radius_(0),
        wheel_sep_coef_(0),
        last_wheel_pos_init_(false),
        last_wheel_pos({0})
{}


void MecanumDriveKinematic::init(ros::NodeHandle& controller_nh) {
    const std::string& complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of('/');
    std::string name_ = complete_ns.substr(id + 1);

    double wheel_radius = 0;
    double wheel_separation_x = 0;
    double wheel_separation_y  = 0;

    controller_nh.param("wheel_radius", wheel_radius, wheel_radius);
    controller_nh.param("wheel_separation_x", wheel_separation_x, wheel_separation_x);
    controller_nh.param("wheel_separation_y", wheel_separation_y, wheel_separation_y);

    ROS_INFO_STREAM_NAMED(name_, "Wheel radius set to " << wheel_radius);
    ROS_INFO_STREAM_NAMED(name_, "Wheel separation x set to " << wheel_separation_x);
    ROS_INFO_STREAM_NAMED(name_, "Wheel separation y set to " << wheel_separation_y);

    wheel_radius_ = wheel_radius,
            wheel_sep_coef_ = (wheel_separation_y + wheel_separation_x) / (2.0);
}


Wheels MecanumDriveKinematic::cartesian_vel_to_wheel_vel(const geometry_msgs::Twist& vel) const {
    double vx_per_R = vel.linear.x / wheel_radius_;
    double vy_per_R = vel.linear.y / wheel_radius_;
    double w_with_coef = wheel_sep_coef_ * wheel_radius_ * vel.angular.z ;

    Wheels wheels_velocity;
    wheels_velocity[0] = + vx_per_R + vy_per_R - w_with_coef;
    wheels_velocity[1] = - vx_per_R + vy_per_R + w_with_coef;
    wheels_velocity[2] = - vx_per_R + vy_per_R - w_with_coef;
    wheels_velocity[3] = + vx_per_R + vy_per_R + w_with_coef;

    return wheels_velocity;
}

geometry_msgs::Twist MecanumDriveKinematic::wheel_vel_to_cartesian_vel(const Wheels& vel) const{
    static double wheel_radius_per4 = wheel_radius_ / 4;

    geometry_msgs::Twist cartesian_vel;
    cartesian_vel.linear.x  = (-vel[0] + vel[1] - vel[2] + vel[3]) * wheel_radius_per4;
    cartesian_vel.linear.y  = (-vel[0] + vel[1] - vel[2] + vel[3]) * wheel_radius_per4;
    cartesian_vel.angular.z = (-vel[0] + vel[1] - vel[2] + vel[3]) * (wheel_radius_per4 / wheel_sep_coef_);

    return cartesian_vel;
}

geometry_msgs::Pose2D MecanumDriveKinematic::wheel_pos_to_cartesian_pos(const Wheels &pos) {
    if (!last_wheel_pos_init_) {
        last_wheel_pos = pos;
        curr_pos = geometry_msgs::Pose2D();
        last_wheel_pos_init_ = true;
        return curr_pos;
    }

    Wheels delta_pos;
    delta_pos[0] = pos[0] - last_wheel_pos[0];
    delta_pos[1] = pos[1] - last_wheel_pos[1];
    delta_pos[2] = pos[2] - last_wheel_pos[2];
    delta_pos[3] = pos[3] - last_wheel_pos[3];
    last_wheel_pos = pos;

    static double wheel_radius_per4 = wheel_radius_ / 4;
    double delta_pos_x = (-delta_pos[0] + delta_pos[1] - delta_pos[2] + delta_pos[3]) * wheel_radius_per4;
    double delta_pos_y = ( delta_pos[0] + delta_pos[1] - delta_pos[2] - delta_pos[3]) * wheel_radius_per4;
    curr_pos.theta += (delta_pos[0] + delta_pos[1] + delta_pos[2] + delta_pos[3]) * (wheel_radius_per4/wheel_sep_coef_);

    const double cos_theta = cos(curr_pos.theta);
    const double sin_theta = sin(curr_pos.theta);

    curr_pos.x += delta_pos_x * cos_theta - delta_pos_x * sin_theta;
    curr_pos.y += delta_pos_y * cos_theta + delta_pos_y * sin_theta;

    return curr_pos;
}

}
