//
// Created by sham on 03.04.23.
//

#ifndef SRC_MECANUM_DRIVE_KINEMATIC_H
#define SRC_MECANUM_DRIVE_KINEMATIC_H

#define BASEWHELLS ((int)(4))

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>


#include <vector>
#include <array>

namespace mecanum_drive_controller {

using Wheels = std::array<double, BASEWHELLS>;

class MecanumDriveKinematic {
public:
    MecanumDriveKinematic();

    void init(ros::NodeHandle& controller_nh);

    Wheels cartesian_vel_to_wheel_vel(const geometry_msgs::Twist& vel) const;

    geometry_msgs::Twist wheel_vel_to_cartesian_vel(const Wheels& vel) const;

    geometry_msgs::Pose2D wheel_pos_to_cartesian_pos(const Wheels& vel);

private:
    double wheel_radius_;
    double wheel_sep_coef_;

    bool last_wheel_pos_init_;
    Wheels last_wheel_pos;
    geometry_msgs::Pose2D curr_pos;
};


}

#endif //SRC_MECANUM_DRIVE_KINEMATIC_H
