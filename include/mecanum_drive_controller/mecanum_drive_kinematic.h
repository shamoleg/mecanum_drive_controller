//
// Created by sham on 03.04.23.
//

#ifndef SRC_MECANUM_DRIVE_KINEMATIC_H
#define SRC_MECANUM_DRIVE_KINEMATIC_H

#define BASEWHELLS ((int)(4))

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



#include <vector>
#include <array>

namespace mecanum_drive_controller {

using Wheels = std::array<double, BASEWHELLS>;

class MecanumDriveKinematic {
public:
    MecanumDriveKinematic();

    void init(ros::NodeHandle& controller_nh);

    Wheels get_wheels_vel(const geometry_msgs::Twist& vel) const;

    geometry_msgs::Twist get_cartesian_vel(const Wheels& vel) const;

    geometry_msgs::Pose2D get_cartesian_pose2d(const Wheels& pos);

    geometry_msgs::Pose get_cartesian_pose(const Wheels& vel);

private:
    double wheel_radius_;
    double wheel_sep_coef_;

    bool last_wheel_pos_init_;
    Wheels last_wheel_pos;
    geometry_msgs::Pose2D curr_pos;
};


}

#endif //SRC_MECANUM_DRIVE_KINEMATIC_H
