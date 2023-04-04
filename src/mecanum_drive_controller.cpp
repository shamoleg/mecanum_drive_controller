//
// Created by sham on 03.04.23.
//

#include "mecanum_drive_controller/mecanum_drive_controller.h"
namespace mecanum_drive_controller {

bool MecanumDriveController::init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &root_nh,
                                  ros::NodeHandle &controller_nh) {
    std::cout << "+++++++++++++++++++++++++++++" << std::endl
            << "+++++++++++++++++++++++++++++" << std::endl
            << "+++++++++++++++++++++++++++++" << std::endl
            << "+++++++++++++++++++++++++++++" << std::endl
            << "+++++++++++++++++++++++++++++" << std::endl
            << "+++++++++++++++++++++++++++++" << std::endl
            ;
    kinematic.init(0.03, 0.3 ,0.3);

    hi_wheel_[0] = hw->getHandle("wheel_joint_fl");
    hi_wheel_[1] = hw->getHandle("wheel_joint_fr");
    hi_wheel_[2] = hw->getHandle("wheel_joint_bl");
    hi_wheel_[3] = hw->getHandle("wheel_joint_br");

    sub_ = controller_nh.subscribe("cmd_vel", 1, &MecanumDriveController::cb_cmd_twist, this);

    return true;
}


void MecanumDriveController::update(const ros::Time &time, const ros::Duration &period) {
    auto cmd_current = *(cmd_rt_buffer.readFromRT());

    const double dt = (time - cmd_current.header.stamp).toSec();
    if (dt > 0.25){
        cmd_current.twist.linear.x = 0.0;
        cmd_current.twist.linear.y = 0.0;
        cmd_current.twist.angular.z = 0.0;
    }

    std_msgs::Float64MultiArray wheel_vel(kinematic.cartesian_to_wheels_velocities(cmd_current.twist));

    hi_wheel_[0].setCommand(wheel_vel.data[0]);
    hi_wheel_[1].setCommand(wheel_vel.data[1]);
    hi_wheel_[2].setCommand(wheel_vel.data[2]);
    hi_wheel_[3].setCommand(wheel_vel.data[3]);

}


void MecanumDriveController::starting(const ros::Time &time) {
    std::cout << "starting";
}


void MecanumDriveController::stopping(const ros::Time &time) {
    std::cout << "stopping";
}

void MecanumDriveController::cb_cmd_twist(const geometry_msgs::Twist &cmd) {
    geometry_msgs::TwistStamped cmd_stamped;
    cmd_stamped.twist = cmd;
    cmd_stamped.header.stamp = ros::Time::now();
    cmd_rt_buffer.writeFromNonRT(cmd_stamped);
}

}
