//
// Created by sham on 03.04.23.
//

#include "mecanum_drive_controller/mecanum_drive_controller.h"
namespace mecanum_drive_controller {

MecanumDriveController::MecanumDriveController(){};

bool MecanumDriveController::init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &root_nh,
                                  ros::NodeHandle &controller_nh) {
    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of('/');
    name_ = complete_ns.substr(id + 1);

    std:std::vector<std::string> wheel_name;
    controller_nh.param("joints", wheel_name, wheel_name);
    for(int i = 0; i < wheel_name.size(); ++i){
        ROS_INFO_STREAM_NAMED(name_, wheel_name[i] << " use joint_handle[" << i << "] is ");
        hi_wheel_[i] = hw->getHandle(wheel_name[i]);
    }


    kinematic.init(0.03, 0.3 ,0.3);

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

    const auto wheel_vel = kinematic.cartesian_to_wheels_velocities(cmd_current.twist);

    hi_wheel_[0].setCommand(wheel_vel[0]);
    hi_wheel_[1].setCommand(wheel_vel[0]);
    hi_wheel_[2].setCommand(wheel_vel[0]);
    hi_wheel_[3].setCommand(wheel_vel[0]);

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
