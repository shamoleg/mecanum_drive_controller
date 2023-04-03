//
// Created by sham on 03.04.23.
//

#include "mecanum_drive_controller.h"
namespace md {

MecanumDriveController::MecanumDriveController() {

}

bool MecanumDriveController::init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &root_nh,
                                  ros::NodeHandle &controller_nh) {
    return Controller::init(hw, root_nh, controller_nh);
}

void MecanumDriveController::update(const ros::Time &time, const ros::Duration &period) {

}

void MecanumDriveController::starting(const ros::Time &time) {
    ControllerBase::starting(time);
}

void MecanumDriveController::stopping(const ros::Time &time) {
    ControllerBase::stopping(time);
}

}
