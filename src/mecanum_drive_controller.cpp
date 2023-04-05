//
// Created by sham on 03.04.23.
//

#include "mecanum_drive_controller/mecanum_drive_controller.h"
namespace mecanum_drive_controller {

MecanumDriveController::MecanumDriveController()= default;

bool MecanumDriveController::init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &root_nh,
                                  ros::NodeHandle &controller_nh) {

    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of('/');
    name_ = complete_ns.substr(id + 1);

    std::vector<std::string> wheel_joint_names = {
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "back_left_wheel_joint",
            "back_right_wheel_joint"
    };
    for(int i = 0; i < wheel_joint_names.size(); ++i){
        std::string wheel_name;
        controller_nh.param(wheel_joint_names[i], wheel_name, wheel_name);
        ROS_INFO_STREAM_NAMED(name_, wheel_joint_names[i] << " use joint_handle[" << i << "] is " << wheel_name);
        hi_wheel_[i] = hw->getHandle(wheel_name);
    }


    kinematic.init(controller_nh);
    set_pub_odom_fields(root_nh, controller_nh);

    sub_ = controller_nh.subscribe("cmd_vel", 1, &MecanumDriveController::cb_cmd_twist, this);

    return true;
}


void MecanumDriveController::update(const ros::Time &time, const ros::Duration &period) {
    geometry_msgs::TwistStamped cmd_current = *(cmd_rt_buffer.readFromRT());

    const double dt = (time - cmd_current.header.stamp).toSec();
    if (dt > 0.25)
        cmd_current = geometry_msgs::TwistStamped();

    const auto wheel_vel = kinematic.cartesian_vel_to_wheel_vel(cmd_current.twist);

    //TODO add reverse param
    hi_wheel_[0].setCommand(wheel_vel[0]);
    hi_wheel_[1].setCommand(- wheel_vel[1]);
    hi_wheel_[2].setCommand(wheel_vel[2]);
    hi_wheel_[3].setCommand(- wheel_vel[3]);

    Wheels wheel_pos{
            hi_wheel_[0].getPosition(),
            hi_wheel_[1].getPosition(),
            hi_wheel_[2].getPosition(),
            hi_wheel_[3].getPosition(),
    };
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

void MecanumDriveController::set_pub_odom_fields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    // TODO set from configure
    double pose_cov_list[6]{0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 0.03};
    double twist_cov_list[6]{0.001, 0.001, 0.001, 1000000.0, 1000000.0, 0.03};
    pub_odom_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
    pub_odom_->msg_.child_frame_id = "base_frame_id_";
    pub_odom_->msg_.header.frame_id = "odom_frame_id_";
    pub_odom_->msg_.pose.covariance = {
            static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
            0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
            0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
            0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
            0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
            0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5]) };
    pub_odom_->msg_.twist.covariance = {
            static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
            0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
            0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
            0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
            0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
            0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5]) };


    pub_odom_tf_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    pub_odom_tf_->msg_.transforms.resize(1);
    pub_odom_tf_->msg_.transforms[0].child_frame_id = "base_frame_id_";
    pub_odom_tf_->msg_.transforms[0].header.frame_id = "odom_frame_id_";
}

}
