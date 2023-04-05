//
// Created by sham on 03.04.23.
//

#ifndef SRC_MECANUMDRIVECONTROLLER_H
#define SRC_MECANUMDRIVECONTROLLER_H

#include "mecanum_drive_kinematic.h"

#include <pluginlib/class_list_macros.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tfMessage.h>


namespace mecanum_drive_controller {
class MecanumDriveController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
public:
    MecanumDriveController();

    bool init(hardware_interface::VelocityJointInterface* hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh) override;

    void update(const ros::Time& time, const ros::Duration& period) override;

    void starting(const ros::Time& time) override;

    void stopping(const ros::Time& time) override;

private:
    std::string name_;
    ros::Subscriber sub_;

    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > pub_odom_;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > pub_odom_tf_;

    mecanum_drive_controller::MecanumDriveKinematic kinematic;

    realtime_tools::RealtimeBuffer<geometry_msgs::TwistStamped> cmd_rt_buffer;

    std::array<hardware_interface::JointHandle, BASEWHELLS> hi_wheel_;

    void cb_cmd_twist(const geometry_msgs::Twist& cmd);

    void set_pub_odom_fields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
};

PLUGINLIB_EXPORT_CLASS(mecanum_drive_controller::MecanumDriveController, controller_interface::ControllerBase)
}
#endif //SRC_MECANUMDRIVECONTROLLER_H
