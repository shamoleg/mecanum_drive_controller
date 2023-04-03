//
// Created by sham on 03.04.23.
//

#ifndef SRC_MECANUMDRIVECONTROLLER_H
#define SRC_MECANUMDRIVECONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

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

    std::vector<hardware_interface::JointHandle> jointHandle_;

};

}
#endif //SRC_MECANUMDRIVECONTROLLER_H
