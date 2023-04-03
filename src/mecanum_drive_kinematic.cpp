//
// Created by sham on 03.04.23.
//

#include "mecanum_drive_kinematic.h"
namespace md {

Vector::Vector(const double& longitudinal, const double& transversal, const double& angle) :
        longitudinal(longitudinal),
        transversal(transversal),
        angle(angle)
        {}

std::vector<double> Vector::to_vector() const {
    return {longitudinal, transversal, angle};
}


MecanumDriveKinematic::MecanumDriveKinematic(double wheel_radius, double wheel_separation_x, double wheel_separation_y) :
        wheel_radius_(wheel_radius),
        wheel_separation_x_(wheel_separation_x),
        wheel_separation_y_(wheel_separation_y)
        {}

md::Wheels MecanumDriveKinematic::cartesian_to_wheels_velocities(md::Vector cartesian_velocities) const {
    double vx = cartesian_velocities.longitudinal / wheel_radius_;
    double vy = cartesian_velocities.transversal /wheel_radius_;
    double w  = ((wheel_separation_x_ + wheel_separation_y_) / wheel_radius_) * cartesian_velocities.angle;

    md::Wheels wheels_velocity(4);
    wheels_velocity[0] = +vx + vy - w;
    wheels_velocity[1] = -vx + vy + w;
    wheels_velocity[2] = -vx + vy - w;
    wheels_velocity[3] = +vx + vy + w;

    return wheels_velocity;
}

md::Vector MecanumDriveKinematic::wheel_to_cartesian_velocities(md::Wheels wheels_velocities) {
    return md::Vector();
}

}
