//
// Created by sham on 03.04.23.
//

#ifndef SRC_MECANUM_DRIVE_KINEMATIC_H
#define SRC_MECANUM_DRIVE_KINEMATIC_H

#include <vector>

namespace md {

struct Vector{
    explicit Vector(const double& longitudinal = 0, const double& transversal = 0, const double& angle = 0);
    double longitudinal;
    double transversal;
    double angle;

    inline std::vector<double> to_vector() const;
};

using Wheels = std::vector<double>;

class MecanumDriveKinematic {
    MecanumDriveKinematic(double wheel_radius, double wheel_separation_x, double wheel_separation_y);

    md::Wheels cartesian_to_wheels_velocities(md::Vector cartesian_velocities);

    md::Vector wheel_to_cartesian_velocities(md::Wheels wheels_velocities);

private:
    double wheel_radius_;
    double wheel_separation_x_;
    double wheel_separation_y_;
};

}

#endif //SRC_MECANUM_DRIVE_KINEMATIC_H
