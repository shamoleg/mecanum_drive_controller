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


}
