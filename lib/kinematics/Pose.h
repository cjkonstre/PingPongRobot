#pragma once

#include <array>
#include <cmath>

//ori is spherical rather than normal
struct Pose {
    struct Orientation {
        double theta;
        double phi;

        std::array<double, 3> n() const {
            double sin_theta = sin(theta);
            double cos_theta = cos(theta);
            double sin_phi = sin(phi);
            double cos_phi = cos(phi);

            std::array<double, 3> normal = {sin_theta*cos_phi, 
                                            cos_theta*cos_phi, 
                                            sin_phi};

            return normal;
        }

        //make sure norm is actually normalized
        inline static Orientation from_normal(const auto& norm) {return Orientation{atan2(norm[0], norm[1]), asin(norm[2])};}
    };

    std::array<double, 3> pos;
    Orientation ori; //theta, phi. (0,0) is (1,0,0).

    inline std::array<double, 5> to5vec() const {return {pos[0], pos[1], pos[2], ori.theta, ori.phi};}
};