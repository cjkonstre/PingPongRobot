/*
the purpose of this was to test dynamic torque optimal time pathing
basically updating the maximum accelerations based on the torque profile on the motors to hopefully
 squeeze a little more accelerations out from lower speeds

doesnt really work though in practice, its solving a different problem. it may be fast, but it might not be faster
in practice, setting a static bound on acceleration will work the best. 
if accels are really an issue, up the pulley diameter to
 get those faster accels out from lower speeds, and set a speed cap so the speed never gets above what the
 required accelerations can handle
*/

#include "matplotlibcpp.h"
#include <ruckig/ruckig.hpp>
#include <vector>

namespace plt = matplotlibcpp;
using namespace ruckig;
 
double torqueProfile(double speed){
    double n = -fabs(speed)/2+4;
    return n<0.1?0.1:n;
}

int main() {
    // Create instances: the Ruckig trajectory generator as well as input and output parameters
    Ruckig<2> ruckig(0.01);  // control cycle
    InputParameter<2> input;
    OutputParameter<2> output;
 
    // Set input parameters
    input.current_position = {3, -2};
    input.current_velocity = {0.0, 2.2};
    input.current_acceleration = {0.0, 2.5};
 
    input.target_position = {5.0, -2.0};
    input.target_velocity = {0.0, 0.5};
    input.target_acceleration = {0.0, 0.0};
 
    input.max_velocity = {30.0, 30.0};
    input.max_acceleration = {10.0, 10.0};
    input.max_jerk = {4.0, 3.0};
 
    // Generate the trajectory within the control loop
    //std::cout << "t | position" << std::endl;
    using vec2=std::array<double, 2>;
    std::vector<vec2> poslist;
    std::vector<vec2> ogposlist;
    std::vector<std::vector<vec2>> posslist;


    
    while (ruckig.update(input, output) == Result::Working) {
        //std::cout << output.time << " | " << output.new_position[0] << std::endl;
        output.pass_to_input(input);
        input.max_acceleration = {torqueProfile(output.new_velocity[0]), 
                                  torqueProfile(output.new_position[1])};
        
        auto thispath_in=input;
        auto thispath_out=output;
        //simulate forward over this path with fixed values
        poslist.clear();
        while (ruckig.update(thispath_in, thispath_out) == Result::Working) {
            thispath_out.pass_to_input(thispath_in);
            std::array<double, 2> pos = {thispath_out.new_position[0], thispath_out.new_position[1]};
            poslist.push_back(pos);
        }
        posslist.push_back(poslist);

        std::array<double, 2> pos = {output.new_position[0], output.new_position[1]};
        ogposlist.push_back(pos);
    }
 
    std::cout << "Trajectory duration: " << output.trajectory.get_duration() << " [s]." << std::endl;


    int pathi = 0;
    for (auto pth : posslist) {
        pathi++;
        if (pathi%10 != 0) continue;
        std::vector<double> xs;
        std::vector<double> ys;
        for (auto pos: pth){
            xs.push_back(pos[0]);
            ys.push_back(pos[1]);
        }
        std::cout<<pathi << "/" << posslist.size() << "\n";
        plt::plot(xs, ys);
    }

    std::vector<double> xs;
    std::vector<double> ys;
    for (auto pos: ogposlist){
        xs.push_back(pos[0]);
        ys.push_back(pos[1]);
    }
    plt::plot(xs, ys);
    plt::show();
}