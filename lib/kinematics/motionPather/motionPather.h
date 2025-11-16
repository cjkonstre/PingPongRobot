//handles pathing and communicaiton with motorcontroller
//would be cumbersome and wouldnt make a ton of sense to try to stream externally

//main functionality of this is to set target state (qs, dqs) and this would move to it in the background and handle comms with motorcontroller
//would need end condition, ie default behavior if it doesnt have an objective.
//could have a queue of objectives? not quite sure yet.


#pragma once

#include "kinematics/inverseK/inverseKin_traits.hpp"
#include "kinematics/SCComms/motorController_traits.hpp"

#include "ruckig/ruckig.hpp"
#include "array"

template <typename MC, typename KS, int DOFS>
    requires (MotorControllerType<MC> &&
              KinematicsSolverType<KS> &&
              MC::dofs == DOFS &&
              KS::dofs == DOFS)
class MotionPather {
private:
    MC& mc;
    KS& kin;

    ruckig::Ruckig<DOFS> otg;
    ruckig::OutputParameter<DOFS> output;

    std::array<double, DOFS> idlePos;

public:
    enum IDLE_CONDITION {GO_IDLEPOS, STOP};

    ruckig::InputParameter<DOFS> input;
    IDLE_CONDITION idle_condition;
    std::array<double, DOFS> idle_pos; //waiting position

    MotionPather(double control_cycle, const std::array<double, DOFS> max_vel, const std::array<double, DOFS> max_acc, const std::array<double, DOFS> max_jerk,
        const std::array<double, DOFS> idlePos, MC& mc, KS& kin);

    void setTarget(const std::array<double, DOFS>& target_qs, const std::array<double, DOFS>& target_dqs);

    void begin(); //starts and allows movement
    void stop();  //stops movements
};
