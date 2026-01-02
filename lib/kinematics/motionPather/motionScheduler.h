#pragma once

#include <vector>
#include <array>
#include "kinematics/Pose.h"

//class that handles motion scheduling. not much use aside from helping the motionpather
class MotionScheduler {
public:
    struct Frame {
        Pose spat;
        Pose vels;
        double minDur = 0;
    };

private:
    std::vector<Frame> schedule;
    size_t index = 0;

public:
    bool loop = false;

    void reset() { index = 0; }

    bool isFinished() const {
        return schedule.empty() || index >= schedule.size();
    }

    void inc() {
        if (isFinished()) return;
        ++index;
        if (loop && index >= schedule.size()) {
            index = 0;
        }
    }

    const Frame& at() const {
        // optional assert
        //assert(!isFinished());
        return schedule[index];
    }

    void add(const Frame& pose) {
        schedule.push_back(pose);
    }
};
