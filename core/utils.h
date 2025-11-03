#pragma once

#include "core/config.h"
#include "kinematics/inverseK/inverseKin.hpp"
#include "core/measurements/dimensions.h"

KinematicsSolver<DOFS> make_kinSolver() {return KinematicsSolver<DOFS>(frame_pulleyPoss, paddle_anchorOffsets, pulley_anchorOffsets_RefOri);}
???
KinematicsSolver<DOFS> kin(frame_pulleyPoss, paddle_anchorOffsets, pulley_anchorOffsets_RefOri);
KinematicsSolver<DOFS> kin = make_kinSolver();