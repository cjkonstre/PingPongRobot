//so that we can use concepts to check for kinematics solvers
//ie can write T func(KinematicsSolverType auto& kin), saying that kin has to be a kinematics solver but dont need to 
//template the whole function/class

#pragma once

#include <type_traits>
#include "kinematics/inverseK/inverseKin.hpp"

template<typename T>
struct is_kinSolver : std::false_type {};

template<int D>
struct is_kinSolver<KinematicsSolver<D>> : std::true_type {};

template<typename T>
concept KinematicsSolverType = is_kinSolver<T>::value;