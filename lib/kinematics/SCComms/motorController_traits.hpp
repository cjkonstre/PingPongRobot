//look @ inverse kins traits hpp for explanation

#pragma once

#include <type_traits>
#include "kinematics/SCComms/motorController.hpp"

template<typename T>
struct is_motorController : std::false_type {};

template<int D, typename P>
struct is_motorController<MotorController<D, P>> : std::true_type {};

template<typename T>
concept MotorControllerType = is_motorController<T>::value;