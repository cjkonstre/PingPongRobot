//hello gg

#pragma once

#include <Eigen/Dense>
#include <array>
#include <utility>
#include <kinematics/kinUtil.h>

template <int cableDOFS>
class KinematicsSolver{

    static Eigen::Matrix3d _HatOperator3d(Eigen::Vector3d t); //returns skew-symmetric cross-product matrix of t

    Eigen::Vector3d objNormalRef; //what the anchoroffsets are relative to

    Eigen::Matrix<double, 3, cableDOFS> pulleyPoints; //spatial vectors from origin to the origin of the cable, ie the pulleyhead
    Eigen::Matrix<double, 3, cableDOFS> anchorOffsets; //spatial vectors from object origin to pulley anchors, when object normal is aligned with the Z direction (facing up)


public:
    KinematicsSolver() = default;
    KinematicsSolver(
        const std::array<std::array<double, 3>, cableDOFS>& pulleyPointsIn,
        const std::array<std::array<double, 3>, cableDOFS>& anchorOffsetsIn,
        const std::array<double, 3>& refOrientation);

    Eigen::Vector3d get_objNormalRef() {return objNormalRef;}

    //gets the positions of the obj anchors with reference to origin, same frame as pulleypoints
    Eigen::Matrix<double, 3, cableDOFS> getAnchorPoints(Eigen::Vector3d objPos, Eigen::Vector3d objnormal);
    Eigen::RowVector<double, cableDOFS> getCableLens(const Eigen::Matrix<double, 3, cableDOFS>& anchorPoints);


    Eigen::RowVector<double, cableDOFS> getCableVels(Eigen::Matrix<double, 3, cableDOFS> anchorPoints, Eigen::Vector3d objVel);
    
    //special bc it does the post procesing of subtracting the cablezeros off
    MotionStateD<cableDOFS> doIK(Eigen::Vector3d objPos, Eigen::Vector3d objnormal, Eigen::Vector3d objVel);
    MotionStateD<cableDOFS> doIK(std::initializer_list<double> objPos,std::initializer_list<double> objNormal,std::initializer_list<double> objVel);
};

#include "kinematics/inverseK/inverseKin.tpp"