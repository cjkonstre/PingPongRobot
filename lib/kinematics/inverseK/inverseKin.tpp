#include <kinematics/inverseK/inverseKin.hpp>

template <int cableDOFS>
Eigen::Matrix3d 
KinematicsSolver<cableDOFS>::_HatOperator3d(
    Eigen::Vector3d t)
{
    Eigen::Matrix3d t_hat;
    t_hat << 0, -t(2), t(1),
        t(2), 0, -t(0),
        -t(1), t(0), 0;
    return t_hat;
}

template <int cableDOFS> 
KinematicsSolver<cableDOFS>::KinematicsSolver(
    const std::array<std::array<double, 3>, cableDOFS>& pulleyPointsIn,
    const std::array<std::array<double, 3>, cableDOFS>& anchorOffsetsIn,
    const std::array<double, 3>& refOrientation):
    objNormalRef(refOrientation.data()) {
    for (int i = 0; i < cableDOFS; ++i) {
        pulleyPoints.col(i) = Eigen::Vector3d(
            pulleyPointsIn[i][0],
            pulleyPointsIn[i][1],
            pulleyPointsIn[i][2]
        );
        anchorOffsets.col(i) = Eigen::Vector3d(
            anchorOffsetsIn[i][0],
            anchorOffsetsIn[i][1],
            anchorOffsetsIn[i][2]
        );
    }
}

template <int cableDOFS>
Eigen::Matrix<double, 3, cableDOFS> 
KinematicsSolver<cableDOFS>::getAnchorPoints(
    Eigen::Vector3d objPos, Eigen::Vector3d objnormal) 
{
    Eigen::Vector3d a = objNormalRef.normalized();
    Eigen::Vector3d b = objnormal.normalized();

    Eigen::Vector3d v = a.cross(b);
    double s = v.norm();
    double c = a.dot(b);

    Eigen::Matrix3d vhat = _HatOperator3d(v);
    Eigen::Matrix3d R;

    if (s < 1e-8) {
        if (c > 0) {
            R = Eigen::Matrix3d::Identity();
        } else {
            Eigen::Vector3d axis = a.unitOrthogonal();
            Eigen::Matrix3d axis_hat = _HatOperator3d(axis);
            R = Eigen::Matrix3d::Identity() + 2 * axis_hat * axis_hat;
        }
    } else {
        R = Eigen::Matrix3d::Identity() + vhat + vhat * vhat * ((1 - c) / (s * s));
    }

    // Rotate anchor offsets and translate
    Eigen::Matrix<double, 3, cableDOFS> anchorpointsprime =
        (R * anchorOffsets).colwise() + objPos;

    return anchorpointsprime;
}

template <int cableDOFS>
Eigen::RowVector<double, cableDOFS>
KinematicsSolver<cableDOFS>::getCableLens(
    const Eigen::Matrix<double, 3, cableDOFS>& anchorPoints)
{
    return (pulleyPoints - anchorPoints).colwise().norm();
}

//since the object will never be actively rotaitng the anchorpoint velocity is jus tthe object velocity
template <int cableDOFS>
Eigen::RowVector<double, cableDOFS>
KinematicsSolver<cableDOFS>::getCableVels(
    Eigen::Matrix<double, 3, cableDOFS> anchorPoints, Eigen::Vector3d objVel)
{
    //cable pointing vectors.
    Eigen::Matrix<double, 3, cableDOFS> IcablePointingVecs = (anchorPoints - pulleyPoints).colwise().normalized();
    //dot the pointing vectors with direction of motion
    return IcablePointingVecs.transpose()*objVel;
}

template <int cableDOFS>
MotionStateD<cableDOFS>
KinematicsSolver<cableDOFS>::doIK(
    Eigen::Vector3d objPos, Eigen::Vector3d objnormal, Eigen::Vector3d objVel) 
{
    Eigen::Matrix<double, 3, cableDOFS> anchorpoints = getAnchorPoints(objPos, objnormal);
    Eigen::RowVector<double, cableDOFS> cablelens = getCableLens(anchorpoints);
    Eigen::RowVector<double, cableDOFS> cablevels = getCableVels(anchorpoints, objVel);

    std::array<double, cableDOFS> cablelens_vec, cablevels_vec;
    std::copy(cablelens.data(), cablelens.data() + cablelens.size(), cablelens_vec.begin());
    std::copy(cablevels.data(), cablevels.data() + cablevels.size(), cablevels_vec.begin());

    return MotionStateD<cableDOFS>(cablelens_vec, cablevels_vec);
}

template <int cableDOFS>
MotionStateD<cableDOFS> 
KinematicsSolver<cableDOFS>::doIK(
    std::initializer_list<double> objPos,
    std::initializer_list<double> objNormal,
    std::initializer_list<double> objVel)
{
    assert(objPos.size() == 3 && objNormal.size() == 3 && objVel.size() == 3);

    Eigen::Vector3d pos((*objPos.begin()), (*(objPos.begin() + 1)), (*(objPos.begin() + 2)));
    Eigen::Vector3d normal((*objNormal.begin()), (*(objNormal.begin() + 1)), (*(objNormal.begin() + 2)));
    Eigen::Vector3d vel((*objVel.begin()), (*(objVel.begin() + 1)), (*(objVel.begin() + 2)));

    return doIK(pos, normal, vel); // delegate to main implementation
}