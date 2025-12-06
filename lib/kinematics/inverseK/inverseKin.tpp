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
    Eigen::Vector3d objPos, Eigen::Vector3d objnormal) const
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
Eigen::Matrix<double, 1, cableDOFS>
KinematicsSolver<cableDOFS>::getCableLens(
    const Eigen::Matrix<double, 3, cableDOFS>& anchorPoints) const
{
    return (pulleyPoints - anchorPoints).colwise().norm();
}

template <int cableDOFS>
Eigen::Matrix<double, 1, cableDOFS>
KinematicsSolver<cableDOFS>::getCableVels(
    const Eigen::Matrix<double, 3, cableDOFS>& anchorPoints,
    const Eigen::Vector3d& objPos,
    const Eigen::Vector3d& objNorm,
    const Eigen::Vector3d& objVel,
    const Eigen::Vector3d& objNormVel) const
{
    // Unit cable direction vectors (from pulley to anchor)
    Eigen::Matrix<double, 3, cableDOFS> IcablePointingVecs = 
        (anchorPoints - pulleyPoints).colwise().normalized();

    Eigen::Vector3d omegaVec = //assumes objNormVel is perp to objNorm. can switch to spherical coords in future maybe
        objNorm.normalized().cross(objNormVel);

    Eigen::Matrix<double, 3, cableDOFS> vAnchors = //rotates the nonrotated anchor offsets
        (_HatOperator3d(omegaVec) * (anchorPoints.colwise()-objPos)).colwise() + objVel;

    Eigen::Matrix<double, 1, cableDOFS> cableVels =
        (vAnchors.array() * IcablePointingVecs.array()).colwise().sum();

    return cableVels;
}

template <int cableDOFS>
MotionStateD<cableDOFS>
KinematicsSolver<cableDOFS>::doIK(
    Eigen::Vector3d objPos, Eigen::Vector3d objnormal, Eigen::Vector3d objVel, Eigen::Vector3d objnormVel) const
{
    Eigen::Matrix<double, 3, cableDOFS> anchorpoints = getAnchorPoints(objPos, objnormal);
    Eigen::Matrix<double, 1, cableDOFS> cablelens = getCableLens(anchorpoints);
    Eigen::Matrix<double, 1, cableDOFS> cablevels = getCableVels(anchorpoints, objPos, objnormal, objVel, objnormVel);

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
    std::initializer_list<double> objVel,
    std::initializer_list<double> objnormVel) const
{
    assert(objPos.size() == 3 && objNormal.size() == 3 && objVel.size() == 3);

    Eigen::Vector3d pos((*objPos.begin()), (*(objPos.begin() + 1)), (*(objPos.begin() + 2)));
    Eigen::Vector3d normal((*objNormal.begin()), (*(objNormal.begin() + 1)), (*(objNormal.begin() + 2)));
    Eigen::Vector3d vel((*objVel.begin()), (*(objVel.begin() + 1)), (*(objVel.begin() + 2)));
    Eigen::Vector3d normvel((*objnormVel.begin()), (*(objnormVel.begin() + 1)), (*(objnormVel.begin() + 2)));

    return doIK(pos, normal, vel, normvel); // delegate to main implementation
}

template <int cableDOFS>
MotionStateD<cableDOFS>  
KinematicsSolver<cableDOFS>::doIK(
    const std::array<double,3>& objPos,
    const std::array<double,3>& objNormal,
    const std::array<double,3>& objVel,
    const std::array<double,3>& objnormVel) const
{
    Eigen::Vector3d pos(objPos[0], objPos[1], objPos[2]);
    Eigen::Vector3d normal(objNormal[0], objNormal[1], objNormal[2]);
    Eigen::Vector3d vel(objVel[0], objVel[1], objVel[2]);
    Eigen::Vector3d normvel(objnormVel[0], objnormVel[1], objnormVel[2]);

    return doIK(pos, normal, vel, normvel);
}