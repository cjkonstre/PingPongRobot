#include "vision/informationSystem/informationSystem.h"

template <int N>
InformationSystem<N>::InformationSystem(
    const std::array<const Camera*, N>& cams,
    const std::array<double, N>& sensorNoisesPx
) : sensors(cams), sigmaPx(sensorNoisesPx) {}

template <int N>
InformationSystem<N>::InformationSystem(
    std::initializer_list<const Camera*> cams,
    std::initializer_list<double> sensorNoisesPx
) {
    int i = 0; for (const Camera* cam : cams) sensors[i++] = cam;
    i = 0; for (double s : sensorNoisesPx) sigmaPx[i++] = s;
}

template <int N>
GaussBlob<3> InformationSystem<N>::combineMeasurements(
    std::initializer_list<cv::Point2f> centersList,
    std::initializer_list<bool> retsList
) {
    std::array<cv::Point2f, N> centers;
    std::array<bool, N> rets;

    int i = 0; for (const auto& p : centersList) centers[i++] = p;
    i = 0; for (bool r : retsList) rets[i++] = r;

    return combineMeasurements(centers, rets);
}
template <int N>
GaussBlob<3> InformationSystem<N>::combineMeasurements(
    const std::array<cv::Point2f, N>& centers,
    const std::array<bool, N>& rets
) {
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();

    std::array<Eigen::Vector3d, N> Cs;
    std::array<Eigen::Vector3d, N> ds;

    int used = 0;

    for (int i = 0; i < N; ++i) {
        if (!rets[i]) continue;

        const Camera& cam = *sensors[i];

        cv::Mat K, D, Rcv, tcv;
        cam.K.convertTo(K, CV_64F);
        cam.D.convertTo(D, CV_64F);
        cam.R.convertTo(Rcv, CV_64F);
        cam.t.convertTo(tcv, CV_64F);

        Eigen::Matrix3d R;
        Eigen::Vector3d t;

        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                R(r, c) = Rcv.at<double>(r, c);
            }

            t(r) = tcv.at<double>(r);
        }

        std::vector<cv::Point2f> src = { centers[i] };
        std::vector<cv::Point2f> undist;

        if (cam.isFisheye) {
            cv::fisheye::undistortPoints(src, undist, K, D);
        } else {
            cv::undistortPoints(src, undist, K, D);
        }

        Eigen::Vector3d rayCam;
        rayCam << undist[0].x, undist[0].y, 1.0;
        rayCam.normalize();

        Eigen::Vector3d C = -R.transpose() * t;

        Eigen::Vector3d d = R.transpose() * rayCam;
        d.normalize();

        Cs[i] = C;
        ds[i] = d;

        Eigen::Matrix3d P =
            Eigen::Matrix3d::Identity() - d * d.transpose();

        A += P;
        b += P * C;

        ++used;
    }

    GaussBlob<3> out;

    if (used == 0) {
        out.mu.setZero();
        out.cov = Eigen::Matrix3d::Identity() * 1e6;
        return out;
    }

    Eigen::Vector3d x =A.completeOrthogonalDecomposition().solve(b);

    Eigen::Matrix3d Lambda = Eigen::Matrix3d::Zero();
    Eigen::Vector3d eta = Eigen::Vector3d::Zero();

    for (int i = 0; i < N; ++i) {
        if (!rets[i]) continue;

        const Camera& cam = *sensors[i];

        cv::Mat K;
        cam.K.convertTo(K, CV_64F);

        double fx = K.at<double>(0, 0);
        double fy = K.at<double>(1, 1);
        double f = 0.5 * (fx + fy);

        double range = (x - Cs[i]).norm();
        range = std::max(range, 0.25);

        double sigmaAngle = sigmaPx[i] / f;
        double sigmaMeters = range * sigmaAngle;

        sigmaMeters = std::max(sigmaMeters, 0.003);

        Eigen::Matrix3d P =
            Eigen::Matrix3d::Identity() - ds[i] * ds[i].transpose();

        Eigen::Matrix3d Ii =
            P / (sigmaMeters * sigmaMeters);

        Lambda += Ii;
        eta += Ii * Cs[i];
    }

    Eigen::Vector3d mu = Lambda.completeOrthogonalDecomposition().solve(eta);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(Lambda);

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();

    const double maxVar = 100.0;      // 10 m stddev for unconstrained dirs
    const double minInfo = 1.0 / maxVar;

    for (int i = 0; i < 3; ++i) {
        double lambda = es.eigenvalues()(i);

        double var;
        if (lambda < minInfo) var = maxVar;
        else var = 1.0 / lambda;


        cov += var * es.eigenvectors().col(i)*es.eigenvectors().col(i).transpose();
    }

    out.mu = mu;
    out.cov = cov;

    return out;
}

template class InformationSystem<3>;