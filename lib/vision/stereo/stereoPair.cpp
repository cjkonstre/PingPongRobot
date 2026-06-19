#include "vision/stereo/stereoPair.h"

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>


cv::Mat StereoPair::makeProjection(
    const cv::Mat& Kcv,
    const cv::Mat& Rcv,
    const cv::Mat& Tcv)
{
    Eigen::Matrix3d K, R;
    Eigen::Vector3d t;

    cv::cv2eigen(Kcv, K);
    cv::cv2eigen(Rcv, R);
    cv::cv2eigen(Tcv, t);

    Eigen::Matrix<double,3,4> Rt;
    Rt.block<3,3>(0,0) = R;
    Rt.col(3) = t;

    Eigen::Matrix<double,3,4> P = K * Rt;

    cv::Mat proj;
    cv::eigen2cv(P, proj);
    return proj;
}

StereoPair::StereoPair(Camera& c1, Camera& c2, std::string stereoConf_path):
    cam1(c1), cam2(c2) {

    std::string filename = "/home/connor/PingPongRobot/core/config/vision/" + cam1.capName + "+" + cam2.capName + "-stereoConf";
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["R"] >> R;
    fs["T"] >> T;
    fs["E"] >> E;
    fs["F"] >> F;
    cv::Mat cov; fs["cov"] >> cov; cv::cv2eigen(cov, covariance);

    proj1 = makeProjection(cam1.K, cv::Mat::eye(3,3,CV_64F), cv::Mat::zeros(3,1,CV_64F));
    proj2 = makeProjection(cam2.K, R, T);
}

GaussBlob<3> StereoPair::get3dMeasurement(
    const cv::Point2f& center1,
    const cv::Point2f& center2
){
    std::vector<cv::Point2f> pts1{center1};
    std::vector<cv::Point2f> pts2{center2};

    cv::Mat pts4D;
    cv::triangulatePoints(proj1, proj2, pts1, pts2, pts4D);

    cv::Mat X = pts4D.col(0);
    double w = X.at<double>(3);

    GaussBlob<3> measurement;

    measurement.mu <<
        X.at<double>(0) / w,
        X.at<double>(1) / w,
        X.at<double>(2) / w;

    measurement.cov = covariance;

    return measurement;
}

bool StereoPair::calibratePnP(
    const std::vector<cv::Point3f>& objectPoints,
    const std::vector<cv::Point2f>& imagePoints1,
    const std::vector<cv::Point2f>& imagePoints2,
    const Camera& cam1,
    const Camera& cam2,
    cv::Mat& R,
    cv::Mat& T,
    cv::Mat& E,
    cv::Mat& F
) {
    if (objectPoints.size() < 6 ||
        imagePoints1.size() != objectPoints.size() ||
        imagePoints2.size() != objectPoints.size()) {
        std::cerr << "invlid input sizes for cali\n";
        return false;
    }

    cv::Mat rvec1, tvec1;
    cv::Mat rvec2, tvec2;

    bool ok1 = cv::solvePnP(
        objectPoints,
        imagePoints1,
        cam1.K,
        cam1.D,
        rvec1,
        tvec1,
        false,
        cv::SOLVEPNP_ITERATIVE
    );

    bool ok2 = cv::solvePnP(
        objectPoints,
        imagePoints2,
        cam2.K,
        cam2.D,
        rvec2,
        tvec2,
        false,
        cv::SOLVEPNP_ITERATIVE
    );

    if (!ok1 || !ok2) {
        std::cerr << "solvePnP failed\n";
        return false;
    }

    cv::Mat R1, R2;
    cv::Rodrigues(rvec1, R1);
    cv::Rodrigues(rvec2, R2);

    // cam1 → cam2
    R = R2 * R1.t();
    T = tvec2 - R * tvec1;

    cv::Mat tx = (cv::Mat_<double>(3,3) <<
        0, -T.at<double>(2), T.at<double>(1),
        T.at<double>(2), 0, -T.at<double>(0),
        -T.at<double>(1), T.at<double>(0), 0
    );

    E = tx * R;
    F = cam2.K.inv().t() * E * cam1.K.inv();

    // diagnostics (optional but useful)
    cv::Mat rvec_rel;
    cv::Rodrigues(R, rvec_rel);
    double rotation_deg = cv::norm(rvec_rel) * 180.0 / CV_PI;

    std::cout << "Baseline (|T|): " << cv::norm(T) << "\n";
    std::cout << "Relative rotation: " << rotation_deg << " deg\n";

    return true;
}

bool StereoPair::calibrateToFile(
    const std::vector<cv::Point3f>& objectPoints,
    const std::vector<cv::Point2f>& imagePoints1,
    const std::vector<cv::Point2f>& imagePoints2,
    Camera& cam1,
    Camera& cam2,
    const std::string& outputPath
) {
    cv::Mat R, T, E, F;

    if (!calibratePnP(objectPoints, imagePoints1, imagePoints2,
                      cam1, cam2, R, T, E, F)) return false;

    cv::FileStorage fs(outputPath, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open " << outputPath << "\n";
        return false;
    }

    fs << "R" << R;
    fs << "T" << T;
    fs << "E" << E;
    fs << "F" << F;
    fs.release();

    std::cout << "Saved to " << outputPath << "\n";

    return true;
}