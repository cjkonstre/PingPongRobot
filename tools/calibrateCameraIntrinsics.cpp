// #Usage!!
// Standard:
// ./caliCam --camera /dev/cam_MR 5 7 0.02765 0.0075
//
// Fisheye:
// ./caliCam --fisheye --camera /dev/cam_MR 5 7 0.02765 0.0075
//
// press 'c' to capture image for calibration
// press ESC to finish and calibrate

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <stdexcept>

using namespace cv;
using namespace std;

bool isNumericCameraArg(const string& s) {
    return !s.empty() && s.find_first_not_of("-0123456789") == string::npos;
}

Point3f charucoObjectPointFromId(
    int id,
    int squaresX,
    float squareLength
) {
    int innerX = squaresX - 1;

    int xIdx = id % innerX;
    int yIdx = id / innerX;

    return Point3f(
        static_cast<float>(xIdx + 1) * squareLength,
        static_cast<float>(yIdx + 1) * squareLength,
        0.0f
    );
}

void charucoFrameToObjectImagePoints(
    const Mat& charucoCorners,
    const Mat& charucoIds,
    int squaresX,
    float squareLength,
    vector<Point3f>& objectPoints,
    vector<Point2f>& imagePoints
) {
    objectPoints.clear();
    imagePoints.clear();

    for (int i = 0; i < static_cast<int>(charucoCorners.total()); ++i) {
        int id = charucoIds.at<int>(i);
        Point2f imgPt = charucoCorners.at<Point2f>(i);

        objectPoints.push_back(
            charucoObjectPointFromId(id, squaresX, squareLength)
        );

        imagePoints.push_back(imgPt);
    }
}

int main(int argc, char** argv) {
    bool useFisheye = false;
    string cameraArg;
    vector<string> positional;

    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];

        if (arg == "--fisheye") useFisheye = true;
        else if (arg == "--camera") {
            if (i + 1 >= argc) throw runtime_error("--camera requires an argument");
            cameraArg = argv[++i];
        } else positional.push_back(arg);
    }

    if (cameraArg.empty() || positional.size() < 4) {
        cout << "Usage: " << argv[0]
                << " [--fisheye] --camera <idx|device> "
                << "<squaresX> <squaresY> <squareLength(m)> <markerLength(m)>"
                << endl;

        cout << "Example standard: "
                << argv[0]
                << " --camera /dev/cam_BL 5 7 0.02765 0.0075"
                << endl;

        cout << "Example fisheye:  "
                << argv[0]
                << " --fisheye --camera /dev/cam_MR 5 7 0.02765 0.0075"
                << endl;

        return 0;
    }

    int squaresX = stoi(positional[0]);
    int squaresY = stoi(positional[1]);
    float squareLength = stof(positional[2]);
    float markerLength = stof(positional[3]);

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_250);

    Ptr<aruco::CharucoBoard> charucoBoard =
        aruco::CharucoBoard::create(
            squaresX,
            squaresY,
            squareLength,
            markerLength,
            dictionary
        );

    Ptr<aruco::DetectorParameters> params =  aruco::DetectorParameters::create();

    params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;

    VideoCapture inputVideo;

    if (isNumericCameraArg(cameraArg)) {
        inputVideo.open(stoi(cameraArg));
        cout << "Opening camera index " << cameraArg << endl;
    } else {
        inputVideo.open(cameraArg);
        cout << "Opening camera device " << cameraArg << endl;
    }

    if (!inputVideo.isOpened()) throw runtime_error("failed to open camera: " + cameraArg);
    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
    inputVideo.set(cv::CAP_PROP_FPS, 120);
    inputVideo.set(cv::CAP_PROP_BUFFERSIZE, 1);

    cout << "Calibration model: "
            << (useFisheye ? "fisheye" : "standard")
            << endl;

    cout << "'c' to capture a frame, ESC to finish calibration." << endl;

    vector<vector<Point2f>> allCharucoCorners;
    vector<vector<int>> allCharucoIds;

    vector<vector<Point3f>> allObjectPointsFisheye;
    vector<vector<Point2f>> allImagePointsFisheye;

    Size imageSize;

    while (true) {
        Mat image;
        inputVideo >> image;

        if (image.empty()) {
            cerr << "empty frame" << endl;
            break;
        }

        imageSize = image.size();

        Mat imageCopy = image.clone();

        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners;

        aruco::detectMarkers(
            image,
            dictionary,
            markerCorners,
            markerIds,
            params
        );

        Mat charucoCorners;
        Mat charucoIds;

        if (!markerIds.empty()) {
            aruco::drawDetectedMarkers(
                imageCopy,
                markerCorners,
                markerIds
            );

            aruco::interpolateCornersCharuco(
                markerCorners,
                markerIds,
                image,
                charucoBoard,
                charucoCorners,
                charucoIds
            );

            if (charucoCorners.total() > 0) {
                aruco::drawDetectedCornersCharuco(
                    imageCopy,
                    charucoCorners,
                    charucoIds
                );
            }
        }

        string status =
            string(useFisheye ? "fisheye" : "standard") +
            " | captured=" + to_string(allCharucoCorners.size()) +
            " | visible corners=" + to_string(charucoCorners.total());

        putText(
            imageCopy,
            status,
            Point(20, 35),
            FONT_HERSHEY_SIMPLEX,
            0.75,
            Scalar(0, 255, 0),
            2
        );

        imshow("ChArUco Calibration", imageCopy);

        int key = waitKey(1);

        if (key == 27) {
            break;
        }

        if (key == 'c' || key == 'C') {
            if (charucoCorners.total() < 14) {
                cout << "not enough ChArUco corners in frame; skipped"
                        << endl;
                continue;
            }

            vector<Point2f> corners;
            vector<int> ids;

            for (int i = 0; i < static_cast<int>(charucoCorners.total()); ++i) {
                corners.push_back(charucoCorners.at<Point2f>(i));
                ids.push_back(charucoIds.at<int>(i));
            }

            allCharucoCorners.push_back(corners);
            allCharucoIds.push_back(ids);

            if (useFisheye) {
                vector<Point3f> objPts;
                vector<Point2f> imgPts;

                charucoFrameToObjectImagePoints(
                    charucoCorners,
                    charucoIds,
                    squaresX,
                    squareLength,
                    objPts,
                    imgPts
                );

                allObjectPointsFisheye.push_back(objPts);
                allImagePointsFisheye.push_back(imgPts);
            }

            cout << "captured frame "
                    << allCharucoCorners.size()
                    << " with "
                    << charucoCorners.total()
                    << " ChArUco corners"
                    << endl;
        }
    }

    destroyAllWindows();

    if (allCharucoCorners.size() < 10) {
        cerr << "Not enough frames for calibration. Capture at least 10."
                << endl;
        return -1;
    }

    Mat cameraMatrix;
    Mat distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    double repError = 0.0;

    if (useFisheye) {
        cameraMatrix = Mat::eye(3, 3, CV_64F);
        distCoeffs = Mat::zeros(4, 1, CV_64F);

        int flags =
            fisheye::CALIB_RECOMPUTE_EXTRINSIC |
            fisheye::CALIB_FIX_SKEW;

        TermCriteria criteria(
            TermCriteria::COUNT + TermCriteria::EPS,
            100,
            1e-6
        );

        repError = fisheye::calibrate(
            allObjectPointsFisheye,
            allImagePointsFisheye,
            imageSize,
            cameraMatrix,
            distCoeffs,
            rvecs,
            tvecs,
            flags,
            criteria
        );
    } else {
        int flags = CALIB_RATIONAL_MODEL;

        repError = aruco::calibrateCameraCharuco(
            allCharucoCorners,
            allCharucoIds,
            charucoBoard,
            imageSize,
            cameraMatrix,
            distCoeffs,
            rvecs,
            tvecs,
            flags
        );
    }

    cout << "\nCalibration finished." << endl;
    cout << "Model: " << (useFisheye ? "fisheye" : "standard") << endl;
    cout << "Image size: " << imageSize << endl;
    cout << "Frames used: " << allCharucoCorners.size() << endl;
    cout << "Reprojection error: " << repError << endl;
    cout << "Camera matrix:\n" << cameraMatrix << endl;
    cout << "Distortion coeffs:\n" << distCoeffs << endl;

    string camName = filesystem::path(cameraArg).filename();

    string filename =
        "/home/connor/PingPongRobot/core/config/vision/" +
        camName +
        "-conf.yml";

    FileStorage fs(filename, FileStorage::WRITE);

    if (!fs.isOpened()) {
        throw runtime_error("failed to write calibration file: " + filename);
    }

    fs << "camera_matrix" << cameraMatrix;
    fs << "dist_coeffs" << distCoeffs;
    fs << "distortion_model" << (useFisheye ? "fisheye" : "standard");
    fs << "reproj_err" << repError;

    fs.release();

    cout << "saved calibration to " << filename << endl;

    return 0;
}