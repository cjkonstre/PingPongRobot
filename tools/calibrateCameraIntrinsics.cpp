// #Usage!!
// `./charuco_calib --camera <camidx> <Xdim> <Ydim> <squarreSize [m]> <marker size [m]>`
// press 'c' to cap image for cali. it will tak like 20-25 shots
//run from like GCC 14.2.0 x86_64-linux-gnu
// ./caliCam --camera -1 5 7 0.02765 0.0075 ; these are the dims for the printed out charuco


#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <iostream>
#include <string>
#include <filesystem>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    if (argc < 6) {
        cout << "Usage: " << argv[0]
             << " --camera <idx> <squaresX> <squaresY> <squareLength(m)> <markerLength(m)>" << endl;
        cout << "Example: " << argv[0] << " --camera /dev/video2 5 7 0.04 0.02" << endl;
        return 0;
    }

    string cameraArg;
    int cameraIdx = -1;

    if (string(argv[1]) == "--camera") {
        cameraArg = argv[2];
        argv += 2;
        argc -= 2;
    }

    int squaresX = atoi(argv[1]);
    int squaresY = atoi(argv[2]);
    float squareLength = atof(argv[3]);
    float markerLength = atof(argv[4]);

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_250);

    Ptr<aruco::CharucoBoard> charucoBoard =
        aruco::CharucoBoard::create(
            squaresX,
            squaresY,
            squareLength,
            markerLength,
            dictionary
        );

    VideoCapture inputVideo;
    if (!cameraArg.empty() &&
        cameraArg.find_first_not_of("-0123456789") == string::npos) {
        // numeric index
        inputVideo.open(stoi(cameraArg));
        cout << "Opening camera index " << cameraArg << endl;
    } else {
        // device path
        inputVideo.open(cameraArg);
        cout << "Opening camera device " << cameraArg << endl;
    }

    cout << "'c' to capture a frame, 'esc' to finish calibration." << endl;

    vector<vector<Point2f>> allCharucoCorners;
    vector<vector<int>> allCharucoIds;
    Size imageSize;

    while (true) {
        Mat image, imageCopy;
        inputVideo >> image;
        if (image.empty()) break;
        image.copyTo(imageCopy);
        imageSize = image.size();

        // ---- Detect ArUco markers ----
        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners;

        auto params = aruco::DetectorParameters::create();
        params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
        aruco::detectMarkers(image, dictionary, markerCorners, markerIds, params);

        // Always show the live feed
        if (!markerIds.empty()) {
            // draw markers
            aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);

            // interpolate ChArUco corners
            Mat charucoCorners, charucoIds;
            aruco::interpolateCornersCharuco(
                markerCorners, markerIds, image,
                charucoBoard, charucoCorners, charucoIds);

            if (charucoCorners.total() > 0) {
                aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds);
            }

            // cap when c pressed
            char key = (char)waitKey(30);
            if (key == 'c' && charucoCorners.total() > 0) {
                cout << "capped frame for cali" << endl;
                vector<Point2f> corners;
                vector<int> ids;
                for (int i = 0; i < charucoCorners.total(); i++) {
                    corners.push_back(charucoCorners.at<Point2f>(i));
                    ids.push_back(charucoIds.at<int>(i));
                }
                allCharucoCorners.push_back(corners);
                allCharucoIds.push_back(ids);
            }
            if (key == 27) break; // esc
        }

        // Show image every loop, even if no markers found
        imshow("ChArUco Calibration", imageCopy);
        if (waitKey(1) == 27) break; // ESC fallback
    }

    if (allCharucoCorners.size() < 10) {
        cerr << "Not enough frames for calibration. Capture at least 10." << endl;
        return -1;
    }

    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    double repError = aruco::calibrateCameraCharuco(
        allCharucoCorners, allCharucoIds, charucoBoard, imageSize,
        cameraMatrix, distCoeffs, rvecs, tvecs,
        CALIB_RATIONAL_MODEL); //higher level lense distortion

    cout << "Calibration finished." << endl;
    cout << "Reprojection error: " << repError << endl;
    cout << "Camera matrix:\n" << cameraMatrix << endl;
    cout << "Distortion coeffs:\n" << distCoeffs << endl;

    string camName = filesystem::path(cameraArg).filename();
    string filename = "/home/connor/PingPongRobot/core/config/vision/" + camName + "-intrinsics.yml";

    FileStorage fs(filename, FileStorage::WRITE);
    fs << "camera_matrix" << cameraMatrix;
    fs << "dist_coeffs" << distCoeffs;
    fs << "reproj_err" << repError;
    fs.release();

    cout << "saved calibration to " << filename << endl;
    return 0;
}
