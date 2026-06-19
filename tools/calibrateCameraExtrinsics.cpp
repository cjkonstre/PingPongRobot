#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>
#include <iomanip>
#include <sstream>

#include "config/config.h"

constexpr double DISPLAY_SCALE = 0.75;

std::string fmt3(float v) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3) << v;
    return ss.str();
}

struct CameraInfo {
    std::string name;
    std::string devicePath;
    std::string caliPath;

    cv::Mat K;
    cv::Mat D;

    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;

    cv::Mat R;
    cv::Mat t;
};

struct MapBounds {
    float xMin;
    float xMax;
    float yMin;
    float yMax;
};

struct ClickState {
    bool hasPoint = false;
    cv::Point2f pointDisplay;
};

enum class SelectionAction {
    Accept,
    Back,
    Next
};

struct SelectionRecord {
    bool visited = false;
    bool hasPoint = false;
    cv::Point2f imagePointOriginal;
};

struct SelectionUIResult {
    SelectionAction action;
    bool hasPoint = false;
    cv::Point2f pointOriginal;
};

struct CursorState {
    size_t xyIdx = 0;
    size_t camIdx = 0;
    size_t zIdx = 0;
};

CursorState decodeCursor(
    size_t linear,
    size_t numCams,
    size_t numZs
) {
    size_t perXY = numCams * numZs;

    CursorState c;
    c.xyIdx = linear / perXY;

    size_t rem = linear % perXY;
    c.camIdx = rem / numZs;
    c.zIdx = rem % numZs;

    return c;
}

static void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    (void)flags;

    if (event == cv::EVENT_LBUTTONDOWN) {
        auto* state = reinterpret_cast<ClickState*>(userdata);
        state->hasPoint = true;
        state->pointDisplay = cv::Point2f(
            static_cast<float>(x),
            static_cast<float>(y)
        );
    }
}

void drawMapWindow(
    const std::string& mapWindowName,
    const std::vector<cv::Point2f>& pointsArr,
    const cv::Point2f& currentXY,
    size_t xyIdx,
    const CameraInfo& cam,
    float z,
    const MapBounds& bounds
) {
    const int W = 500;
    const int H = 500;
    const int pad = 50;

    cv::Mat map(H, W, CV_8UC3, cv::Scalar(30, 30, 30));

    auto worldToMap = [&](const cv::Point2f& p) -> cv::Point {
        float xNorm = (p.x - bounds.xMin) / (bounds.xMax - bounds.xMin);
        float yNorm = (p.y - bounds.yMin) / (bounds.yMax - bounds.yMin);

        int px = static_cast<int>((1.0f - xNorm) * (W - 2 * pad) + pad);
        int py = static_cast<int>((1.0f - yNorm) * (H - 2 * pad) + pad);

        return cv::Point(px, py);
    };

    auto drawPoint = [&](const cv::Point2f& p, const cv::Scalar& color, int radius) {
        cv::Point q = worldToMap(p);
        cv::circle(map, q, radius, color, -1);
        cv::circle(map, q, radius + 3, color, 1);
    };

    for (const auto& p : pointsArr) {
        drawPoint(p, cv::Scalar(160, 160, 160), 4);
    }

    drawPoint(currentXY, cv::Scalar(0, 0, 255), 7);

    cv::Point origin = worldToMap(cv::Point2f(0.0f, 0.0f));

    if (origin.x >= 0 && origin.x < W && origin.y >= 0 && origin.y < H) {
        cv::line(
            map,
            cv::Point(origin.x - 8, origin.y),
            cv::Point(origin.x + 8, origin.y),
            cv::Scalar(255, 255, 255),
            1
        );

        cv::line(
            map,
            cv::Point(origin.x, origin.y - 8),
            cv::Point(origin.x, origin.y + 8),
            cv::Scalar(255, 255, 255),
            1
        );

        cv::putText(
            map,
            "0,0",
            origin + cv::Point(8, -8),
            cv::FONT_HERSHEY_SIMPLEX,
            0.45,
            cv::Scalar(255, 255, 255),
            1
        );
    }

    std::string status =
        "point " + std::to_string(xyIdx + 1) +
        "/" + std::to_string(pointsArr.size()) +
        " | " + cam.name +
        " | z=" + fmt3(z);

    cv::putText(
        map,
        status,
        cv::Point(20, 30),
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        cv::Scalar(0, 255, 0),
        2
    );

    cv::putText(
        map,
        "+y",
        cv::Point(W / 2 - 10, 25),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(200, 200, 200),
        1
    );

    cv::putText(
        map,
        "+x",
        cv::Point(pad - 35, H / 2),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(200, 200, 200),
        1
    );

    cv::putText(
        map,
        "-x",
        cv::Point(W - pad + 10, H / 2),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(200, 200, 200),
        1
    );

    cv::putText(
        map,
        "-y",
        cv::Point(W / 2 - 10, H - 15),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(200, 200, 200),
        1
    );

    cv::imshow(mapWindowName, map);
}

void loadIntrinsics(CameraInfo& cam) {
    cv::FileStorage fs(cam.caliPath, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        throw std::runtime_error("failed to open cali file: " + cam.caliPath);
    }

    fs["camera_matrix"] >> cam.K;
    fs["dist_coeffs"] >> cam.D;

    if (cam.K.empty() || cam.D.empty()) {
        throw std::runtime_error("missing camera_matrix or dist_coeffs in: " + cam.caliPath);
    }

    cam.K.convertTo(cam.K, CV_64F);
    cam.D.convertTo(cam.D, CV_64F);
}

void saveCalibrationWithExtrinsics(const CameraInfo& cam) {
    cv::FileStorage fs(cam.caliPath, cv::FileStorage::WRITE);

    if (!fs.isOpened()) {
        throw std::runtime_error("failed to write cali file: " + cam.caliPath);
    }

    fs << "camera_matrix" << cam.K;
    fs << "dist_coeffs" << cam.D;
    fs << "rot_mat" << cam.R;
    fs << "trans_mat" << cam.t;

    fs.release();
}

void saveCollectedPoints(
    const std::string& path,
    const std::vector<CameraInfo>& cameras
) {
    cv::FileStorage fs(path, cv::FileStorage::WRITE);

    if (!fs.isOpened()) {
        throw std::runtime_error("failed to write collected points file: " + path);
    }

    for (const auto& cam : cameras) {
        fs << cam.name << "{";
        fs << "object_points" << cam.objectPoints;
        fs << "image_points" << cam.imagePoints;
        fs << "}";
    }

    fs.release();

    std::cout << "\nsaved collected points to: " << path << std::endl;
}

void loadCollectedPoints(
    const std::string& path,
    std::vector<CameraInfo>& cameras
) {
    cv::FileStorage fs(path, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        throw std::runtime_error("failed to read collected points file: " + path);
    }

    for (auto& cam : cameras) {
        cv::FileNode node = fs[cam.name];

        if (node.empty()) {
            throw std::runtime_error("missing camera section in points file: " + cam.name);
        }

        cam.objectPoints.clear();
        cam.imagePoints.clear();

        node["object_points"] >> cam.objectPoints;
        node["image_points"] >> cam.imagePoints;

        if (cam.objectPoints.size() != cam.imagePoints.size()) {
            throw std::runtime_error(cam.name + ": object_points and image_points size mismatch");
        }

        std::cout << "loaded " << cam.objectPoints.size()
                  << " points for " << cam.name << std::endl;
    }

    fs.release();
}

SelectionUIResult selectPointFromCamera(
    std::vector<cv::VideoCapture>& caps,
    size_t activeCamIdx,
    const std::string& windowName,
    const std::string& mapWindowName,
    const std::vector<cv::Point2f>& pointsArr,
    const MapBounds& mapBounds,
    const CameraInfo& cam,
    const cv::Point2f& xy,
    size_t xyIdx,
    float z,
    const SelectionRecord& existing
){
    ClickState state;

    if (existing.hasPoint) {
        state.hasPoint = true;
        state.pointDisplay =
            existing.imagePointOriginal * static_cast<float>(DISPLAY_SCALE);
    }

    cv::setMouseCallback(windowName, mouseCallback, &state);

    bool mapDrawn = false;
    while (true) {
        bool activeGrabbed = false;

        for (size_t i = 0; i < caps.size(); ++i) {
            bool ok = caps[i].grab();

            if (i == activeCamIdx) {
                activeGrabbed = ok;
            }
        }

        if (!activeGrabbed) {
            std::cerr << "failed to grab frame from " << cam.devicePath << std::endl;
            continue;
        }

        cv::Mat frame;

        if (!caps[activeCamIdx].retrieve(frame) || frame.empty()) {
            std::cerr << "failed to retrieve frame from " << cam.devicePath << std::endl;
            continue;
        }

        if (frame.empty()) {
            std::cerr << "empty frame from " << cam.devicePath << std::endl;
            continue;
        }

        cv::Mat scaled;
        cv::resize(
            frame,
            scaled,
            cv::Size(
                static_cast<int>(1280 * DISPLAY_SCALE),
                static_cast<int>(800 * DISPLAY_SCALE)
            ),
            0,
            0,
            cv::INTER_AREA
        );

        cv::Mat display = scaled.clone();

        std::string line1 =
            "point " + std::to_string(xyIdx + 1) +
            "/" + std::to_string(pointsArr.size()) +
            " | " + cam.name +
            " | x=" + fmt3(xy.x) +
            " y=" + fmt3(xy.y) +
            " z=" + fmt3(z);

        cv::putText(
            display,
            line1,
            cv::Point(20, 35),
            cv::FONT_HERSHEY_SIMPLEX,
            0.65,
            cv::Scalar(0, 255, 0),
            2
        );

        if (state.hasPoint) {
            cv::circle(display, state.pointDisplay, 5, cv::Scalar(0, 0, 255), -1);
            cv::circle(display, state.pointDisplay, 10, cv::Scalar(0, 0, 255), 2);

            cv::Point2f pointOriginal =
                state.pointDisplay * static_cast<float>(1.0 / DISPLAY_SCALE);

            std::string clicked =
                "stored: (" +
                fmt3(pointOriginal.x) +
                ", " +
                fmt3(pointOriginal.y) +
                ")";

            cv::putText(
                display,
                clicked,
                cv::Point(20, 70),
                cv::FONT_HERSHEY_SIMPLEX,
                0.65,
                cv::Scalar(0, 0, 255),
                2
            );
        }

        if (!mapDrawn) {
    drawMapWindow(
        mapWindowName,
        pointsArr,
        xy,
        xyIdx,
        cam,
        z,
        mapBounds
    );

    mapDrawn = true;
}

        cv::imshow(windowName, display);

        int key = cv::waitKey(1);

        if (key == 13 || key == 10) {
            SelectionUIResult out;
            out.action = SelectionAction::Accept;

            if (state.hasPoint) {
                out.hasPoint = true;
                out.pointOriginal =
                    state.pointDisplay * static_cast<float>(1.0 / DISPLAY_SCALE);
            }

            return out;
        }

        if (key == 'x' || key == 'X') {
            state.hasPoint = false;
            continue;
        }

        if (key == 'b' || key == 'B') {
            SelectionUIResult out;
            out.action = SelectionAction::Back;
            return out;
        }

        if (key == 'n' || key == 'N') {
            SelectionUIResult out;
            out.action = SelectionAction::Next;
            return out;
        }

        if (key == 27) {
            throw std::runtime_error("user exited with ESC");
        }
    }
}
void solveExtrinsics(CameraInfo& cam) {
    if (cam.objectPoints.size() < 4) {
        throw std::runtime_error(cam.name + ": need at least 4 correspondences for solvePnP");
    }

    cv::Mat rvec;
    cv::Mat tvec;

    bool ok = cv::solvePnP(
        cam.objectPoints,
        cam.imagePoints,
        cam.K,
        cam.D,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_ITERATIVE
    );

    if (!ok) {
        throw std::runtime_error(cam.name + ": solvePnP failed");
    }

    cv::Rodrigues(rvec, cam.R);
    cam.t = tvec;

    std::vector<cv::Point2f> projected;

    cv::projectPoints(
        cam.objectPoints,
        rvec,
        tvec,
        cam.K,
        cam.D,
        projected
    );

    double sumPx = 0.0;
    double sumSqPx = 0.0;
    double maxPx = 0.0;

    for (size_t i = 0; i < projected.size(); ++i) {
        double err = cv::norm(projected[i] - cam.imagePoints[i]);
        sumPx += err;
        sumSqPx += err * err;
        maxPx = std::max(maxPx, err);
    }

    double meanPx = sumPx / static_cast<double>(projected.size());
    double rmsPx = std::sqrt(sumSqPx / static_cast<double>(projected.size()));

    cv::Mat cameraCenterWorld = -cam.R.t() * cam.t;

    double sumRayM = 0.0;
    double sumSqRayM = 0.0;
    double maxRayM = 0.0;

    cv::Mat Kinv = cam.K.inv();

    for (size_t i = 0; i < cam.objectPoints.size(); ++i) {
        const cv::Point2f& uv = cam.imagePoints[i];
        const cv::Point3f& Pw_f = cam.objectPoints[i];

        cv::Mat pix = (cv::Mat_<double>(3, 1) << uv.x, uv.y, 1.0);

        cv::Mat rayCam = Kinv * pix;
        rayCam = rayCam / cv::norm(rayCam);

        cv::Mat rayWorld = cam.R.t() * rayCam;
        rayWorld = rayWorld / cv::norm(rayWorld);

        cv::Mat Pw = (cv::Mat_<double>(3, 1) << Pw_f.x, Pw_f.y, Pw_f.z);
        cv::Mat C = cameraCenterWorld;

        cv::Mat CP = Pw - C;

        double along = CP.dot(rayWorld);
        cv::Mat closest = C + along * rayWorld;

        double rayErr = cv::norm(Pw - closest);

        sumRayM += rayErr;
        sumSqRayM += rayErr * rayErr;
        maxRayM = std::max(maxRayM, rayErr);
    }

    double meanRayM = sumRayM / static_cast<double>(cam.objectPoints.size());
    double rmsRayM = std::sqrt(sumSqRayM / static_cast<double>(cam.objectPoints.size()));

    std::cout << "\n===== " << cam.name << " =====" << std::endl;
    std::cout << "num correspondences: " << cam.objectPoints.size() << std::endl;

    std::cout << "R:\n" << cam.R << std::endl;
    std::cout << "t:\n" << cam.t << std::endl;
    std::cout << "camera center in world:\n" << cameraCenterWorld << std::endl;

    std::cout << "\nimage reprojection error:" << std::endl;
    std::cout << "  mean: " << meanPx << " px" << std::endl;
    std::cout << "  rms:  " << rmsPx << " px" << std::endl;
    std::cout << "  max:  " << maxPx << " px" << std::endl;

    std::cout << "\nworld ray-point distance:" << std::endl;
    std::cout << "  mean: " << meanRayM << " m" << std::endl;
    std::cout << "  rms:  " << rmsRayM << " m" << std::endl;
    std::cout << "  max:  " << maxRayM << " m" << std::endl;

    std::cout << "\nrough interpretation:" << std::endl;

    if (rmsPx < 1.0) std::cout << "  reprojection: excellent" << std::endl;
    else if (rmsPx < 2.0) std::cout << "  reprojection: good" << std::endl;
    else if (rmsPx < 5.0) std::cout << "  reprojection: usable, eh" << std::endl;
    else std::cout << "  reprojection: bad;" << std::endl;

    if (rmsRayM < 0.005) std::cout << "  3D consistency: excellent, under 5 mm RMS" << std::endl;
    else if (rmsRayM < 0.015) std::cout << "  3D consistency: good, under 1.5 cm RMS" << std::endl;
    else if (rmsRayM < 0.04) std::cout << "  3D consistency: usable, but not precise" << std::endl;
    else std::cout << "  3D consistency: poor; " << std::endl;
}

//#define DO_DATA_COLLECTION

int main() {
    try {
        std::vector<cv::Point2f> pointsArr = {
            {0, TABLE_LENGTH - 10._cm},
            {0, TABLE_LENGTH - 50._cm},
            {0, 60._cm},
            {0, 30._cm},
            {0, 0},

            {40._cm, TABLE_LENGTH - 10._cm},
            {TABLE_WIDTH / 2 - 50._cm, TABLE_LENGTH - 50._cm},
            {30._cm, 60._cm},
            {30._cm, 30._cm},
            {30._cm, 0._cm},

            {60._cm, 60._cm},
            {60._cm, 30._cm},
            {60._cm, 0._cm},

            {TABLE_WIDTH / 2, TABLE_LENGTH - 10._cm},
            {TABLE_WIDTH / 2, TABLE_LENGTH - 50._cm},

            {TABLE_WIDTH, TABLE_LENGTH - 10._cm},
            {TABLE_WIDTH, TABLE_LENGTH - 50._cm},
            {TABLE_WIDTH, 60._cm},
            {TABLE_WIDTH, 30._cm},
            {TABLE_WIDTH, 0},

            {TABLE_WIDTH - 40._cm, TABLE_LENGTH - 10._cm},
            {TABLE_WIDTH / 2 + 50._cm, TABLE_LENGTH - 50._cm},
            {TABLE_WIDTH - 30._cm, 60._cm},
            {TABLE_WIDTH - 30._cm, 30._cm},
            {TABLE_WIDTH - 30._cm, 0._cm},

            {TABLE_WIDTH - 60._cm, 60._cm},
            {TABLE_WIDTH - 60._cm, 30._cm},
            {TABLE_WIDTH - 60._cm, 0._cm},

            {TABLE_WIDTH / 2 - 25._cm, TABLE_LENGTH - 50._cm},
            {TABLE_WIDTH / 2 + 25._cm, TABLE_LENGTH - 50._cm},
        };

        std::vector<float> zs = {
            static_cast<float>(0.0),
            static_cast<float>(20._cm),
            static_cast<float>(40._cm),
            static_cast<float>(60._cm),
            static_cast<float>(80._cm)
        };

        std::vector<CameraInfo> cameras = {
            {
                "cam_BL",
                "/dev/cam_BL",
                "/home/connor/PingPongRobot/core/config/vision/cam_BL-conf.yml"
            },
            {
                "cam_BR",
                "/dev/cam_BR",
                "/home/connor/PingPongRobot/core/config/vision/cam_BR-conf.yml"
            },
            {
                "cam_MR",
                "/dev/cam_MR",
                "/home/connor/PingPongRobot/core/config/vision/cam_MR-conf.yml"
            }
        };

        const std::string correspondencePath =
            "/home/connor/PingPongRobot/core/config/vision/extrinsic_points_raw.yml";

        const std::string windowName = "extrinsic calibration";
        const std::string mapWindowName = "calibration map";

        MapBounds mapBounds {
            static_cast<float>(-10._cm),
            static_cast<float>(TABLE_WIDTH + 10._cm),
            static_cast<float>(-10._cm),
            static_cast<float>(TABLE_LENGTH + 10._cm)
        };

        for (auto& cam : cameras) {
            loadIntrinsics(cam);
            std::cout << "loaded intrinsics for " << cam.name << std::endl;
        }

#ifdef DO_DATA_COLLECTION
        std::vector<cv::VideoCapture> caps;
        caps.reserve(cameras.size());

        for (const auto& cam : cameras) {
            caps.emplace_back(cam.devicePath);

            if (!caps.back().isOpened()) {
                throw std::runtime_error("failed to open video stream: " + cam.devicePath);
            }

            caps.back().set(
                cv::CAP_PROP_FOURCC,
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G')
            );

            caps.back().set(cv::CAP_PROP_FRAME_WIDTH, 1280);
            caps.back().set(cv::CAP_PROP_FRAME_HEIGHT, 800);
            caps.back().set(cv::CAP_PROP_FPS, 120);
            caps.back().set(cv::CAP_PROP_BUFFERSIZE, 1);

            std::cout << "opened stream for " << cam.name << std::endl;
        }

        cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(mapWindowName, cv::WINDOW_AUTOSIZE);

        std::vector<std::vector<std::vector<SelectionRecord>>> selections(
            cameras.size(),
            std::vector<std::vector<SelectionRecord>>(
                pointsArr.size(),
                std::vector<SelectionRecord>(zs.size())
            )
        );

        size_t totalStates = pointsArr.size() * cameras.size() * zs.size();
        size_t currentLinear = 0;
        size_t furthestLinear = 0;

        while (currentLinear < totalStates) {
            CursorState cur = decodeCursor(
                currentLinear,
                cameras.size(),
                zs.size()
            );

            auto& cam = cameras[cur.camIdx];

            const cv::Point2f& xy = pointsArr[cur.xyIdx];
            float z = zs[cur.zIdx];

            SelectionRecord& rec =
                selections[cur.camIdx][cur.xyIdx][cur.zIdx];

                    SelectionUIResult result = selectPointFromCamera(
            caps,
            cur.camIdx,
            windowName,
            mapWindowName,
            pointsArr,
            mapBounds,
            cam,
            xy,
            cur.xyIdx,
            z,
            rec
        );

            if (result.action == SelectionAction::Back) {
                if (currentLinear > 0) {
                    --currentLinear;
                }

                continue;
            }

            if (result.action == SelectionAction::Next) {
                if (currentLinear < furthestLinear) {
                    ++currentLinear;
                }

                continue;
            }

            rec.visited = true;
            rec.hasPoint = result.hasPoint;

            if (result.hasPoint) {
                rec.imagePointOriginal = result.pointOriginal;

                std::cout << "stored "
                          << cam.name
                          << " point=" << cur.xyIdx + 1
                          << " world=("
                          << xy.x << ", "
                          << xy.y << ", "
                          << z << ") image=("
                          << result.pointOriginal.x << ", "
                          << result.pointOriginal.y << ")"
                          << std::endl;
            } else {
                std::cout << "skipped "
                          << cam.name
                          << " point=" << cur.xyIdx + 1
                          << " world=("
                          << xy.x << ", "
                          << xy.y << ", "
                          << z << ")"
                          << std::endl;
            }

            if (currentLinear == furthestLinear && furthestLinear + 1 < totalStates) {
                ++furthestLinear;
            }

            ++currentLinear;
        }

        for (auto& cam : cameras) {
            cam.objectPoints.clear();
            cam.imagePoints.clear();
        }

        for (size_t camIdx = 0; camIdx < cameras.size(); ++camIdx) {
            auto& cam = cameras[camIdx];

            for (size_t xyIdx = 0; xyIdx < pointsArr.size(); ++xyIdx) {
                for (size_t zIdx = 0; zIdx < zs.size(); ++zIdx) {
                    const SelectionRecord& rec =
                        selections[camIdx][xyIdx][zIdx];

                    if (!rec.visited || !rec.hasPoint) {
                        continue;
                    }

                    const cv::Point2f& xy = pointsArr[xyIdx];
                    float z = zs[zIdx];

                    cam.objectPoints.emplace_back(xy.x, xy.y, z);
                    cam.imagePoints.push_back(rec.imagePointOriginal);
                }
            }
        }

        cv::destroyWindow(windowName);
        cv::destroyWindow(mapWindowName);

        for (auto& cap : caps) {
            cap.release();
        }

        saveCollectedPoints(correspondencePath, cameras);
#endif

        loadCollectedPoints(correspondencePath, cameras);

        for (auto& cam : cameras) {
            solveExtrinsics(cam);
            saveCalibrationWithExtrinsics(cam);

            std::cout << "saved extrinsics to "
                      << cam.caliPath << std::endl;
        }

        std::cout << "\nDone.\n";
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "\nERROR: " << e.what() << std::endl;
        return 1;
    }
}