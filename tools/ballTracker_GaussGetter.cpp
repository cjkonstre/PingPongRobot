//to detect the ball a gaussian somehting is used to get it. this gets the info needed for that color magic

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <iostream>
#include <string>
#include <nlohmann/json.hpp>
#include "config/config.h"

using namespace cv;
using namespace std;
using json = nlohmann::json;

json matToJson(const cv::Mat& m)
{
    json j;
    j["rows"] = m.rows;
    j["cols"] = m.cols;

    std::vector<float> data;
    data.assign((float*)m.datastart, (float*)m.dataend);

    j["data"] = data;

    return j;
}

int main(int argc, char** argv) {
    if (argc < 1) {
        cout << "Usage: " << argv[0]
             << " --camera <idx>" << endl;
        return 0;
    }

    string cameraArg;
    int cameraIdx = -1;

    if (string(argv[1]) == "--camera") {
        cameraArg = argv[2];
        argv += 2;
        argc -= 2;
    }

    std::string camName = std::filesystem::path(cameraArg).filename().string();

    VideoCapture inputVideo;
    inputVideo.open(cameraArg);
    inputVideo.set(cv::CAP_PROP_FOURCC,
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
    inputVideo.set(cv::CAP_PROP_FPS, 120);

    std::cout << "'c' to collect foreground, ball ROI (do inside) \n'esc to finish";
    std::vector<cv::Mat> fg_samples;
    while (true){
        Mat image, imageCopy;
        inputVideo >> image;
        if (image.empty()) break;

        cv::GaussianBlur(image, image, cv::Size(3, 3), 0);
        Mat lab;
        cv::cvtColor(image, lab, cv::COLOR_BGR2Lab);

        cv::imshow("frame", image);
        int k = cv::waitKey(1);

        if (k==99){ //'c', cap ROI
            cv::Rect roi = cv::selectROI("frame", image, false);
            if (roi.width>0 && roi.height>0){
                cv::Mat lab_roi = lab(roi);
                std::vector<cv::Mat> channels;
                cv::split(lab_roi, channels);
                cv::Mat ab;
                cv::hconcat(
                    channels[1].reshape(1, roi.width*roi.height),
                    channels[2].reshape(1, roi.width*roi.height),
                    ab
                );

                ab.convertTo(ab, CV_32F);
                fg_samples.push_back(ab);
            }
        } else if (k==27) break;
    }

    if (fg_samples.empty()) {
        std::cerr << "No foreground samples collected. Exiting.\n";
        return 1;
    }

    cv::Mat fg_ab, bg_ab;
    cv::vconcat(fg_samples, fg_ab);

    cv::Mat mu, Sigma;
    cv::reduce(fg_ab, mu, 0, cv::REDUCE_AVG);  // 1×2

    cv::calcCovarMatrix(
        fg_ab,
        Sigma,
        mu,
        cv::COVAR_NORMAL | cv::COVAR_ROWS
    );

    Sigma /= (fg_ab.rows - 1);

    mu.convertTo(mu, CV_32F);
    Sigma.convertTo(Sigma, CV_32F);

    cv::Mat invS; cv::invert(Sigma, invS, cv::DECOMP_SVD);

    json config;

    /* load existing file if present */
    if (std::filesystem::exists(DETCONFIG_PATH)) {
        std::ifstream in(DETCONFIG_PATH);
        in >> config;
    }

    /* overwrite or insert this camera */
    config[camName]["mu"] = matToJson(mu);
    config[camName]["Sigma"] = matToJson(Sigma);

    /* write back */
    std::ofstream out(DETCONFIG_PATH);
    out << config.dump(4);

    std::cout << "Saved Gaussian model for "
            << camName << " to "
            << DETCONFIG_PATH << std::endl;

    while (true) {
        cv::Mat frame;
        if (!inputVideo.read(frame)) continue;

        cv::GaussianBlur(frame, frame, cv::Size(3,3), 0);

        cv::Mat lab;
        cv::cvtColor(frame, lab, cv::COLOR_BGR2Lab);

        cv::imshow("frame", frame);

        // Extract a,b channels
        std::vector<cv::Mat> ch;
        cv::split(lab, ch);

        cv::Mat a = ch[1];
        cv::Mat b = ch[2];

        a.convertTo(a, CV_32F);
        b.convertTo(b, CV_32F);

        cv::Mat ab;
        cv::hconcat(
            a.reshape(1, a.total()),
            b.reshape(1, b.total()),
            ab
        );

        // d = ab - mu
        cv::Mat d;
        cv::Mat mu_rep;
        cv::repeat(mu, ab.rows, 1, mu_rep);

        cv::subtract(ab, mu_rep, d);

        cv::Mat temp = d * invS;
        cv::Mat score;

        cv::multiply(temp, d, temp);
        cv::reduce(temp, score, 1, cv::REDUCE_SUM);  // N×1

        cv::Mat likelihood;
        cv::exp(-0.5 * score, likelihood);

        cv::Mat mask = likelihood > 0.2f;

        likelihood = likelihood.reshape(1, lab.rows);
        mask = mask.reshape(1, lab.rows);

        cv::imshow("faef", likelihood);
        cv::imshow("lk", mask * 255);

        int key = cv::waitKey(1);
        if (key == 'q')
            break;
    }


}