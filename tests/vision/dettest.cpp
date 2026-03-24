#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include "misc/timeLog/timeLog.hpp"

using namespace std;


//!! any improvements made to this should be carried over to the balldet detection object !!
//this bypasses it for dev convinience
int main()
{
    cv::Mat mu, invS;
    double scrthresh = 10;

    string configPath = "/home/connor/PingPongRobot/core/config/vision/det_conf.yml";

    cv::FileStorage fs(configPath, cv::FileStorage::READ);
    cv::Mat Sigma;
    fs["cam_MR"]["mu"] >> mu;
    fs["cam_MR"]["Sigma"] >> Sigma;
    fs.release();

    if (mu.empty() || Sigma.empty()) throw runtime_error("Invalid model contents");

    mu.convertTo(mu, CV_32F);
    Sigma.convertTo(Sigma, CV_32F);
    cv::invert(Sigma, invS, cv::DECOMP_SVD);


    auto bgSub = cv::createBackgroundSubtractorMOG2(200, 16, false);

    cv::VideoCapture cap("/dev/cam_MR");
    if (!cap.isOpened())
        throw runtime_error("Failed to open camera");

    cap.set(cv::CAP_PROP_FOURCC,
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
    cap.set(cv::CAP_PROP_FPS, 120);

    cv::Mat frame;

    while (true)
    {
        TIMELOGDT << "start\n";
        cap >> frame;
        if (frame.empty()) break;

        TIMELOGDT << "got frame\n";

        cv::Mat blurred;
        cv::GaussianBlur(frame, blurred, {7,7}, 0);

        TIMELOGDT << " frae blurred mask gotten\n";

        cv::Mat fgMask;
        bgSub->apply(blurred, fgMask);
        cv::morphologyEx(
            fgMask, fgMask, cv::MORPH_OPEN,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, {3,3})
        );

        TIMELOGDT << " fg mask gotten\n";

        cv::Mat lab;
        cv::cvtColor(blurred, lab, cv::COLOR_BGR2Lab);

        vector<cv::Mat> ch;
        cv::split(lab, ch);

        cv::Mat a, b;
        ch[1].convertTo(a, CV_32F);
        ch[2].convertTo(b, CV_32F);

        TIMELOGDT << " lab mask gotten\n";

        cv::Mat ab;
        cv::hconcat(
            a.reshape(1, a.total()),
            b.reshape(1, b.total()),
            ab
        );
        TIMELOGDT << "  ab concat\n";

        cv::Mat mu_rep;
        cv::repeat(mu, ab.rows, 1, mu_rep);

        cv::Mat d = ab - mu_rep;

        cv::Mat temp = d * invS;
        cv::multiply(temp, d, temp);
        TIMELOGDT << "  temp calced\n";

        cv::Mat score;
        cv::reduce(temp, score, 1, cv::REDUCE_SUM);

        TIMELOGDT << "colmask calcs done\n";

        /*
        double minVal, maxVal;
        cv::minMaxLoc(score, &minVal, &maxVal);

        std::cout << "min score: " << minVal << std::endl;*/

        cv::Mat distMap = score.reshape(1, frame.rows);
        cv::Mat distVis;
        cv::normalize(distMap, distVis, 0, 255, cv::NORM_MINMAX);
        distVis.convertTo(distVis, CV_8U);
        cv::bitwise_not(distVis, distVis);

        cv::Mat colorMask = score < scrthresh;
        colorMask = colorMask.reshape(1, frame.rows);
        colorMask.convertTo(colorMask, CV_8U, 255);


        cv::Mat mask = colorMask;
        //cv::bitwise_and(colorMask, fgMask, mask); // !!!!!!!!

        TIMELOGDT << "masks raw done\n";


        cv::morphologyEx(
            mask, mask, cv::MORPH_OPEN,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, {5,5})
        );
        cv::morphologyEx(
            mask, mask, cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, {7,7})
        );

        TIMELOGDT << "mask morpho done\n";

        vector<vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        if (!contours.empty())
        {
            auto best = max_element(
                contours.begin(), contours.end(),
                [](const auto& a, const auto& b)
                { return cv::contourArea(a) < cv::contourArea(b); }
            );

            if (cv::contourArea(*best) >= 50)
            {
                cv::Point2f center;
                float rad;
                cv::minEnclosingCircle(*best, center, rad);
                cv::circle(frame, center, (int)rad, {0,255,0}, 2);
            }
        }

        TIMELOGDT << "contourfind finished \n";
        cv::imshow("bg", fgMask);
        cv::imshow("Likelihood Map", distVis);
        cv::imshow("mass", colorMask);
        cv::imshow("Ball Detection", frame);
        TIMELOGDT << "loop finished\n\n";

        if (cv::waitKey(1) == 27) break;
    }

    return 0;
}
