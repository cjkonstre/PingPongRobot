#include "vision/camera/camera.h"
#include "vision/ballDet/ballDet.h"
#include "vision/stereo/multiStereo.h"
#include "config/config.h"
#include "utils.h"

#include <algorithm>
//test cameraRec class

BallDetector balldet(DETCONFIG_PATH, "cam_BL"); //doing bl, idk if i want to keep the different settings per thing idk

void dothething(Frame frame, std::string name){
    if (frame.timestamp_us == 0 || frame.frame.empty()) return;
    cv::Mat im = frame.frame.clone();

    cv::Point2f c; float r;
    //balldet.findBall(im, c, r); 

    //cv::circle(im, c, r, (0, 255, 0), 5);
    cv::imshow(name, im);
}

int main(){
    const std::string saveedddir = "/home/connor/PingPongRobot/tests/test_data/";
    CameraRec cam_BL(saveedddir + "cam_BL_rev.avi", saveedddir + "cam_BL_ts.txt", CONF_PATH + "vision/cam_BL-intrinsics.yml");
    CameraRec cam_BR(saveedddir + "cam_BR_rev.avi", saveedddir + "cam_BR_ts.txt", CONF_PATH + "vision/cam_BR-intrinsics.yml");
    CameraRec cam_MR(saveedddir + "cam_MR_rev.avi", saveedddir + "cam_MR_ts.txt", CONF_PATH + "vision/cam_MR-intrinsics.yml");
    
    //synch cams
    synchCamrecsToNow(cam_BL, cam_BR, cam_MR);
    cam_BL.beginLoop();
    cam_BR.beginLoop();
    cam_MR.beginLoop();

    for (;;) {
        //loop

        dothething(cam_BL.frame_buffer[0], "camBL");
        dothething(cam_BR.frame_buffer[0], "camBR");
        dothething(cam_MR.frame_buffer[0], "camMR");

        if (cv::waitKey(1)==27) break;
    }

    cam_BL.release();
    cam_BR.release();
    cam_MR.release();
    return 0;
}