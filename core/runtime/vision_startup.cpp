#include "runtime/vision_startup.h"
//#include "utils.h"
#include "config/config.h"
#include "vision/camera/camera.h"

#ifdef DO_STUBBEDCAMS
 void synchCamrecsToNow(CameraRec& cam1, CameraRec& cam2, CameraRec& cam3){
    cam1.tsReader >> cam1.first_ts;
    cam2.tsReader >> cam2.first_ts;
    cam3.tsReader >> cam3.first_ts;
    uint64_t earlieststart = std::min({cam1.first_ts,  cam2.first_ts, cam3.first_ts});

    cam1.offset = cam1.first_ts - earlieststart;
    cam2.offset = cam2.first_ts - earlieststart;
    cam3.offset = cam3.first_ts - earlieststart;

    auto global_start_time = std::chrono::steady_clock::now();
    cam1.start_time = global_start_time;
    cam2.start_time = global_start_time;
    cam3.start_time = global_start_time;
} 

#endif


//cams still need to be shut down 
void setup_cams(){
    #ifdef DO_STUBBEDCAMS //do simmed rec
    const std::string saveedddir = "/home/connor/PingPongRobot/tests/test_data/";
    CameraRec cam_BL(saveedddir + "cam_BL_rev.avi", saveedddir + "cam_BL_ts.txt", CONF_PATH + "vision/cam_BL-intrinsics.yml");
    CameraRec cam_BR(saveedddir + "cam_BR_rev.avi", saveedddir + "cam_BR_ts.txt", CONF_PATH + "vision/cam_BR-intrinsics.yml");
    CameraRec cam_MR(saveedddir + "cam_MR_rev.avi", saveedddir + "cam_MR_ts.txt", CONF_PATH + "vision/cam_MR-intrinsics.yml");
    
    //synch cams
    synchCamrecsToNow(cam_BL, cam_BR, cam_MR);
    #else //do actual rec
    Camera camBL("/dev/cam_BL", CONF_PATH + "vision/cam_BL-intrinsics.yml", 1280, 800, 120, 35);
    Camera camBR("/dev/cam_BR", CONF_PATH + "vision/cam_BR-intrinsics.yml", 1280, 800, 120, 35);
    Camera camMR("/dev/cam_MR", CONF_PATH + "vision/cam_MR-intrinsics.yml", 1920, 1080, 120, 300);
    #endif
}
