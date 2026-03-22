//both settings and config, as its pretty small rn
//should include form lib, but not util

#pragma once

#include "kinematics/Pose.h"
#include "kinematics/SCComms/packet.h"
#include "kinematics/SCComms/motorController.hpp"
#include "kinematics/motionPather/motionPather.hpp"
#include <array>
#include "config/config_loader.h" //so can include config and be good

//config settings
#define PI 3.14159
#define DOFS 7

constexpr double operator ""_m(long double val) {return val;}
constexpr double operator ""_cm(long double val) {return val/100;}
constexpr double operator ""_mm(long double val) {return val/1000;}
constexpr double operator ""_ft(long double val) {return val*0.3048;}
constexpr double operator ""_in(long double val) {return val/12*0.3048;}

//units in m
//relative to table height
#define TABLE_HEIGHT 29.75_in //roughly
#define TABLE_WIDTH 5._ft //x, width of table
#define TABLE_LENGTH 4.5_ft //y, length of half table !!

//w/ ref to paddle facing forward
#define PADDLE_HEIGHT 157.49_mm
#define PADDLE_ANC_TOPCENTER {{0, -4._mm, PADDLE_HEIGHT/2}}
#define PADDLE_ANC_TOPRIGHT {{59.887_mm, -4._mm, 47.225_mm}}
#define PADDLE_ANC_TOPLEFT {{-59.887_mm, -4._mm, 47.225_mm}}
#define PADDLE_ANC_MIDLEFT {{-74.917_mm, -4._mm, -19.694_mm}}
#define PADDLE_ANC_MIDRIGHT {{74.917_mm, -4._mm, -19.694_mm}}
#define PADDLE_ANC_BOTTOMRIGHT {{42.352_mm, -4._mm, -PADDLE_HEIGHT/2}}
#define PADDLE_ANC_BOTTOMLEFT {{-42.352_mm, -4._mm, -PADDLE_HEIGHT/2}}
constexpr std::array<std::array<double, 3>, 7> paddle_anchorOffsets = {{
    PADDLE_ANC_TOPRIGHT,
    PADDLE_ANC_BOTTOMRIGHT,
    PADDLE_ANC_MIDLEFT,
    PADDLE_ANC_TOPCENTER,
    PADDLE_ANC_TOPLEFT,
    PADDLE_ANC_BOTTOMLEFT,
    PADDLE_ANC_MIDRIGHT,
}};
constexpr std::array<double, 3> pulley_anchorOffsets_refOri = {0, 1, 0};

constexpr double pulley_diameter = 14._cm;


const std::string CONF_PATH = "/home/connor/PingPongRobot/core/config/";
const std::string KINCONFIG_PATH = "/home/connor/PingPongRobot/core/config/kin_conf.json"; //path to kinematics config
const std::string DETCONFIG_PATH = "/home/connor/PingPongRobot/core/config/vision/det_conf.json"; //path to ball detection config

//aliases
using PosFrame = PosFrameD<DOFS>;
using MotorController = MotorControllerD<DOFS, PosFrame>;

//constant positions. sp mean spherical
#define ORI_sp_FORWARD {0, 0}
#define ORI_sp_UPWARD {0, PI/2} 

constexpr Pose home_pose{{TABLE_WIDTH/2, PADDLE_HEIGHT/2, 8._mm}, 
                         ORI_sp_UPWARD};
constexpr Pose idle_pose{{TABLE_WIDTH/2, 50._cm, 0.4_m}, 
                         ORI_sp_FORWARD};

