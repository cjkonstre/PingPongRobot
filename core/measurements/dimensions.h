#pragma once

#include <array>
#include "kinematics/inverseK/inverseKin.hpp"

constexpr double operator ""_cm(long double val) {return val/100;}
constexpr double operator ""_mm(long double val) {return val/1000;}
constexpr double operator ""_ft(long double val) {return val*0.3048;}
constexpr double operator ""_in(long double val) {return val/12*0.3048;}

//units in m
//relative to table height
#define TABLE_HEIGHT 29.75_in //roughly
#define TABLE_WIDTH 5._ft //x, width of table
#define TABLE_LENGTH 4.5_ft //y, length of half table !!
constexpr std::array<std::array<double, 3>, 7> frame_pulleyPoss = {{
    {{TABLE_WIDTH + 1.5_ft + 1._cm, TABLE_LENGTH-3._cm, 6._ft+3.7_cm - TABLE_HEIGHT}},
    {{TABLE_WIDTH+30._cm, TABLE_LENGTH-3._cm, 26.75_in+8.5_cm - TABLE_HEIGHT}},
    {{-18._in+2._cm, -2._ft+3.5_in+3._cm, 2.5_ft+3.7_cm-TABLE_HEIGHT}},
    {{TABLE_WIDTH/2-3.5_cm, -2._ft+3.5_in+3._cm, 64.75_in-TABLE_HEIGHT}},
    {{-1.5_ft -1._cm, TABLE_LENGTH-3._cm, 6._ft+3.7_cm - TABLE_HEIGHT}},
    {{-30._cm, TABLE_LENGTH-3._cm, 26.75_in+8.5_cm - TABLE_HEIGHT}},
    {{TABLE_WIDTH+18._in-2._cm, -2._ft+3.5_in+3._cm, 2.5_ft+3.7_cm-TABLE_HEIGHT}},
}};

//w/ ref to paddle facing forward
#define PADDLE_HEIGHT 157.49_mm
constexpr std::array<std::array<double, 3>, 7> paddle_anchorOffsets = {{
    {{59.887_mm, -4._mm, 47.225_mm}},
    {{42.352_mm, -4._mm, -78.264_mm}},
    {{-74.917_mm, -4._mm, -19.694_mm}},
    {{0, -4._mm, PADDLE_HEIGHT/2}},
    {{-42.352_mm, -4._mm, -78.264_mm}},
    {{-59.887_mm, -4._mm, 47.225_mm}},
    {{74.917_mm, -4._mm, -19.694_mm}},
}};

constexpr std::array<std::array<double, 3>, 7> pulley_anchorOffsets_refOri = {0, 1, 0};