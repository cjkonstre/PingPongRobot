//unchanging dimensions

#pragma once

#include <array>

constexpr double operator ""_m(long double val) {return val;}
constexpr double operator ""_cm(long double val) {return val/100;}
constexpr double operator ""_mm(long double val) {return val/1000;}
constexpr double operator ""_ft(long double val) {return val*0.3048;}
constexpr double operator ""_in(long double val) {return val/12*0.3048;}

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