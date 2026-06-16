#include "misc/3dRenderer/3dRenderer.h"
#include <cmath>
#include "config/config.h"

int main() {
    viz3d::init(1280, 720, "test");

    float t = 0.f;

    while (viz3d::open()) {
        viz3d::begin();
        //draw axes
        float axlen = 0.25;
        viz3d::line(0, 0, 0, axlen, 0, 0, 1, 0 ,0);
        viz3d::line(0, 0, 0, 0, 0, axlen, 0, 1 ,0);
        viz3d::line(0, 0, 0, 0, axlen, 0, 0, 0 ,1);
        //draw table
        viz3d::grid(0, 0, TABLE_WIDTH, TABLE_LENGTH, 9, 8);
        viz3d::line(0, 0, TABLE_LENGTH, 0, 0, TABLE_LENGTH*2, 0.3f,0.3f,0.3f);
        viz3d::line(TABLE_WIDTH, 0, TABLE_LENGTH, TABLE_WIDTH, 0, TABLE_LENGTH*2, 0.3f,0.3f,0.3f);
        viz3d::line(0, 0, TABLE_LENGTH*2, TABLE_WIDTH, 0, TABLE_LENGTH*2, 0.3f,0.3f,0.3f);
        viz3d::line(0, 0, TABLE_LENGTH, 0, 0.1525, TABLE_LENGTH, 0.3f,0.3f,0.3f);
        viz3d::line(0, 0.1525, TABLE_LENGTH, TABLE_WIDTH, 0.1525, TABLE_LENGTH, 0.3f,0.3f,0.3f);
        viz3d::line(TABLE_WIDTH, 0, TABLE_LENGTH, TABLE_WIDTH, 0.1525, TABLE_LENGTH, 0.3f,0.3f,0.3f);

        //viz3d::axes();

        // rotating circle
        //viz3d::circle(0,1,0, 2, sinf(t),1,cosf(t), 1,0.5f,0);

        // bouncing point traced as a line
        //float y = fabsf(sinf(t)) * 3.f;
        //viz3d::line(0,0,0, 0,y,0, 0,1,1);

        // static quad outline
        //viz3d::quad(3,0,0, 1,0,0, 0,1,0, 0.4f,0.8f,1, false);

        //t += 0.016f;
        viz3d::end();
    }

    viz3d::shutdown();
}