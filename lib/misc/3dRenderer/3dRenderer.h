#pragma once

//adapted into general renderer
#include <string>


//yeah yeah coded by claude who cares
//lil' 3d renderer, should be super low latency
//primitives live hre
namespace viz3d {
    void init(int width, int height, const char* title);
    void shutdown();
    bool open();
    void begin();
    void end();
    void line(float x0,float y0,float z0, float x1,float y1,float z1, float r,float g,float b);
    void circle(float cx,float cy,float cz, float radius, float nx,float ny,float nz, float r,float g,float b, int segments=32);
    void sphere(float cx, float cy, float cz, float radius,
            float r, float g, float b,
            int rings = 12, int segments = 64) ;
    void quad(float cx,float cy,float cz, float hx,float hy,float hz, float ux,float uy,float uz, float r,float g,float b, bool filled=false);
    void grid(float size=10.f, int divs=20);
    void grid(float x, float y, float dx, float dy, int subdivx, int subdivy);
    void axes(float len=1.f);

    void startRec(const std::string& path, double fps = 60.0);
    void stopRec();
    bool isRecording();
}