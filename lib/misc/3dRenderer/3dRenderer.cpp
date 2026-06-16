#include "misc/3dRenderer/3dRenderer.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "imgui.h"
#include "implot.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include <vector>
#include <cmath>
#include <stdexcept>

namespace viz3d {
// ── Embedded shaders ──────────────────────────────────────────────────────
static const char* kVert = R"(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aCol;
uniform mat4 uMVP;
out vec3 vCol;
void main() { vCol = aCol; gl_Position = uMVP * vec4(aPos,1.0); }
)";
static const char* kFrag = R"(
#version 330 core
in vec3 vCol; out vec4 FragColor;
void main() { FragColor = vec4(vCol,1.0); }
)";

// ── Internal state ────────────────────────────────────────────────────────
struct Vert { float x,y,z, r,g,b; };

static GLFWwindow*        s_win      = nullptr;
static GLuint             s_prog     = 0;
static GLuint             s_vao[2]   = {};   // [0]=lines [1]=tris
static GLuint             s_vbo[2]   = {};
static GLint              s_uMVP     = -1;
static std::vector<Vert>  s_lines, s_tris;

// Camera
static float s_yaw = 45.f, s_pitch = 30.f, s_dist = 10.f;
static glm::vec3 s_target = {0,0,0};
static double s_mx = 0, s_my = 0;
static bool   s_lbtn = false, s_mbtn = false;

// ── Callbacks ─────────────────────────────────────────────────────────────
static void on_scroll(GLFWwindow*, double, double dy) {
    s_dist = fmaxf(0.1f, s_dist - (float)dy * s_dist * 0.1f);
}
static void on_button(GLFWwindow* w, int btn, int action, int) {
    if (btn == GLFW_MOUSE_BUTTON_LEFT)   s_lbtn = action == GLFW_PRESS;
    if (btn == GLFW_MOUSE_BUTTON_MIDDLE) s_mbtn = action == GLFW_PRESS;
    if (action == GLFW_PRESS) glfwGetCursorPos(w, &s_mx, &s_my);
}
static void on_cursor(GLFWwindow*, double x, double y) {
    float dx = (float)(x - s_mx), dy = (float)(y - s_my);
    s_mx = x; s_my = y;
    if (s_lbtn) {
        s_yaw   += dx * 0.4f;
        s_pitch  = fmaxf(-89.f, fminf(89.f, s_pitch + dy * 0.4f));
    }
    if (s_mbtn) {
        float scale = s_dist * 0.001f;
        float yaw   = glm::radians(s_yaw);
        glm::vec3 right = { cosf(yaw), 0, sinf(yaw) };
        glm::vec3 up    = { 0, 1, 0 };
        s_target -= right * dx * scale;
        s_target += up    * dy * scale;
    }
}

// ── Init / shutdown ───────────────────────────────────────────────────────
void init(int w, int h, const char* title) {
    if (!glfwInit()) throw std::runtime_error("glfwInit failed");
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    glfwWindowHint(GLFW_SAMPLES, 4);

    s_win = glfwCreateWindow(w, h, title, nullptr, nullptr);
    if (!s_win) { glfwTerminate(); throw std::runtime_error("glfwCreateWindow failed"); }

    glfwSetScrollCallback    (s_win, on_scroll);
    glfwSetMouseButtonCallback(s_win, on_button);
    glfwSetCursorPosCallback (s_win, on_cursor);
    glfwMakeContextCurrent   (s_win);
    glfwSwapInterval(0); // no vsync = lowest latency

    glewExperimental = GL_TRUE;
    glewInit();

    // compile shader
    auto compile = [](GLenum t, const char* src) {
        GLuint s = glCreateShader(t);
        glShaderSource(s, 1, &src, nullptr);
        glCompileShader(s);
        return s;
    };
    GLuint vert = compile(GL_VERTEX_SHADER, kVert);
    GLuint frag = compile(GL_FRAGMENT_SHADER, kFrag);
    s_prog = glCreateProgram();
    glAttachShader(s_prog, vert); glAttachShader(s_prog, frag);
    glLinkProgram(s_prog);
    glDeleteShader(vert); glDeleteShader(frag);
    s_uMVP = glGetUniformLocation(s_prog, "uMVP");

    // VAO/VBO for lines and tris
    glGenVertexArrays(2, s_vao);
    glGenBuffers(2, s_vbo);
    for (int i = 0; i < 2; i++) {
        glBindVertexArray(s_vao[i]);
        glBindBuffer(GL_ARRAY_BUFFER, s_vbo[i]);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vert), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vert), (void*)(3*sizeof(float)));
    }
    glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glLineWidth(1.5f);

    
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(s_win, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

void shutdown() {
    glDeleteVertexArrays(2, s_vao);
    glDeleteBuffers(2, s_vbo);
    glDeleteProgram(s_prog);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(s_win);
    glfwTerminate();
    s_win = nullptr;
}

// ── Frame ─────────────────────────────────────────────────────────────────
bool open() { return s_win && !glfwWindowShouldClose(s_win); }

void begin() {
    glfwPollEvents();
    int w, h; glfwGetFramebufferSize(s_win, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(0.08f, 0.08f, 0.10f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    s_lines.clear();
    s_tris.clear();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void end() {
    // build MVP
    float yaw   = glm::radians(s_yaw);
    float pitch = glm::radians(s_pitch);
    glm::vec3 dir = { cosf(pitch)*cosf(yaw), sinf(pitch), cosf(pitch)*sinf(yaw) };
    glm::vec3 eye = s_target + dir * s_dist;

    int w, h; glfwGetFramebufferSize(s_win, &w, &h);
    float aspect = h > 0 ? (float)w / (float)h : 1.f;
    glm::mat4 mvp = glm::perspective(glm::radians(45.f), aspect, 0.01f, 1000.f)
                  * glm::lookAt(eye, s_target, {0,1,0});

    glUseProgram(s_prog);
    glUniformMatrix4fv(s_uMVP, 1, GL_FALSE, glm::value_ptr(mvp));

    // upload + draw lines
    if (!s_lines.empty()) {
        glBindVertexArray(s_vao[0]);
        glBindBuffer(GL_ARRAY_BUFFER, s_vbo[0]);
        glBufferData(GL_ARRAY_BUFFER, s_lines.size()*sizeof(Vert), s_lines.data(), GL_STREAM_DRAW);
        glDrawArrays(GL_LINES, 0, (GLsizei)s_lines.size());
    }
    // upload + draw tris
    if (!s_tris.empty()) {
        glBindVertexArray(s_vao[1]);
        glBindBuffer(GL_ARRAY_BUFFER, s_vbo[1]);
        glBufferData(GL_ARRAY_BUFFER, s_tris.size()*sizeof(Vert), s_tris.data(), GL_STREAM_DRAW);
        glDrawArrays(GL_TRIANGLES, 0, (GLsizei)s_tris.size());
    }
    glBindVertexArray(0);

    glBindVertexArray(0);

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(s_win);
}

// ── Primitives ────────────────────────────────────────────────────────────
void line(float x0,float y0,float z0, float x1,float y1,float z1,
              float r, float g, float b) {
    s_lines.push_back({x0,y0,z0, r,g,b});
    s_lines.push_back({x1,y1,z1, r,g,b});
}

void circle(float cx,float cy,float cz, float radius,
                float nx,float ny,float nz,
                float r,float g,float b, int segments) {
    glm::vec3 n = glm::normalize(glm::vec3{nx,ny,nz});
    glm::vec3 t = glm::normalize(glm::cross(n, fabsf(n.x) < 0.9f ? glm::vec3{1,0,0} : glm::vec3{0,1,0}));
    glm::vec3 u = glm::cross(n, t);
    float step = 2.f * (float)M_PI / segments;
    for (int i = 0; i < segments; i++) {
        float a0 = i * step, a1 = a0 + step;
        glm::vec3 p0 = glm::vec3{cx,cy,cz} + (t*cosf(a0) + u*sinf(a0)) * radius;
        glm::vec3 p1 = glm::vec3{cx,cy,cz} + (t*cosf(a1) + u*sinf(a1)) * radius;
        s_lines.push_back({p0.x,p0.y,p0.z, r,g,b});
        s_lines.push_back({p1.x,p1.y,p1.z, r,g,b});
    }
}

void quad(float cx,float cy,float cz,
              float hx,float hy,float hz,
              float ux,float uy,float uz,
              float r,float g,float b, bool filled) {
    glm::vec3 c{cx,cy,cz}, he{hx,hy,hz}, ue{ux,uy,uz};
    glm::vec3 v0=c-he-ue, v1=c+he-ue, v2=c+he+ue, v3=c-he+ue;
    if (filled) {
        s_tris.push_back({v0.x,v0.y,v0.z, r,g,b});
        s_tris.push_back({v1.x,v1.y,v1.z, r,g,b});
        s_tris.push_back({v2.x,v2.y,v2.z, r,g,b});
        s_tris.push_back({v0.x,v0.y,v0.z, r,g,b});
        s_tris.push_back({v2.x,v2.y,v2.z, r,g,b});
        s_tris.push_back({v3.x,v3.y,v3.z, r,g,b});
    } else {
        line(v0.x,v0.y,v0.z, v1.x,v1.y,v1.z, r,g,b);
        line(v1.x,v1.y,v1.z, v2.x,v2.y,v2.z, r,g,b);
        line(v2.x,v2.y,v2.z, v3.x,v3.y,v3.z, r,g,b);
        line(v3.x,v3.y,v3.z, v0.x,v0.y,v0.z, r,g,b);
    }
}

void grid(float size, int divs) {
    float half = size * 0.5f, step = size / divs;
    for (int i = 0; i <= divs; i++) {
        float t = -half + i * step;
        line(t,0,-half, t,0, half, 0.3f,0.3f,0.3f);
        line(-half,0,t,  half,0,t,  0.3f,0.3f,0.3f);
    }
}

void grid(float x, float y, float dx, float dy, int subdivx, int subdivy) {
    for (int i=0; i<= subdivx; i++) line(i*dx/subdivx, 0, 0, i*dx/subdivx, 0, dy, 0.3f,0.3f,0.3f);
    for (int i=0; i<= subdivy; i++) line(0, 0, i*dy/subdivy, dx, 0, i*dy/subdivy, 0.3f,0.3f,0.3f);
}

void axes(float len) {
    line(0,0,0, len,0,0, 1,0.2f,0.2f);
    line(0,0,0, 0,len,0, 0.2f,1,0.2f);
    line(0,0,0, 0,0,len, 0.2f,0.5f,1);
}

}