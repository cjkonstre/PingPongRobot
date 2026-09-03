cmake_minimum_required(VERSION 3.10.0)
cmake_policy(SET CMP0079 NEW)
project(PingPongRobot VERSION 0.1.0 LANGUAGES C CXX)

# ===== Global Settings =====
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

# ===== System / Pre-installed =====
find_package(OpenCV    REQUIRED)
find_package(Boost 1.66 REQUIRED COMPONENTS system)
find_package(Threads   REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Ceres     REQUIRED)
find_package(Python3   REQUIRED COMPONENTS Interpreter Development NumPy)

find_package(httplib CONFIG REQUIRED)
find_package(ruckig CONFIG REQUIRED)

add_library(Ruckig::Ruckig ALIAS ruckig::ruckig)

find_package(glfw3 REQUIRED)
find_package(GLEW  REQUIRED)
find_package(glm   REQUIRED)
find_package(OpenGL REQUIRED)


# ===== FetchContent =====
FetchContent_Declare(nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)
# ===== ImGui + ImPlot =====
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.5-docking
)
FetchContent_Declare(implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG        v0.16
)
FetchContent_MakeAvailable(imgui implot)

add_library(imgui_implot STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)
target_include_directories(imgui_implot PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${implot_SOURCE_DIR}
)
target_compile_definitions(imgui_implot PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLEW)
target_link_libraries(imgui_implot PUBLIC glfw GLEW::GLEW OpenGL::GL)

# ===== Interface Targets =====
add_library(matplotlibcpp INTERFACE)
target_include_directories(matplotlibcpp INTERFACE /usr/local/include/matplotlib-cpp)
target_link_libraries(matplotlibcpp INTERFACE Python3::Python Python3::NumPy)

# ===== Lib Vars =====
set(UTIL_LIBS
matplotlibcpp
nlohmann_json::nlohmann_json
glfw
GLEW::GLEW
glm::glm
OpenGL::GL
imgui_implot
)

set(VISION_LIBS ${OpenCV_LIBS})#arducam_evk_sdk::arducam_evk_sdk)
set(KIN_LIBS Threads::Threads Ruckig::Ruckig Eigen3::Eigen Boost::system)

# ===== Libraries =====
file(GLOB_RECURSE LIB_SOURCES "${CMAKE_SOURCE_DIR}/lib/*.cpp")
add_library(robotlibs STATIC ${LIB_SOURCES})
target_include_directories(robotlibs PUBLIC ${CMAKE_SOURCE_DIR}/lib)
target_link_libraries(robotlibs PUBLIC ${UTIL_LIBS} ${KIN_LIBS} ${OpenCV_LIBS})

file(GLOB_RECURSE CORE_SOURCES "${CMAKE_SOURCE_DIR}/core/*.cpp")
add_library(core STATIC ${CORE_SOURCES})
target_include_directories(core PUBLIC ${CMAKE_SOURCE_DIR}/core)
target_link_libraries(core PUBLIC robotlibs nlohmann_json::nlohmann_json ${UTIL_LIBS} ${KIN_LIBS} ${OpenCV_LIBS})


# EXECS
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ===== Helper function =====
include(CMakeParseArguments)
function(add_execs DIR)
  # Usage: add_execs(<dir> PREFIX <prefix> LIBS <lib1> <lib2> ...)
  set(options)
  set(oneValueArgs PREFIX)
  set(multiValueArgs LIBS)
  cmake_parse_arguments(AEF "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  file(GLOB_RECURSE SRC_FILES "${DIR}/*.cpp")
  foreach(SRC_FILE ${SRC_FILES})
    get_filename_component(NAME_WE ${SRC_FILE} NAME_WE)
    set(TGT "${AEF_PREFIX}${NAME_WE}")
    add_executable(${TGT} ${SRC_FILE})
    target_link_libraries(${TGT} PRIVATE ${AEF_LIBS})
  endforeach()
endfunction()

# ===== Executables =====

# Everything in src/ with Ruckig + Eigen + OpenCV
add_execs("${CMAKE_SOURCE_DIR}/src"
  PREFIX ""
  LIBS robotlibs core ${UTIL_LIBS} ${KIN_LIBS} ${OpenCV_LIBS} ${VISION_LIBS}
)

# Everything in tools/ with Ruckig + Eigen + OpenCV
add_execs("${CMAKE_SOURCE_DIR}/tools"
  PREFIX ""
  LIBS robotlibs core Eigen3::Eigen Ceres::ceres ${UTIL_LIBS} ${KIN_LIBS} ${OpenCV_LIBS} ${VISION_LIBS}
)

#tets
add_execs("${CMAKE_SOURCE_DIR}/tests/kinematics"
  PREFIX "test_"
  LIBS robotlibs core ${UTIL_LIBS} ${KIN_LIBS} pthread
)

add_execs("${CMAKE_SOURCE_DIR}/tests/vision"
  PREFIX "test_"
  LIBS robotlibs core ${UTIL_LIBS} ${VISION_LIBS} ${KIN_LIBS}
)

# ===== Testing =====
include(CTest)
enable_testing()