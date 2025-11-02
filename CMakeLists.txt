cmake_minimum_required(VERSION 3.10.0)
project(PingPongRobot VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ===== Dependencies =====
find_package(OpenCV REQUIRED)
find_package(Boost 1.66 REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)


# === Ruckig ===
# Disable Ruckig’s own examples/tests (we only want the library)
set(RUCKIG_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(RUCKIG_BUILD_TESTS OFF CACHE BOOL "" FORCE)

# Add the Ruckig subdirectory (it has its own CMakeLists.txt)
add_subdirectory(${CMAKE_SOURCE_DIR}/lib/kinematics/ruckig)

# If your compiler doesn't support <format>, force Ruckig to use {fmt}
# (Ruckig’s CMake will detect std::format if available, but this override is safe)
#find_package(fmt REQUIRED)
#target_compile_definitions(ruckig PUBLIC RUCKIG_USE_FMTLIB=1)
#target_link_libraries(ruckig PUBLIC fmt::fmt)


# --- Eigen as header-only
add_library(eigen INTERFACE)
target_include_directories(eigen INTERFACE
    ${CMAKE_SOURCE_DIR}/lib/kinematics/eigen-3.4.0
)

# ImGui
#add_library(imgui STATIC
#    lib/utils/imgui/imgui.cpp
#    lib/utils/imgui/imgui_draw.cpp
#    lib/utils/imgui/imgui_tables.cpp
#    lib/utils/imgui/imgui_widgets.cpp
#    lib/utils/imgui/backends/imgui_impl_glfw.cpp
#    lib/utils/imgui/backends/imgui_impl_opengl3.cpp
#)
#target_include_directories(imgui PUBLIC
#    ${CMAKE_SOURCE_DIR}/lib/utils/imgui
#    ${CMAKE_SOURCE_DIR}/lib/utils/imgui/backends
#)

# ImPlot
#add_library(implot STATIC
#    lib/utils/implot/implot.cpp
#    lib/utils/implot/implot_items.cpp
#)
#target_include_directories(implot PUBLIC
#    ${CMAKE_SOURCE_DIR}/lib/utils/implot
#)
#target_link_libraries(implot PUBLIC imgui)

#find_package(OpenGL REQUIRED)
#find_package(glfw3 REQUIRED)

#target_link_libraries(imgui PUBLIC OpenGL::GL glfw)

#matplotlib
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

add_library(matplotlibcpp INTERFACE)
target_include_directories(matplotlibcpp INTERFACE /usr/local/include/matplotlib-cpp)
target_link_libraries(matplotlibcpp INTERFACE Python3::Python Python3::NumPy)

#arducam
#find_package(arducam_evk_sdk REQUIRED)

# --- Common include dirs (for OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS},
                    ${CMAKE_SOURCE_DIR}/lib)

# --- Put all executables in build/bin
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

# Utils (plotting, etc.)
set(UTIL_LIBS matplotlibcpp) #set(UTIL_LIBS implot imgui OpenGL::GL glfw matplotlibcpp)
set(VISION_LIBS ${OpenCV_LIBS} )#arducam_evk_sdk::arducam_evk_sdk)
set(KIN_LIBS Threads::Threads ruckig eigen Boost::system)

# ===== Executables =====

# Everything in src/ with Ruckig + Eigen + OpenCV
add_execs("${CMAKE_SOURCE_DIR}/src"
  PREFIX ""
  LIBS ${UTIL_LIBS} ${KIN_LIBS} ${OpenCV_LIBS}
)

# Everything in tools/ with Ruckig + Eigen + OpenCV
add_execs("${CMAKE_SOURCE_DIR}/tools"
  PREFIX ""
  LIBS ${UTIL_LIBS} ${KIN_LIBS} ${OpenCV_LIBS}
)

#tets

add_execs("${CMAKE_SOURCE_DIR}/tests/kinematics"
  PREFIX "test_"
  LIBS ${UTIL_LIBS} ${KIN_LIBS} pthread
)

add_execs("${CMAKE_SOURCE_DIR}/tests/vision"
  PREFIX "test_"
  LIBS ${UTIL_LIBS} ${VISION_LIBS}
)

# ===== Testing =====
include(CTest)
enable_testing()