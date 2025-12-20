cmake_minimum_required(VERSION 3.10.0)
cmake_policy(SET CMP0079 NEW)
project(PingPongRobot VERSION 0.1.0 LANGUAGES C CXX)

include(FetchContent)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ===== Dependencies =====
find_package(OpenCV REQUIRED)
find_package(Boost 1.66 REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
# Ensure Eigen headers are available system-wide
if(TARGET Eigen3::Eigen)
    get_target_property(_eigen_inc Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
    if(NOT _eigen_inc)
        # On some systems Eigen3::Eigen has no include dirs; add manually
        message(STATUS "Manually adding Eigen include path: ${EIGEN3_INCLUDE_DIRS}")
        target_include_directories(core INTERFACE ${EIGEN3_INCLUDE_DIRS})
        target_include_directories(robotlibs PUBLIC ${EIGEN3_INCLUDE_DIRS})
    endif()
else()
    include_directories(${EIGEN3_INCLUDE_DIRS})
endif()
find_package(Ruckig REQUIRED)
if(NOT TARGET Ruckig::Ruckig)
    add_library(Ruckig::Ruckig SHARED IMPORTED)
    set_target_properties(Ruckig::Ruckig PROPERTIES
        IMPORTED_LOCATION "/usr/local/lib/libruckig.so"
        INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include"
    )
endif()

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED) #matplotlib
add_library(matplotlibcpp INTERFACE)
target_include_directories(matplotlibcpp INTERFACE /usr/local/include/matplotlib-cpp)
target_link_libraries(matplotlibcpp INTERFACE Python3::Python Python3::NumPy)

FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

#lib vars
set(UTIL_LIBS matplotlibcpp) #set(UTIL_LIBS implot imgui OpenGL::GL glfw matplotlibcpp)
set(VISION_LIBS ${OpenCV_LIBS} )#arducam_evk_sdk::arducam_evk_sdk)
set(KIN_LIBS Threads::Threads Ruckig::Ruckig Eigen3::Eigen Boost::system)

# ===== Libraries =====
file(GLOB_RECURSE LIB_SOURCES "${CMAKE_SOURCE_DIR}/lib/*.cpp")
add_library(robotlibs STATIC ${LIB_SOURCES})
target_include_directories(robotlibs PUBLIC ${CMAKE_SOURCE_DIR}/lib)
target_link_libraries(robotlibs PUBLIC Eigen3::Eigen Ruckig::Ruckig Boost::system Threads::Threads)

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
  LIBS robotlibs core ${UTIL_LIBS} ${KIN_LIBS} ${OpenCV_LIBS}
)

# Everything in tools/ with Ruckig + Eigen + OpenCV
add_execs("${CMAKE_SOURCE_DIR}/tools"
  PREFIX ""
  LIBS robotlibs core ${UTIL_LIBS} ${KIN_LIBS} ${OpenCV_LIBS}
)

#tets

add_execs("${CMAKE_SOURCE_DIR}/tests/kinematics"
  PREFIX "test_"
  LIBS robotlibs ${UTIL_LIBS} ${KIN_LIBS} pthread
)

add_execs("${CMAKE_SOURCE_DIR}/tests/vision"
  PREFIX "test_"
  LIBS robotlibs ${UTIL_LIBS} ${VISION_LIBS}
)

# ===== Testing =====
include(CTest)
enable_testing()