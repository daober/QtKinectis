cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(freenector2 CXX)

OPTION(ENABLE_OPENCL "Enable OpenCL support" ON)
OPTION(ENABLE_OPENGL "Enable OpenGL support" ON)
OPTION(ENABLE_CUDA   "Enable CUDA support" ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_BUILD_TYPE Debug)
SET(Boost_USE_MULTITHREAD ON)

find_package(PCL 1.7.1	REQUIRED)
find_package(Boost      REQUIRED)

# needed for boost asio (networking)
find_package(Boost REQUIRED COMPONENTS system thread regex program_options)

find_package(freenect2  REQUIRED)
find_package(OpenCV     REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(VTK        REQUIRED)

set(freenector2Lib_src ${PROJECT_SOURCE_DIR}/src/qtvisualizer.cpp)
set(freenector2Lib_hdr ${PROJECT_SOURCE_DIR}/include/qtvisualizer.h)
set(freenector2Lib_ui ${PROJECT_SOURCE_DIR}/ui/qtvisualizer.ui)

set(freenector2Bin_src ${PROJECT_SOURCE_DIR}/src/init.cpp)

qt5_wrap_cpp(freenector2_hdr_moc ${freenector2Lib_hdr})
qt5_wrap_ui(freenector2_ui_moc ${freenector2Lib_ui})

# run silent
#find_package(Eigen3 	QUIET)
find_package(Eigen3 REQUIRED)
# lines underneath needed for OSX + brew package manager installation formulae for opencv3 and eigen3
if(NOT EIGEN_FOUND)
    # Fallback to cmake_modules
    find_package(Eigen REQUIRED)
    include (${PROJ_SOURCE_DIR}/modules/FindEigen3.cmake)
    set(EIGEN_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})

    # necessary for eigen lib installed via brew
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
    # map additional variables to EIGEN3_ prefix as you wish

endif()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIR})


pkg_search_module(GLFW 	REQUIRED glfw3)

if(ENABLE_OPENCL)
    find_package(OpenCL)
endif(ENABLE_OPENCL)


if(ENABLE_OPENGL)
    find_package(OpenGL)
    pkg_search_module(GLFW 	REQUIRED glfw3)
    include_directories(${GLFW_INCLUDE_DIRS})
    include_directories(${OPENGL_INCLUDE_DIR})
endif(ENABLE_OPENGL)


add_definitions(${PCL_DEFINITIONS})

link_directories(${PCL_LIBRARY_DIRS})

# include own include directory
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${freenect2_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_BINARY_DIR})

add_definitions(${Qt5Widgets_DEFINITIONS})

add_library(freenector2Lib SHARED 
                        ${freenector2Lib_src}
                        ${freenector2Lib_hdr_moc}
                        ${freenector2_ui_moc}
            )



set(SOURCES src/init.cpp src/iocloud.cpp src/grabber.cpp src/grabber_impl.cpp src/logger.cpp src/eventlistener.cpp src/argparser.cpp src/resampling.cpp src/outlierremoval.cpp src/compress.cpp src/network.cpp src/registration.cpp src/convertcloud.cpp)

add_executable(freenector2 ${SOURCES} include/iocloud.hpp include/grabber.hpp include/grabber_impl.hpp include/logger.hpp include/eventlistener.hpp include/argparser.hpp include/resampling.hpp include/outlierremoval.hpp include/compress.hpp include/network.hpp include/registration.hpp include/convertcloud.hpp ${freenector2Bin_src})

target_link_libraries(freenector2 ${OpenCV_LIBS} ${freenect2_LIBRARIES} ${PCL_LIBRARIES} ${OPENGL_LIBRARIES} ${VTK_LIBRARIES} ${Boost_LIBRARIES} Qt5::Widgets)

