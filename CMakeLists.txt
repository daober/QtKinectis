cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(freenector2 CXX)

OPTION(ENABLE_OPENCL "Enable OpenCL support" ON)
OPTION(ENABLE_OPENGL "Enable OpenGL support" ON)
OPTION(ENABLE_CUDA   "Enable CUDA support" ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

SET(CMAKE_BUILD_TYPE Debug)


find_package(PCL 1.7.1	REQUIRED)
find_package(Boost      REQUIRED)
find_package(freenect2  REQUIRED)
find_package(OpenCV     REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(VTK        REQUIRED)

# run silent
find_package(Eigen3 	QUIET)
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

add_definitions(${Qt5Widgets_DEFINITIONS})


set(SOURCES src/init.cpp src/grabber.cpp src/grabber_impl.cpp src/logger.cpp src/eventlistener.cpp src/io_cloud.cpp )

add_executable(freenector2 ${SOURCES} ${project_FORMS_HEADERS} ${project_HEADERS_MOC} )

target_link_libraries(freenector2 ${OpenCV_LIBS} ${freenect2_LIBRARIES} ${PCL_LIBRARIES} ${OPENGL_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${QT_LIBRARIES} ${VTK_LIBRARIES})
