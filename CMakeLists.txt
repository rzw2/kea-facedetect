cmake_minimum_required(VERSION 3.4 FATAL_ERROR)

project(kea_opencv_viewer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)

# The install directory of the ToF library
# In this case I extracted it in my Downloads folder 
set(tof_DIR "/home/refael/Downloads/artifacts/tof-internal-linux-x86_64/lib/cmake/tof")
# This is where I built OpenCV, due to my interesting build setup
set(OpenCV_DIR "/home/refael/Documents/support/opencv/build")

find_package(tof REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(OpenMP REQUIRED)

# Face detection package ... 
set(facedetection_DIR "/home/refael/Documents/support/libfacedetection/build")
find_package(facedetection REQUIRED)

set(kea_face_detection_srcs src/kea_face_detection.cpp src/colormap.cpp)

include_directories(${OpenCV_INCLUDE_DIRS}) 
include_directories("/home/refael/Documents/support/libfacedetection/build")

add_executable(kea_face_detection ${kea_face_detection_srcs})


target_link_libraries(kea_face_detection PRIVATE OpenMP::OpenMP_CXX tof::tof
                                                ${OpenCV_LIBS} facedetection)

install(TARGETS kea_face_detection DESTINATION ${CMAKE_INSTALL_BINDIR})
