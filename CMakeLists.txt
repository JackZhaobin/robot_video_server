cmake_minimum_required(VERSION 3.8)
project(robot_video_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g -O0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(robot_interface REQUIRED)

# 查找 CUDA
find_package(CUDA REQUIRED)

link_directories(/usr/local/cuda-12.2/lib64)
link_directories(/usr/lib/aarch64-linux-gnu)
link_directories(/usr/lib/aarch64-linux-gnu/tegra)
link_directories(/usr/lib/aarch64-linux-gnu/tegra-egl)
link_directories(/usr/lib/aarch64-linux-gnu/nvidia)
link_directories(${PROJECT_SOURCE_DIR}/3rdparty/rtspserver/lib)
link_directories(${PROJECT_SOURCE_DIR}/3rdparty/zdepth/lib)
link_directories(${PROJECT_SOURCE_DIR}/3rdparty/dds/lib)
link_directories(${PROJECT_SOURCE_DIR}/3rdparty/yllog/lib)
link_directories(${PROJECT_SOURCE_DIR}/3rdparty/orbbecsdk/lib/arm64)

include_directories(
    include
    nvidia/include
    3rdparty/rtspserver/include
    3rdparty/zdepth/include
    3rdparty/dds/include
    3rdparty/yllog/include
    3rdparty/orbbecsdk/include
    /usr/include/drm
    /usr/include/opencv4
    /usr/local/cuda-12.2/include
    /usr/lib/aarch64-linux-gnu
    /usr/lib/aarch64-linux-gnu/tegra
    /usr/lib/aarch64-linux-gnu/tegra-egl
    /usr/lib/aarch64-linux-gnu/nvidia
)

file(GLOB SOURCES "src/*.cpp" "nvidia/common/classes/*.cpp")

add_executable(robot_video_server ${SOURCES})

target_link_libraries(robot_video_server 
    ${OpenCV_LIBS}
    # ${CUDA_LIBRARIES}
    nvv4l2
    nvvideo
    nvmedia
    nvbufsurface
    nvbufsurftransform
    nvscibuf
    nvscicommon
    nvjpeg
    nvosd
    drm
    cuda
    cudart
    EGL
    GLESv2
    X11
    rtspserver
    OrbbecSDK
    yhdds
    yllog
    zdepth
    zstd
    pthread)

ament_target_dependencies(
    robot_video_server
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
    image_transport
)

install(TARGETS robot_video_server
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 3rdparty/orbbecsdk/lib/arm64/
    DESTINATION lib/
)

install(DIRECTORY 3rdparty/rtspserver/lib/
    DESTINATION lib/
)

install(DIRECTORY 3rdparty/dds/lib/
    DESTINATION lib/
)

install(DIRECTORY 3rdparty/yllog/lib/
    DESTINATION lib/
)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
