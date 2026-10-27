cmake_minimum_required(VERSION 3.5)
project(im_mesh)

SET(CMAKE_BUILD_TYPE "Release")

ADD_COMPILE_OPTIONS(-std=c++17 )

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")


message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("core for MP:  4")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message("core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(CGAL REQUIRED)
set(Sophus_LIBRARIES libSophus.so)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
  # "msg/States.msg"
)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(include/ikd-Tree/)
include_directories(src/meshing/)
include_directories(src/meshing/r3live/)
include_directories(src/meshing/kd_tree_dev/)
include_directories(src/meshing/rgb_map/)
include_directories(src/tools/)
include_directories(src/tools/imgui)
include_directories(src/tools/shader)
include_directories(src/tools/openGL_libs)
include_directories(src)


set(FONTPATH ${CMAKE_CURRENT_SOURCE_DIR}/font/)
add_definitions(-DFONT_DIR=\"${FONTPATH}/\")

set(SHADERPATH ${CMAKE_CURRENT_SOURCE_DIR}/src/shader/)
add_definitions(-DSHADER_DIR=\"${SHADERPATH}/\")

add_executable(ImMesh_mapping
  src/ImMesh_node.cpp
  src/ImMesh_mesh_reconstruction.cpp
  include/ikd-Tree/ikd_Tree.cpp
  src/voxel_mapping.cpp
  src/voxel_mapping_common.cpp
  src/voxel_loc.cpp
  src/preprocess.cpp
  src/IMU_Processing.cpp

  ./src/meshing/mesh_rec_display.cpp
  ./src/meshing/mesh_rec_geometry.cpp
  ./src/meshing/r3live/triangle.cpp
  ./src/meshing/r3live/image_frame.cpp
  ./src/meshing/r3live/pointcloud_rgbd.cpp
  ./src/tools/openGL_libs/openGL_camera.cpp
  ./src/tools/openGL_libs/openGL_camera_view.cpp
  ./src/tools/openGL_libs/glad.c
  
  ./src/tools/imgui/imgui.cpp
  ./src/tools/imgui/imgui_draw.cpp
  ./src/tools/imgui/imgui_impl_glfw.cpp
  ./src/tools/imgui/imgui_impl_opengl3.cpp
  ./src/tools/imgui/imgui_tables.cpp
  ./src/tools/imgui/imgui_widgets.cpp
)
target_link_libraries(ImMesh_mapping 
                      ${PCL_LIBRARIES} 
                      ${PYTHON_LIBRARIES}
                      CGAL::CGAL
                      tbb
                      ${OpenCV_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${Boost_SYSTEM_LIBRARIES}
                      ${Boost_FILESYSTEM_LIBRARY}
                      ${Boost_SERIALIZATION_LIBRARY} # serialization
                      ${CERES_LIBRARIES}
                      GL GLU glfw X11 Xrandr pthread Xi dl Xinerama Xcursor)

ament_target_dependencies(ImMesh_mapping nav_msgs rclcpp tf2 tf2_ros tf2_eigen visualization_msgs PCL cv_bridge std_msgs geometry_msgs livox_ros_driver2 sensor_msgs pcl_conversions cv_bridge)

install(TARGETS
  ImMesh_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config rviz_cfg launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp Eigen detection_msgs livox_ros_driver2)
ament_package()
