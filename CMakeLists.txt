cmake_minimum_required(VERSION 3.8)
project(huateng_camera_node)

# ================= Compiler options =================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================= Dependencies =================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

# ==== Message/service generation ====
find_package(rosidl_generator_c REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# ================= Interface generation =================
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetFloat32.srv"
  DEPENDENCIES builtin_interfaces
)

# Export runtime so executables can use generated headers
ament_export_dependencies(
  rosidl_default_runtime
  rosidl_generator_c
  rosidl_generator_cpp
  rosidl_typesupport_cpp
)

# ================= Include directories =================
# 👇 Adiciona o diretório de includes gerados em build/
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# ================= Build node executable =================
add_executable(huateng_camera_node_exec src/huateng_camera_node.cpp)

ament_target_dependencies(huateng_camera_node_exec
  rclcpp sensor_msgs std_msgs cv_bridge image_transport
  OpenCV camera_info_manager diagnostic_msgs rosidl_default_runtime
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(huateng_camera_node_exec
  ${OpenCV_LIBS}
  /usr/lib/libMVSDK.so
  ${cpp_typesupport_target}
)


# ================= Dependências de geração =================
add_dependencies(huateng_camera_node_exec
  ${PROJECT_NAME}__cpp
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp
  ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp
)

# ================= Instalação =================
install(TARGETS huateng_camera_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
