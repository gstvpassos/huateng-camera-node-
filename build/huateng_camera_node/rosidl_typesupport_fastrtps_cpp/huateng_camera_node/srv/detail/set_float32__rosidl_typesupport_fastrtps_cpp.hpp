// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from huateng_camera_node:srv/SetFloat32.idl
// generated code does not contain a copyright notice

#ifndef HUATENG_CAMERA_NODE__SRV__DETAIL__SET_FLOAT32__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define HUATENG_CAMERA_NODE__SRV__DETAIL__SET_FLOAT32__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "huateng_camera_node/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "huateng_camera_node/srv/detail/set_float32__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace huateng_camera_node
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_huateng_camera_node
cdr_serialize(
  const huateng_camera_node::srv::SetFloat32_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_huateng_camera_node
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  huateng_camera_node::srv::SetFloat32_Request & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_huateng_camera_node
get_serialized_size(
  const huateng_camera_node::srv::SetFloat32_Request & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_huateng_camera_node
max_serialized_size_SetFloat32_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace huateng_camera_node

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_huateng_camera_node
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, huateng_camera_node, srv, SetFloat32_Request)();

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "huateng_camera_node/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
// already included above
// #include "huateng_camera_node/srv/detail/set_float32__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// already included above
// #include "fastcdr/Cdr.h"

namespace huateng_camera_node
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_huateng_camera_node
cdr_serialize(
  const huateng_camera_node::srv::SetFloat32_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_huateng_camera_node
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  huateng_camera_node::srv::SetFloat32_Response & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_huateng_camera_node
get_serialized_size(
  const huateng_camera_node::srv::SetFloat32_Response & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_huateng_camera_node
max_serialized_size_SetFloat32_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace huateng_camera_node

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_huateng_camera_node
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, huateng_camera_node, srv, SetFloat32_Response)();

#ifdef __cplusplus
}
#endif

#include "rmw/types.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "huateng_camera_node/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_huateng_camera_node
const rosidl_service_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, huateng_camera_node, srv, SetFloat32)();

#ifdef __cplusplus
}
#endif

#endif  // HUATENG_CAMERA_NODE__SRV__DETAIL__SET_FLOAT32__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
