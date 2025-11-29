// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from huateng_camera_node:srv/SetFloat32.idl
// generated code does not contain a copyright notice

#ifndef HUATENG_CAMERA_NODE__SRV__DETAIL__SET_FLOAT32__STRUCT_H_
#define HUATENG_CAMERA_NODE__SRV__DETAIL__SET_FLOAT32__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetFloat32 in the package huateng_camera_node.
typedef struct huateng_camera_node__srv__SetFloat32_Request
{
  float data;
} huateng_camera_node__srv__SetFloat32_Request;

// Struct for a sequence of huateng_camera_node__srv__SetFloat32_Request.
typedef struct huateng_camera_node__srv__SetFloat32_Request__Sequence
{
  huateng_camera_node__srv__SetFloat32_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} huateng_camera_node__srv__SetFloat32_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetFloat32 in the package huateng_camera_node.
typedef struct huateng_camera_node__srv__SetFloat32_Response
{
  bool success;
} huateng_camera_node__srv__SetFloat32_Response;

// Struct for a sequence of huateng_camera_node__srv__SetFloat32_Response.
typedef struct huateng_camera_node__srv__SetFloat32_Response__Sequence
{
  huateng_camera_node__srv__SetFloat32_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} huateng_camera_node__srv__SetFloat32_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUATENG_CAMERA_NODE__SRV__DETAIL__SET_FLOAT32__STRUCT_H_
