// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from huateng_camera_node:srv/SetFloat32.idl
// generated code does not contain a copyright notice

#ifndef HUATENG_CAMERA_NODE__SRV__DETAIL__SET_FLOAT32__BUILDER_HPP_
#define HUATENG_CAMERA_NODE__SRV__DETAIL__SET_FLOAT32__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "huateng_camera_node/srv/detail/set_float32__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace huateng_camera_node
{

namespace srv
{

namespace builder
{

class Init_SetFloat32_Request_data
{
public:
  Init_SetFloat32_Request_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::huateng_camera_node::srv::SetFloat32_Request data(::huateng_camera_node::srv::SetFloat32_Request::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::huateng_camera_node::srv::SetFloat32_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::huateng_camera_node::srv::SetFloat32_Request>()
{
  return huateng_camera_node::srv::builder::Init_SetFloat32_Request_data();
}

}  // namespace huateng_camera_node


namespace huateng_camera_node
{

namespace srv
{

namespace builder
{

class Init_SetFloat32_Response_success
{
public:
  Init_SetFloat32_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::huateng_camera_node::srv::SetFloat32_Response success(::huateng_camera_node::srv::SetFloat32_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::huateng_camera_node::srv::SetFloat32_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::huateng_camera_node::srv::SetFloat32_Response>()
{
  return huateng_camera_node::srv::builder::Init_SetFloat32_Response_success();
}

}  // namespace huateng_camera_node

#endif  // HUATENG_CAMERA_NODE__SRV__DETAIL__SET_FLOAT32__BUILDER_HPP_
