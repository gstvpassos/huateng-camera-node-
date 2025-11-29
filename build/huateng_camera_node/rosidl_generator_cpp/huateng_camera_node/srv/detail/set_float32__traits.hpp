// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from huateng_camera_node:srv/SetFloat32.idl
// generated code does not contain a copyright notice

#ifndef HUATENG_CAMERA_NODE__SRV__DETAIL__SET_FLOAT32__TRAITS_HPP_
#define HUATENG_CAMERA_NODE__SRV__DETAIL__SET_FLOAT32__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "huateng_camera_node/srv/detail/set_float32__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace huateng_camera_node
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetFloat32_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetFloat32_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetFloat32_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace huateng_camera_node

namespace rosidl_generator_traits
{

[[deprecated("use huateng_camera_node::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const huateng_camera_node::srv::SetFloat32_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  huateng_camera_node::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use huateng_camera_node::srv::to_yaml() instead")]]
inline std::string to_yaml(const huateng_camera_node::srv::SetFloat32_Request & msg)
{
  return huateng_camera_node::srv::to_yaml(msg);
}

template<>
inline const char * data_type<huateng_camera_node::srv::SetFloat32_Request>()
{
  return "huateng_camera_node::srv::SetFloat32_Request";
}

template<>
inline const char * name<huateng_camera_node::srv::SetFloat32_Request>()
{
  return "huateng_camera_node/srv/SetFloat32_Request";
}

template<>
struct has_fixed_size<huateng_camera_node::srv::SetFloat32_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<huateng_camera_node::srv::SetFloat32_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<huateng_camera_node::srv::SetFloat32_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace huateng_camera_node
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetFloat32_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetFloat32_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetFloat32_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace huateng_camera_node

namespace rosidl_generator_traits
{

[[deprecated("use huateng_camera_node::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const huateng_camera_node::srv::SetFloat32_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  huateng_camera_node::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use huateng_camera_node::srv::to_yaml() instead")]]
inline std::string to_yaml(const huateng_camera_node::srv::SetFloat32_Response & msg)
{
  return huateng_camera_node::srv::to_yaml(msg);
}

template<>
inline const char * data_type<huateng_camera_node::srv::SetFloat32_Response>()
{
  return "huateng_camera_node::srv::SetFloat32_Response";
}

template<>
inline const char * name<huateng_camera_node::srv::SetFloat32_Response>()
{
  return "huateng_camera_node/srv/SetFloat32_Response";
}

template<>
struct has_fixed_size<huateng_camera_node::srv::SetFloat32_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<huateng_camera_node::srv::SetFloat32_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<huateng_camera_node::srv::SetFloat32_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<huateng_camera_node::srv::SetFloat32>()
{
  return "huateng_camera_node::srv::SetFloat32";
}

template<>
inline const char * name<huateng_camera_node::srv::SetFloat32>()
{
  return "huateng_camera_node/srv/SetFloat32";
}

template<>
struct has_fixed_size<huateng_camera_node::srv::SetFloat32>
  : std::integral_constant<
    bool,
    has_fixed_size<huateng_camera_node::srv::SetFloat32_Request>::value &&
    has_fixed_size<huateng_camera_node::srv::SetFloat32_Response>::value
  >
{
};

template<>
struct has_bounded_size<huateng_camera_node::srv::SetFloat32>
  : std::integral_constant<
    bool,
    has_bounded_size<huateng_camera_node::srv::SetFloat32_Request>::value &&
    has_bounded_size<huateng_camera_node::srv::SetFloat32_Response>::value
  >
{
};

template<>
struct is_service<huateng_camera_node::srv::SetFloat32>
  : std::true_type
{
};

template<>
struct is_service_request<huateng_camera_node::srv::SetFloat32_Request>
  : std::true_type
{
};

template<>
struct is_service_response<huateng_camera_node::srv::SetFloat32_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // HUATENG_CAMERA_NODE__SRV__DETAIL__SET_FLOAT32__TRAITS_HPP_
