// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from drone_control_interfaces:srv/Nav.idl
// generated code does not contain a copyright notice

#ifndef DRONE_CONTROL_INTERFACES__SRV__DETAIL__NAV__TRAITS_HPP_
#define DRONE_CONTROL_INTERFACES__SRV__DETAIL__NAV__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "drone_control_interfaces/srv/detail/nav__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace drone_control_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Nav_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: latitude_deg
  {
    out << "latitude_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude_deg, out);
    out << ", ";
  }

  // member: longitude_deg
  {
    out << "longitude_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude_deg, out);
    out << ", ";
  }

  // member: absolute_altitude_m
  {
    out << "absolute_altitude_m: ";
    rosidl_generator_traits::value_to_yaml(msg.absolute_altitude_m, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Nav_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: latitude_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude_deg, out);
    out << "\n";
  }

  // member: longitude_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude_deg, out);
    out << "\n";
  }

  // member: absolute_altitude_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "absolute_altitude_m: ";
    rosidl_generator_traits::value_to_yaml(msg.absolute_altitude_m, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Nav_Request & msg, bool use_flow_style = false)
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

}  // namespace drone_control_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use drone_control_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const drone_control_interfaces::srv::Nav_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  drone_control_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use drone_control_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const drone_control_interfaces::srv::Nav_Request & msg)
{
  return drone_control_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<drone_control_interfaces::srv::Nav_Request>()
{
  return "drone_control_interfaces::srv::Nav_Request";
}

template<>
inline const char * name<drone_control_interfaces::srv::Nav_Request>()
{
  return "drone_control_interfaces/srv/Nav_Request";
}

template<>
struct has_fixed_size<drone_control_interfaces::srv::Nav_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<drone_control_interfaces::srv::Nav_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<drone_control_interfaces::srv::Nav_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace drone_control_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Nav_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Nav_Response & msg,
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

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Nav_Response & msg, bool use_flow_style = false)
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

}  // namespace drone_control_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use drone_control_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const drone_control_interfaces::srv::Nav_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  drone_control_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use drone_control_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const drone_control_interfaces::srv::Nav_Response & msg)
{
  return drone_control_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<drone_control_interfaces::srv::Nav_Response>()
{
  return "drone_control_interfaces::srv::Nav_Response";
}

template<>
inline const char * name<drone_control_interfaces::srv::Nav_Response>()
{
  return "drone_control_interfaces/srv/Nav_Response";
}

template<>
struct has_fixed_size<drone_control_interfaces::srv::Nav_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<drone_control_interfaces::srv::Nav_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<drone_control_interfaces::srv::Nav_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<drone_control_interfaces::srv::Nav>()
{
  return "drone_control_interfaces::srv::Nav";
}

template<>
inline const char * name<drone_control_interfaces::srv::Nav>()
{
  return "drone_control_interfaces/srv/Nav";
}

template<>
struct has_fixed_size<drone_control_interfaces::srv::Nav>
  : std::integral_constant<
    bool,
    has_fixed_size<drone_control_interfaces::srv::Nav_Request>::value &&
    has_fixed_size<drone_control_interfaces::srv::Nav_Response>::value
  >
{
};

template<>
struct has_bounded_size<drone_control_interfaces::srv::Nav>
  : std::integral_constant<
    bool,
    has_bounded_size<drone_control_interfaces::srv::Nav_Request>::value &&
    has_bounded_size<drone_control_interfaces::srv::Nav_Response>::value
  >
{
};

template<>
struct is_service<drone_control_interfaces::srv::Nav>
  : std::true_type
{
};

template<>
struct is_service_request<drone_control_interfaces::srv::Nav_Request>
  : std::true_type
{
};

template<>
struct is_service_response<drone_control_interfaces::srv::Nav_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DRONE_CONTROL_INTERFACES__SRV__DETAIL__NAV__TRAITS_HPP_
