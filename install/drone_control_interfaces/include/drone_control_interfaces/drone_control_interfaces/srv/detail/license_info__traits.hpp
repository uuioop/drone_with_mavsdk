// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from drone_control_interfaces:srv/LicenseInfo.idl
// generated code does not contain a copyright notice

#ifndef DRONE_CONTROL_INTERFACES__SRV__DETAIL__LICENSE_INFO__TRAITS_HPP_
#define DRONE_CONTROL_INTERFACES__SRV__DETAIL__LICENSE_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "drone_control_interfaces/srv/detail/license_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace drone_control_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LicenseInfo_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_plate
  {
    out << "target_plate: ";
    rosidl_generator_traits::value_to_yaml(msg.target_plate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LicenseInfo_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_plate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_plate: ";
    rosidl_generator_traits::value_to_yaml(msg.target_plate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LicenseInfo_Request & msg, bool use_flow_style = false)
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
  const drone_control_interfaces::srv::LicenseInfo_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  drone_control_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use drone_control_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const drone_control_interfaces::srv::LicenseInfo_Request & msg)
{
  return drone_control_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<drone_control_interfaces::srv::LicenseInfo_Request>()
{
  return "drone_control_interfaces::srv::LicenseInfo_Request";
}

template<>
inline const char * name<drone_control_interfaces::srv::LicenseInfo_Request>()
{
  return "drone_control_interfaces/srv/LicenseInfo_Request";
}

template<>
struct has_fixed_size<drone_control_interfaces::srv::LicenseInfo_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<drone_control_interfaces::srv::LicenseInfo_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<drone_control_interfaces::srv::LicenseInfo_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace drone_control_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LicenseInfo_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LicenseInfo_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
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

inline std::string to_yaml(const LicenseInfo_Response & msg, bool use_flow_style = false)
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
  const drone_control_interfaces::srv::LicenseInfo_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  drone_control_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use drone_control_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const drone_control_interfaces::srv::LicenseInfo_Response & msg)
{
  return drone_control_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<drone_control_interfaces::srv::LicenseInfo_Response>()
{
  return "drone_control_interfaces::srv::LicenseInfo_Response";
}

template<>
inline const char * name<drone_control_interfaces::srv::LicenseInfo_Response>()
{
  return "drone_control_interfaces/srv/LicenseInfo_Response";
}

template<>
struct has_fixed_size<drone_control_interfaces::srv::LicenseInfo_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<drone_control_interfaces::srv::LicenseInfo_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<drone_control_interfaces::srv::LicenseInfo_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<drone_control_interfaces::srv::LicenseInfo>()
{
  return "drone_control_interfaces::srv::LicenseInfo";
}

template<>
inline const char * name<drone_control_interfaces::srv::LicenseInfo>()
{
  return "drone_control_interfaces/srv/LicenseInfo";
}

template<>
struct has_fixed_size<drone_control_interfaces::srv::LicenseInfo>
  : std::integral_constant<
    bool,
    has_fixed_size<drone_control_interfaces::srv::LicenseInfo_Request>::value &&
    has_fixed_size<drone_control_interfaces::srv::LicenseInfo_Response>::value
  >
{
};

template<>
struct has_bounded_size<drone_control_interfaces::srv::LicenseInfo>
  : std::integral_constant<
    bool,
    has_bounded_size<drone_control_interfaces::srv::LicenseInfo_Request>::value &&
    has_bounded_size<drone_control_interfaces::srv::LicenseInfo_Response>::value
  >
{
};

template<>
struct is_service<drone_control_interfaces::srv::LicenseInfo>
  : std::true_type
{
};

template<>
struct is_service_request<drone_control_interfaces::srv::LicenseInfo_Request>
  : std::true_type
{
};

template<>
struct is_service_response<drone_control_interfaces::srv::LicenseInfo_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DRONE_CONTROL_INTERFACES__SRV__DETAIL__LICENSE_INFO__TRAITS_HPP_
