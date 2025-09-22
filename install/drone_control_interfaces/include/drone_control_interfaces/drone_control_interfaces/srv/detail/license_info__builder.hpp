// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from drone_control_interfaces:srv/LicenseInfo.idl
// generated code does not contain a copyright notice

#ifndef DRONE_CONTROL_INTERFACES__SRV__DETAIL__LICENSE_INFO__BUILDER_HPP_
#define DRONE_CONTROL_INTERFACES__SRV__DETAIL__LICENSE_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "drone_control_interfaces/srv/detail/license_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace drone_control_interfaces
{

namespace srv
{

namespace builder
{

class Init_LicenseInfo_Request_target_plate
{
public:
  Init_LicenseInfo_Request_target_plate()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::drone_control_interfaces::srv::LicenseInfo_Request target_plate(::drone_control_interfaces::srv::LicenseInfo_Request::_target_plate_type arg)
  {
    msg_.target_plate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_control_interfaces::srv::LicenseInfo_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_control_interfaces::srv::LicenseInfo_Request>()
{
  return drone_control_interfaces::srv::builder::Init_LicenseInfo_Request_target_plate();
}

}  // namespace drone_control_interfaces


namespace drone_control_interfaces
{

namespace srv
{

namespace builder
{

class Init_LicenseInfo_Response_message
{
public:
  Init_LicenseInfo_Response_message()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::drone_control_interfaces::srv::LicenseInfo_Response message(::drone_control_interfaces::srv::LicenseInfo_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_control_interfaces::srv::LicenseInfo_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_control_interfaces::srv::LicenseInfo_Response>()
{
  return drone_control_interfaces::srv::builder::Init_LicenseInfo_Response_message();
}

}  // namespace drone_control_interfaces

#endif  // DRONE_CONTROL_INTERFACES__SRV__DETAIL__LICENSE_INFO__BUILDER_HPP_
