// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from drone_control_interfaces:srv/Nav.idl
// generated code does not contain a copyright notice

#ifndef DRONE_CONTROL_INTERFACES__SRV__DETAIL__NAV__BUILDER_HPP_
#define DRONE_CONTROL_INTERFACES__SRV__DETAIL__NAV__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "drone_control_interfaces/srv/detail/nav__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace drone_control_interfaces
{

namespace srv
{

namespace builder
{

class Init_Nav_Request_absolute_altitude_m
{
public:
  explicit Init_Nav_Request_absolute_altitude_m(::drone_control_interfaces::srv::Nav_Request & msg)
  : msg_(msg)
  {}
  ::drone_control_interfaces::srv::Nav_Request absolute_altitude_m(::drone_control_interfaces::srv::Nav_Request::_absolute_altitude_m_type arg)
  {
    msg_.absolute_altitude_m = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_control_interfaces::srv::Nav_Request msg_;
};

class Init_Nav_Request_longitude_deg
{
public:
  explicit Init_Nav_Request_longitude_deg(::drone_control_interfaces::srv::Nav_Request & msg)
  : msg_(msg)
  {}
  Init_Nav_Request_absolute_altitude_m longitude_deg(::drone_control_interfaces::srv::Nav_Request::_longitude_deg_type arg)
  {
    msg_.longitude_deg = std::move(arg);
    return Init_Nav_Request_absolute_altitude_m(msg_);
  }

private:
  ::drone_control_interfaces::srv::Nav_Request msg_;
};

class Init_Nav_Request_latitude_deg
{
public:
  Init_Nav_Request_latitude_deg()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Nav_Request_longitude_deg latitude_deg(::drone_control_interfaces::srv::Nav_Request::_latitude_deg_type arg)
  {
    msg_.latitude_deg = std::move(arg);
    return Init_Nav_Request_longitude_deg(msg_);
  }

private:
  ::drone_control_interfaces::srv::Nav_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_control_interfaces::srv::Nav_Request>()
{
  return drone_control_interfaces::srv::builder::Init_Nav_Request_latitude_deg();
}

}  // namespace drone_control_interfaces


namespace drone_control_interfaces
{

namespace srv
{

namespace builder
{

class Init_Nav_Response_message
{
public:
  explicit Init_Nav_Response_message(::drone_control_interfaces::srv::Nav_Response & msg)
  : msg_(msg)
  {}
  ::drone_control_interfaces::srv::Nav_Response message(::drone_control_interfaces::srv::Nav_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_control_interfaces::srv::Nav_Response msg_;
};

class Init_Nav_Response_success
{
public:
  Init_Nav_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Nav_Response_message success(::drone_control_interfaces::srv::Nav_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Nav_Response_message(msg_);
  }

private:
  ::drone_control_interfaces::srv::Nav_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_control_interfaces::srv::Nav_Response>()
{
  return drone_control_interfaces::srv::builder::Init_Nav_Response_success();
}

}  // namespace drone_control_interfaces

#endif  // DRONE_CONTROL_INTERFACES__SRV__DETAIL__NAV__BUILDER_HPP_
