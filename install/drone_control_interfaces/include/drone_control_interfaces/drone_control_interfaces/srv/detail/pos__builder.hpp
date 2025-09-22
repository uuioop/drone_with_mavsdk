// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from drone_control_interfaces:srv/Pos.idl
// generated code does not contain a copyright notice

#ifndef DRONE_CONTROL_INTERFACES__SRV__DETAIL__POS__BUILDER_HPP_
#define DRONE_CONTROL_INTERFACES__SRV__DETAIL__POS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "drone_control_interfaces/srv/detail/pos__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace drone_control_interfaces
{

namespace srv
{

namespace builder
{

class Init_Pos_Request_is_ned
{
public:
  explicit Init_Pos_Request_is_ned(::drone_control_interfaces::srv::Pos_Request & msg)
  : msg_(msg)
  {}
  ::drone_control_interfaces::srv::Pos_Request is_ned(::drone_control_interfaces::srv::Pos_Request::_is_ned_type arg)
  {
    msg_.is_ned = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_control_interfaces::srv::Pos_Request msg_;
};

class Init_Pos_Request_w
{
public:
  explicit Init_Pos_Request_w(::drone_control_interfaces::srv::Pos_Request & msg)
  : msg_(msg)
  {}
  Init_Pos_Request_is_ned w(::drone_control_interfaces::srv::Pos_Request::_w_type arg)
  {
    msg_.w = std::move(arg);
    return Init_Pos_Request_is_ned(msg_);
  }

private:
  ::drone_control_interfaces::srv::Pos_Request msg_;
};

class Init_Pos_Request_z
{
public:
  explicit Init_Pos_Request_z(::drone_control_interfaces::srv::Pos_Request & msg)
  : msg_(msg)
  {}
  Init_Pos_Request_w z(::drone_control_interfaces::srv::Pos_Request::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_Pos_Request_w(msg_);
  }

private:
  ::drone_control_interfaces::srv::Pos_Request msg_;
};

class Init_Pos_Request_y
{
public:
  explicit Init_Pos_Request_y(::drone_control_interfaces::srv::Pos_Request & msg)
  : msg_(msg)
  {}
  Init_Pos_Request_z y(::drone_control_interfaces::srv::Pos_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Pos_Request_z(msg_);
  }

private:
  ::drone_control_interfaces::srv::Pos_Request msg_;
};

class Init_Pos_Request_x
{
public:
  Init_Pos_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pos_Request_y x(::drone_control_interfaces::srv::Pos_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Pos_Request_y(msg_);
  }

private:
  ::drone_control_interfaces::srv::Pos_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_control_interfaces::srv::Pos_Request>()
{
  return drone_control_interfaces::srv::builder::Init_Pos_Request_x();
}

}  // namespace drone_control_interfaces


namespace drone_control_interfaces
{

namespace srv
{

namespace builder
{

class Init_Pos_Response_message
{
public:
  explicit Init_Pos_Response_message(::drone_control_interfaces::srv::Pos_Response & msg)
  : msg_(msg)
  {}
  ::drone_control_interfaces::srv::Pos_Response message(::drone_control_interfaces::srv::Pos_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_control_interfaces::srv::Pos_Response msg_;
};

class Init_Pos_Response_success
{
public:
  Init_Pos_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pos_Response_message success(::drone_control_interfaces::srv::Pos_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Pos_Response_message(msg_);
  }

private:
  ::drone_control_interfaces::srv::Pos_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_control_interfaces::srv::Pos_Response>()
{
  return drone_control_interfaces::srv::builder::Init_Pos_Response_success();
}

}  // namespace drone_control_interfaces

#endif  // DRONE_CONTROL_INTERFACES__SRV__DETAIL__POS__BUILDER_HPP_
