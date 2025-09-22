// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from drone_control_interfaces:srv/Nav.idl
// generated code does not contain a copyright notice

#ifndef DRONE_CONTROL_INTERFACES__SRV__DETAIL__NAV__STRUCT_HPP_
#define DRONE_CONTROL_INTERFACES__SRV__DETAIL__NAV__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__drone_control_interfaces__srv__Nav_Request __attribute__((deprecated))
#else
# define DEPRECATED__drone_control_interfaces__srv__Nav_Request __declspec(deprecated)
#endif

namespace drone_control_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Nav_Request_
{
  using Type = Nav_Request_<ContainerAllocator>;

  explicit Nav_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude_deg = 0.0;
      this->longitude_deg = 0.0;
      this->absolute_altitude_m = 0.0;
    }
  }

  explicit Nav_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude_deg = 0.0;
      this->longitude_deg = 0.0;
      this->absolute_altitude_m = 0.0;
    }
  }

  // field types and members
  using _latitude_deg_type =
    double;
  _latitude_deg_type latitude_deg;
  using _longitude_deg_type =
    double;
  _longitude_deg_type longitude_deg;
  using _absolute_altitude_m_type =
    double;
  _absolute_altitude_m_type absolute_altitude_m;

  // setters for named parameter idiom
  Type & set__latitude_deg(
    const double & _arg)
  {
    this->latitude_deg = _arg;
    return *this;
  }
  Type & set__longitude_deg(
    const double & _arg)
  {
    this->longitude_deg = _arg;
    return *this;
  }
  Type & set__absolute_altitude_m(
    const double & _arg)
  {
    this->absolute_altitude_m = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_control_interfaces::srv::Nav_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_control_interfaces::srv::Nav_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_control_interfaces::srv::Nav_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_control_interfaces::srv::Nav_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_control_interfaces::srv::Nav_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_control_interfaces::srv::Nav_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_control_interfaces::srv::Nav_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_control_interfaces::srv::Nav_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_control_interfaces::srv::Nav_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_control_interfaces::srv::Nav_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_control_interfaces__srv__Nav_Request
    std::shared_ptr<drone_control_interfaces::srv::Nav_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_control_interfaces__srv__Nav_Request
    std::shared_ptr<drone_control_interfaces::srv::Nav_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Nav_Request_ & other) const
  {
    if (this->latitude_deg != other.latitude_deg) {
      return false;
    }
    if (this->longitude_deg != other.longitude_deg) {
      return false;
    }
    if (this->absolute_altitude_m != other.absolute_altitude_m) {
      return false;
    }
    return true;
  }
  bool operator!=(const Nav_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Nav_Request_

// alias to use template instance with default allocator
using Nav_Request =
  drone_control_interfaces::srv::Nav_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace drone_control_interfaces


#ifndef _WIN32
# define DEPRECATED__drone_control_interfaces__srv__Nav_Response __attribute__((deprecated))
#else
# define DEPRECATED__drone_control_interfaces__srv__Nav_Response __declspec(deprecated)
#endif

namespace drone_control_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Nav_Response_
{
  using Type = Nav_Response_<ContainerAllocator>;

  explicit Nav_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit Nav_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_control_interfaces::srv::Nav_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_control_interfaces::srv::Nav_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_control_interfaces::srv::Nav_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_control_interfaces::srv::Nav_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_control_interfaces::srv::Nav_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_control_interfaces::srv::Nav_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_control_interfaces::srv::Nav_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_control_interfaces::srv::Nav_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_control_interfaces::srv::Nav_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_control_interfaces::srv::Nav_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_control_interfaces__srv__Nav_Response
    std::shared_ptr<drone_control_interfaces::srv::Nav_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_control_interfaces__srv__Nav_Response
    std::shared_ptr<drone_control_interfaces::srv::Nav_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Nav_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const Nav_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Nav_Response_

// alias to use template instance with default allocator
using Nav_Response =
  drone_control_interfaces::srv::Nav_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace drone_control_interfaces

namespace drone_control_interfaces
{

namespace srv
{

struct Nav
{
  using Request = drone_control_interfaces::srv::Nav_Request;
  using Response = drone_control_interfaces::srv::Nav_Response;
};

}  // namespace srv

}  // namespace drone_control_interfaces

#endif  // DRONE_CONTROL_INTERFACES__SRV__DETAIL__NAV__STRUCT_HPP_
