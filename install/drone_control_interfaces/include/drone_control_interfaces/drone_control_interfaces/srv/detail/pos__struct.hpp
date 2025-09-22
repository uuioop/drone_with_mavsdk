// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from drone_control_interfaces:srv/Pos.idl
// generated code does not contain a copyright notice

#ifndef DRONE_CONTROL_INTERFACES__SRV__DETAIL__POS__STRUCT_HPP_
#define DRONE_CONTROL_INTERFACES__SRV__DETAIL__POS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__drone_control_interfaces__srv__Pos_Request __attribute__((deprecated))
#else
# define DEPRECATED__drone_control_interfaces__srv__Pos_Request __declspec(deprecated)
#endif

namespace drone_control_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Pos_Request_
{
  using Type = Pos_Request_<ContainerAllocator>;

  explicit Pos_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->w = 0.0;
      this->is_ned = false;
    }
  }

  explicit Pos_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->w = 0.0;
      this->is_ned = false;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _z_type =
    double;
  _z_type z;
  using _w_type =
    double;
  _w_type w;
  using _is_ned_type =
    bool;
  _is_ned_type is_ned;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const double & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__w(
    const double & _arg)
  {
    this->w = _arg;
    return *this;
  }
  Type & set__is_ned(
    const bool & _arg)
  {
    this->is_ned = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_control_interfaces::srv::Pos_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_control_interfaces::srv::Pos_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_control_interfaces::srv::Pos_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_control_interfaces::srv::Pos_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_control_interfaces::srv::Pos_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_control_interfaces::srv::Pos_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_control_interfaces::srv::Pos_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_control_interfaces::srv::Pos_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_control_interfaces::srv::Pos_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_control_interfaces::srv::Pos_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_control_interfaces__srv__Pos_Request
    std::shared_ptr<drone_control_interfaces::srv::Pos_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_control_interfaces__srv__Pos_Request
    std::shared_ptr<drone_control_interfaces::srv::Pos_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pos_Request_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->w != other.w) {
      return false;
    }
    if (this->is_ned != other.is_ned) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pos_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pos_Request_

// alias to use template instance with default allocator
using Pos_Request =
  drone_control_interfaces::srv::Pos_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace drone_control_interfaces


#ifndef _WIN32
# define DEPRECATED__drone_control_interfaces__srv__Pos_Response __attribute__((deprecated))
#else
# define DEPRECATED__drone_control_interfaces__srv__Pos_Response __declspec(deprecated)
#endif

namespace drone_control_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Pos_Response_
{
  using Type = Pos_Response_<ContainerAllocator>;

  explicit Pos_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit Pos_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    drone_control_interfaces::srv::Pos_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_control_interfaces::srv::Pos_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_control_interfaces::srv::Pos_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_control_interfaces::srv::Pos_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_control_interfaces::srv::Pos_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_control_interfaces::srv::Pos_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_control_interfaces::srv::Pos_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_control_interfaces::srv::Pos_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_control_interfaces::srv::Pos_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_control_interfaces::srv::Pos_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_control_interfaces__srv__Pos_Response
    std::shared_ptr<drone_control_interfaces::srv::Pos_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_control_interfaces__srv__Pos_Response
    std::shared_ptr<drone_control_interfaces::srv::Pos_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pos_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pos_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pos_Response_

// alias to use template instance with default allocator
using Pos_Response =
  drone_control_interfaces::srv::Pos_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace drone_control_interfaces

namespace drone_control_interfaces
{

namespace srv
{

struct Pos
{
  using Request = drone_control_interfaces::srv::Pos_Request;
  using Response = drone_control_interfaces::srv::Pos_Response;
};

}  // namespace srv

}  // namespace drone_control_interfaces

#endif  // DRONE_CONTROL_INTERFACES__SRV__DETAIL__POS__STRUCT_HPP_
