// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from drone_control_interfaces:srv/LicenseInfo.idl
// generated code does not contain a copyright notice

#ifndef DRONE_CONTROL_INTERFACES__SRV__DETAIL__LICENSE_INFO__STRUCT_HPP_
#define DRONE_CONTROL_INTERFACES__SRV__DETAIL__LICENSE_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__drone_control_interfaces__srv__LicenseInfo_Request __attribute__((deprecated))
#else
# define DEPRECATED__drone_control_interfaces__srv__LicenseInfo_Request __declspec(deprecated)
#endif

namespace drone_control_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LicenseInfo_Request_
{
  using Type = LicenseInfo_Request_<ContainerAllocator>;

  explicit LicenseInfo_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_plate = "";
    }
  }

  explicit LicenseInfo_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_plate(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_plate = "";
    }
  }

  // field types and members
  using _target_plate_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _target_plate_type target_plate;

  // setters for named parameter idiom
  Type & set__target_plate(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->target_plate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_control_interfaces::srv::LicenseInfo_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_control_interfaces::srv::LicenseInfo_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_control_interfaces::srv::LicenseInfo_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_control_interfaces::srv::LicenseInfo_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_control_interfaces::srv::LicenseInfo_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_control_interfaces::srv::LicenseInfo_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_control_interfaces::srv::LicenseInfo_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_control_interfaces::srv::LicenseInfo_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_control_interfaces::srv::LicenseInfo_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_control_interfaces::srv::LicenseInfo_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_control_interfaces__srv__LicenseInfo_Request
    std::shared_ptr<drone_control_interfaces::srv::LicenseInfo_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_control_interfaces__srv__LicenseInfo_Request
    std::shared_ptr<drone_control_interfaces::srv::LicenseInfo_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LicenseInfo_Request_ & other) const
  {
    if (this->target_plate != other.target_plate) {
      return false;
    }
    return true;
  }
  bool operator!=(const LicenseInfo_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LicenseInfo_Request_

// alias to use template instance with default allocator
using LicenseInfo_Request =
  drone_control_interfaces::srv::LicenseInfo_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace drone_control_interfaces


#ifndef _WIN32
# define DEPRECATED__drone_control_interfaces__srv__LicenseInfo_Response __attribute__((deprecated))
#else
# define DEPRECATED__drone_control_interfaces__srv__LicenseInfo_Response __declspec(deprecated)
#endif

namespace drone_control_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LicenseInfo_Response_
{
  using Type = LicenseInfo_Response_<ContainerAllocator>;

  explicit LicenseInfo_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message = "";
    }
  }

  explicit LicenseInfo_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message = "";
    }
  }

  // field types and members
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_control_interfaces::srv::LicenseInfo_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_control_interfaces::srv::LicenseInfo_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_control_interfaces::srv::LicenseInfo_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_control_interfaces::srv::LicenseInfo_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_control_interfaces::srv::LicenseInfo_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_control_interfaces::srv::LicenseInfo_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_control_interfaces::srv::LicenseInfo_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_control_interfaces::srv::LicenseInfo_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_control_interfaces::srv::LicenseInfo_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_control_interfaces::srv::LicenseInfo_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_control_interfaces__srv__LicenseInfo_Response
    std::shared_ptr<drone_control_interfaces::srv::LicenseInfo_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_control_interfaces__srv__LicenseInfo_Response
    std::shared_ptr<drone_control_interfaces::srv::LicenseInfo_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LicenseInfo_Response_ & other) const
  {
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const LicenseInfo_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LicenseInfo_Response_

// alias to use template instance with default allocator
using LicenseInfo_Response =
  drone_control_interfaces::srv::LicenseInfo_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace drone_control_interfaces

namespace drone_control_interfaces
{

namespace srv
{

struct LicenseInfo
{
  using Request = drone_control_interfaces::srv::LicenseInfo_Request;
  using Response = drone_control_interfaces::srv::LicenseInfo_Response;
};

}  // namespace srv

}  // namespace drone_control_interfaces

#endif  // DRONE_CONTROL_INTERFACES__SRV__DETAIL__LICENSE_INFO__STRUCT_HPP_
