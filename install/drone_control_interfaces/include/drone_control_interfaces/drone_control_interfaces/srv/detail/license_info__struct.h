// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from drone_control_interfaces:srv/LicenseInfo.idl
// generated code does not contain a copyright notice

#ifndef DRONE_CONTROL_INTERFACES__SRV__DETAIL__LICENSE_INFO__STRUCT_H_
#define DRONE_CONTROL_INTERFACES__SRV__DETAIL__LICENSE_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_plate'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/LicenseInfo in the package drone_control_interfaces.
typedef struct drone_control_interfaces__srv__LicenseInfo_Request
{
  rosidl_runtime_c__String target_plate;
} drone_control_interfaces__srv__LicenseInfo_Request;

// Struct for a sequence of drone_control_interfaces__srv__LicenseInfo_Request.
typedef struct drone_control_interfaces__srv__LicenseInfo_Request__Sequence
{
  drone_control_interfaces__srv__LicenseInfo_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_control_interfaces__srv__LicenseInfo_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/LicenseInfo in the package drone_control_interfaces.
typedef struct drone_control_interfaces__srv__LicenseInfo_Response
{
  rosidl_runtime_c__String message;
} drone_control_interfaces__srv__LicenseInfo_Response;

// Struct for a sequence of drone_control_interfaces__srv__LicenseInfo_Response.
typedef struct drone_control_interfaces__srv__LicenseInfo_Response__Sequence
{
  drone_control_interfaces__srv__LicenseInfo_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_control_interfaces__srv__LicenseInfo_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DRONE_CONTROL_INTERFACES__SRV__DETAIL__LICENSE_INFO__STRUCT_H_
