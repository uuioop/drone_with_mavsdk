// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from drone_control_interfaces:srv/Nav.idl
// generated code does not contain a copyright notice

#ifndef DRONE_CONTROL_INTERFACES__SRV__DETAIL__NAV__STRUCT_H_
#define DRONE_CONTROL_INTERFACES__SRV__DETAIL__NAV__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Nav in the package drone_control_interfaces.
typedef struct drone_control_interfaces__srv__Nav_Request
{
  double latitude_deg;
  double longitude_deg;
  double absolute_altitude_m;
} drone_control_interfaces__srv__Nav_Request;

// Struct for a sequence of drone_control_interfaces__srv__Nav_Request.
typedef struct drone_control_interfaces__srv__Nav_Request__Sequence
{
  drone_control_interfaces__srv__Nav_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_control_interfaces__srv__Nav_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Nav in the package drone_control_interfaces.
typedef struct drone_control_interfaces__srv__Nav_Response
{
  bool success;
  rosidl_runtime_c__String message;
} drone_control_interfaces__srv__Nav_Response;

// Struct for a sequence of drone_control_interfaces__srv__Nav_Response.
typedef struct drone_control_interfaces__srv__Nav_Response__Sequence
{
  drone_control_interfaces__srv__Nav_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_control_interfaces__srv__Nav_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DRONE_CONTROL_INTERFACES__SRV__DETAIL__NAV__STRUCT_H_
