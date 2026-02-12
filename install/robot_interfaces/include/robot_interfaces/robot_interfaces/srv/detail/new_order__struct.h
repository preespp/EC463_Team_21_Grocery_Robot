// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:srv/NewOrder.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__SRV__DETAIL__NEW_ORDER__STRUCT_H_
#define ROBOT_INTERFACES__SRV__DETAIL__NEW_ORDER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'order'
#include "robot_interfaces/msg/detail/order__struct.h"

/// Struct defined in srv/NewOrder in the package robot_interfaces.
typedef struct robot_interfaces__srv__NewOrder_Request
{
  robot_interfaces__msg__Order order;
} robot_interfaces__srv__NewOrder_Request;

// Struct for a sequence of robot_interfaces__srv__NewOrder_Request.
typedef struct robot_interfaces__srv__NewOrder_Request__Sequence
{
  robot_interfaces__srv__NewOrder_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__srv__NewOrder_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/NewOrder in the package robot_interfaces.
typedef struct robot_interfaces__srv__NewOrder_Response
{
  bool accepted;
  rosidl_runtime_c__String message;
} robot_interfaces__srv__NewOrder_Response;

// Struct for a sequence of robot_interfaces__srv__NewOrder_Response.
typedef struct robot_interfaces__srv__NewOrder_Response__Sequence
{
  robot_interfaces__srv__NewOrder_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__srv__NewOrder_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__SRV__DETAIL__NEW_ORDER__STRUCT_H_
