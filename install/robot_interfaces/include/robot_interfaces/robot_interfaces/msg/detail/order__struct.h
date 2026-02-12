// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:msg/Order.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__ORDER__STRUCT_H_
#define ROBOT_INTERFACES__MSG__DETAIL__ORDER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'role'
// Member 'requester_id'
#include "rosidl_runtime_c/string.h"
// Member 'items'
#include "robot_interfaces/msg/detail/order_item__struct.h"

/// Struct defined in msg/Order in the package robot_interfaces.
typedef struct robot_interfaces__msg__Order
{
  int64_t order_id;
  rosidl_runtime_c__String role;
  rosidl_runtime_c__String requester_id;
  robot_interfaces__msg__OrderItem__Sequence items;
} robot_interfaces__msg__Order;

// Struct for a sequence of robot_interfaces__msg__Order.
typedef struct robot_interfaces__msg__Order__Sequence
{
  robot_interfaces__msg__Order * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__msg__Order__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__MSG__DETAIL__ORDER__STRUCT_H_
