// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:msg/OrderItem.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__ORDER_ITEM__STRUCT_H_
#define ROBOT_INTERFACES__MSG__DETAIL__ORDER_ITEM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'product_id'
// Member 'name'
// Member 'aisle'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/OrderItem in the package robot_interfaces.
typedef struct robot_interfaces__msg__OrderItem
{
  rosidl_runtime_c__String product_id;
  rosidl_runtime_c__String name;
  rosidl_runtime_c__String aisle;
  int32_t rack;
  int32_t shelf_level;
  int32_t qty;
  float price;
  int32_t stock;
} robot_interfaces__msg__OrderItem;

// Struct for a sequence of robot_interfaces__msg__OrderItem.
typedef struct robot_interfaces__msg__OrderItem__Sequence
{
  robot_interfaces__msg__OrderItem * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__msg__OrderItem__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__MSG__DETAIL__ORDER_ITEM__STRUCT_H_
