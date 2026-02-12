// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_interfaces:msg/Order.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_interfaces/msg/detail/order__rosidl_typesupport_introspection_c.h"
#include "robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_interfaces/msg/detail/order__functions.h"
#include "robot_interfaces/msg/detail/order__struct.h"


// Include directives for member types
// Member `role`
// Member `requester_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `items`
#include "robot_interfaces/msg/order_item.h"
// Member `items`
#include "robot_interfaces/msg/detail/order_item__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_interfaces__msg__Order__init(message_memory);
}

void robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_fini_function(void * message_memory)
{
  robot_interfaces__msg__Order__fini(message_memory);
}

size_t robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__size_function__Order__items(
  const void * untyped_member)
{
  const robot_interfaces__msg__OrderItem__Sequence * member =
    (const robot_interfaces__msg__OrderItem__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__get_const_function__Order__items(
  const void * untyped_member, size_t index)
{
  const robot_interfaces__msg__OrderItem__Sequence * member =
    (const robot_interfaces__msg__OrderItem__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__get_function__Order__items(
  void * untyped_member, size_t index)
{
  robot_interfaces__msg__OrderItem__Sequence * member =
    (robot_interfaces__msg__OrderItem__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__fetch_function__Order__items(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const robot_interfaces__msg__OrderItem * item =
    ((const robot_interfaces__msg__OrderItem *)
    robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__get_const_function__Order__items(untyped_member, index));
  robot_interfaces__msg__OrderItem * value =
    (robot_interfaces__msg__OrderItem *)(untyped_value);
  *value = *item;
}

void robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__assign_function__Order__items(
  void * untyped_member, size_t index, const void * untyped_value)
{
  robot_interfaces__msg__OrderItem * item =
    ((robot_interfaces__msg__OrderItem *)
    robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__get_function__Order__items(untyped_member, index));
  const robot_interfaces__msg__OrderItem * value =
    (const robot_interfaces__msg__OrderItem *)(untyped_value);
  *item = *value;
}

bool robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__resize_function__Order__items(
  void * untyped_member, size_t size)
{
  robot_interfaces__msg__OrderItem__Sequence * member =
    (robot_interfaces__msg__OrderItem__Sequence *)(untyped_member);
  robot_interfaces__msg__OrderItem__Sequence__fini(member);
  return robot_interfaces__msg__OrderItem__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_member_array[4] = {
  {
    "order_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_interfaces__msg__Order, order_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "role",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_interfaces__msg__Order, role),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "requester_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_interfaces__msg__Order, requester_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "items",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_interfaces__msg__Order, items),  // bytes offset in struct
    NULL,  // default value
    robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__size_function__Order__items,  // size() function pointer
    robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__get_const_function__Order__items,  // get_const(index) function pointer
    robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__get_function__Order__items,  // get(index) function pointer
    robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__fetch_function__Order__items,  // fetch(index, &value) function pointer
    robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__assign_function__Order__items,  // assign(index, value) function pointer
    robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__resize_function__Order__items  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_members = {
  "robot_interfaces__msg",  // message namespace
  "Order",  // message name
  4,  // number of fields
  sizeof(robot_interfaces__msg__Order),
  robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_member_array,  // message members
  robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_type_support_handle = {
  0,
  &robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, msg, Order)() {
  robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, msg, OrderItem)();
  if (!robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_type_support_handle.typesupport_identifier) {
    robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
