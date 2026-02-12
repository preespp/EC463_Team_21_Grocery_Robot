// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_interfaces:msg/OrderItem.idl
// generated code does not contain a copyright notice
#include "robot_interfaces/msg/detail/order_item__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `product_id`
// Member `name`
// Member `aisle`
#include "rosidl_runtime_c/string_functions.h"

bool
robot_interfaces__msg__OrderItem__init(robot_interfaces__msg__OrderItem * msg)
{
  if (!msg) {
    return false;
  }
  // product_id
  if (!rosidl_runtime_c__String__init(&msg->product_id)) {
    robot_interfaces__msg__OrderItem__fini(msg);
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    robot_interfaces__msg__OrderItem__fini(msg);
    return false;
  }
  // aisle
  if (!rosidl_runtime_c__String__init(&msg->aisle)) {
    robot_interfaces__msg__OrderItem__fini(msg);
    return false;
  }
  // rack
  // shelf_level
  // qty
  // price
  // stock
  return true;
}

void
robot_interfaces__msg__OrderItem__fini(robot_interfaces__msg__OrderItem * msg)
{
  if (!msg) {
    return;
  }
  // product_id
  rosidl_runtime_c__String__fini(&msg->product_id);
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // aisle
  rosidl_runtime_c__String__fini(&msg->aisle);
  // rack
  // shelf_level
  // qty
  // price
  // stock
}

bool
robot_interfaces__msg__OrderItem__are_equal(const robot_interfaces__msg__OrderItem * lhs, const robot_interfaces__msg__OrderItem * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // product_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->product_id), &(rhs->product_id)))
  {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // aisle
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->aisle), &(rhs->aisle)))
  {
    return false;
  }
  // rack
  if (lhs->rack != rhs->rack) {
    return false;
  }
  // shelf_level
  if (lhs->shelf_level != rhs->shelf_level) {
    return false;
  }
  // qty
  if (lhs->qty != rhs->qty) {
    return false;
  }
  // price
  if (lhs->price != rhs->price) {
    return false;
  }
  // stock
  if (lhs->stock != rhs->stock) {
    return false;
  }
  return true;
}

bool
robot_interfaces__msg__OrderItem__copy(
  const robot_interfaces__msg__OrderItem * input,
  robot_interfaces__msg__OrderItem * output)
{
  if (!input || !output) {
    return false;
  }
  // product_id
  if (!rosidl_runtime_c__String__copy(
      &(input->product_id), &(output->product_id)))
  {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // aisle
  if (!rosidl_runtime_c__String__copy(
      &(input->aisle), &(output->aisle)))
  {
    return false;
  }
  // rack
  output->rack = input->rack;
  // shelf_level
  output->shelf_level = input->shelf_level;
  // qty
  output->qty = input->qty;
  // price
  output->price = input->price;
  // stock
  output->stock = input->stock;
  return true;
}

robot_interfaces__msg__OrderItem *
robot_interfaces__msg__OrderItem__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__OrderItem * msg = (robot_interfaces__msg__OrderItem *)allocator.allocate(sizeof(robot_interfaces__msg__OrderItem), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_interfaces__msg__OrderItem));
  bool success = robot_interfaces__msg__OrderItem__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_interfaces__msg__OrderItem__destroy(robot_interfaces__msg__OrderItem * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_interfaces__msg__OrderItem__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_interfaces__msg__OrderItem__Sequence__init(robot_interfaces__msg__OrderItem__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__OrderItem * data = NULL;

  if (size) {
    data = (robot_interfaces__msg__OrderItem *)allocator.zero_allocate(size, sizeof(robot_interfaces__msg__OrderItem), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_interfaces__msg__OrderItem__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_interfaces__msg__OrderItem__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_interfaces__msg__OrderItem__Sequence__fini(robot_interfaces__msg__OrderItem__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_interfaces__msg__OrderItem__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_interfaces__msg__OrderItem__Sequence *
robot_interfaces__msg__OrderItem__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__OrderItem__Sequence * array = (robot_interfaces__msg__OrderItem__Sequence *)allocator.allocate(sizeof(robot_interfaces__msg__OrderItem__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_interfaces__msg__OrderItem__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_interfaces__msg__OrderItem__Sequence__destroy(robot_interfaces__msg__OrderItem__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_interfaces__msg__OrderItem__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_interfaces__msg__OrderItem__Sequence__are_equal(const robot_interfaces__msg__OrderItem__Sequence * lhs, const robot_interfaces__msg__OrderItem__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_interfaces__msg__OrderItem__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_interfaces__msg__OrderItem__Sequence__copy(
  const robot_interfaces__msg__OrderItem__Sequence * input,
  robot_interfaces__msg__OrderItem__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_interfaces__msg__OrderItem);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_interfaces__msg__OrderItem * data =
      (robot_interfaces__msg__OrderItem *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_interfaces__msg__OrderItem__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_interfaces__msg__OrderItem__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_interfaces__msg__OrderItem__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
