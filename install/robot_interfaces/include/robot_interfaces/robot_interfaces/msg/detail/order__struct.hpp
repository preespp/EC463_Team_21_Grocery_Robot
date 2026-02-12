// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_interfaces:msg/Order.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__ORDER__STRUCT_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__ORDER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'items'
#include "robot_interfaces/msg/detail/order_item__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_interfaces__msg__Order __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__msg__Order __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Order_
{
  using Type = Order_<ContainerAllocator>;

  explicit Order_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->order_id = 0ll;
      this->role = "";
      this->requester_id = "";
    }
  }

  explicit Order_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : role(_alloc),
    requester_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->order_id = 0ll;
      this->role = "";
      this->requester_id = "";
    }
  }

  // field types and members
  using _order_id_type =
    int64_t;
  _order_id_type order_id;
  using _role_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _role_type role;
  using _requester_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _requester_id_type requester_id;
  using _items_type =
    std::vector<robot_interfaces::msg::OrderItem_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_interfaces::msg::OrderItem_<ContainerAllocator>>>;
  _items_type items;

  // setters for named parameter idiom
  Type & set__order_id(
    const int64_t & _arg)
  {
    this->order_id = _arg;
    return *this;
  }
  Type & set__role(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->role = _arg;
    return *this;
  }
  Type & set__requester_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->requester_id = _arg;
    return *this;
  }
  Type & set__items(
    const std::vector<robot_interfaces::msg::OrderItem_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_interfaces::msg::OrderItem_<ContainerAllocator>>> & _arg)
  {
    this->items = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::msg::Order_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::msg::Order_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::msg::Order_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::msg::Order_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::Order_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::Order_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::Order_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::Order_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::msg::Order_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::msg::Order_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__msg__Order
    std::shared_ptr<robot_interfaces::msg::Order_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__msg__Order
    std::shared_ptr<robot_interfaces::msg::Order_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Order_ & other) const
  {
    if (this->order_id != other.order_id) {
      return false;
    }
    if (this->role != other.role) {
      return false;
    }
    if (this->requester_id != other.requester_id) {
      return false;
    }
    if (this->items != other.items) {
      return false;
    }
    return true;
  }
  bool operator!=(const Order_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Order_

// alias to use template instance with default allocator
using Order =
  robot_interfaces::msg::Order_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__ORDER__STRUCT_HPP_
