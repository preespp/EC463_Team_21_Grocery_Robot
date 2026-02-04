// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_interfaces:msg/OrderItem.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__ORDER_ITEM__STRUCT_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__ORDER_ITEM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_interfaces__msg__OrderItem __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__msg__OrderItem __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OrderItem_
{
  using Type = OrderItem_<ContainerAllocator>;

  explicit OrderItem_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->product_id = "";
      this->name = "";
      this->aisle = "";
      this->rack = 0l;
      this->shelf_level = 0l;
      this->qty = 0l;
      this->price = 0.0f;
      this->stock = 0l;
    }
  }

  explicit OrderItem_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : product_id(_alloc),
    name(_alloc),
    aisle(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->product_id = "";
      this->name = "";
      this->aisle = "";
      this->rack = 0l;
      this->shelf_level = 0l;
      this->qty = 0l;
      this->price = 0.0f;
      this->stock = 0l;
    }
  }

  // field types and members
  using _product_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _product_id_type product_id;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _aisle_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _aisle_type aisle;
  using _rack_type =
    int32_t;
  _rack_type rack;
  using _shelf_level_type =
    int32_t;
  _shelf_level_type shelf_level;
  using _qty_type =
    int32_t;
  _qty_type qty;
  using _price_type =
    float;
  _price_type price;
  using _stock_type =
    int32_t;
  _stock_type stock;

  // setters for named parameter idiom
  Type & set__product_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->product_id = _arg;
    return *this;
  }
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__aisle(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->aisle = _arg;
    return *this;
  }
  Type & set__rack(
    const int32_t & _arg)
  {
    this->rack = _arg;
    return *this;
  }
  Type & set__shelf_level(
    const int32_t & _arg)
  {
    this->shelf_level = _arg;
    return *this;
  }
  Type & set__qty(
    const int32_t & _arg)
  {
    this->qty = _arg;
    return *this;
  }
  Type & set__price(
    const float & _arg)
  {
    this->price = _arg;
    return *this;
  }
  Type & set__stock(
    const int32_t & _arg)
  {
    this->stock = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::msg::OrderItem_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::msg::OrderItem_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::msg::OrderItem_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::msg::OrderItem_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::OrderItem_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::OrderItem_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::OrderItem_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::OrderItem_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::msg::OrderItem_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::msg::OrderItem_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__msg__OrderItem
    std::shared_ptr<robot_interfaces::msg::OrderItem_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__msg__OrderItem
    std::shared_ptr<robot_interfaces::msg::OrderItem_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OrderItem_ & other) const
  {
    if (this->product_id != other.product_id) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->aisle != other.aisle) {
      return false;
    }
    if (this->rack != other.rack) {
      return false;
    }
    if (this->shelf_level != other.shelf_level) {
      return false;
    }
    if (this->qty != other.qty) {
      return false;
    }
    if (this->price != other.price) {
      return false;
    }
    if (this->stock != other.stock) {
      return false;
    }
    return true;
  }
  bool operator!=(const OrderItem_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OrderItem_

// alias to use template instance with default allocator
using OrderItem =
  robot_interfaces::msg::OrderItem_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__ORDER_ITEM__STRUCT_HPP_
