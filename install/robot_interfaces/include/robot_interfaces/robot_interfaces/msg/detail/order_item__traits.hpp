// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_interfaces:msg/OrderItem.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__ORDER_ITEM__TRAITS_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__ORDER_ITEM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_interfaces/msg/detail/order_item__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const OrderItem & msg,
  std::ostream & out)
{
  out << "{";
  // member: product_id
  {
    out << "product_id: ";
    rosidl_generator_traits::value_to_yaml(msg.product_id, out);
    out << ", ";
  }

  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: aisle
  {
    out << "aisle: ";
    rosidl_generator_traits::value_to_yaml(msg.aisle, out);
    out << ", ";
  }

  // member: rack
  {
    out << "rack: ";
    rosidl_generator_traits::value_to_yaml(msg.rack, out);
    out << ", ";
  }

  // member: shelf_level
  {
    out << "shelf_level: ";
    rosidl_generator_traits::value_to_yaml(msg.shelf_level, out);
    out << ", ";
  }

  // member: qty
  {
    out << "qty: ";
    rosidl_generator_traits::value_to_yaml(msg.qty, out);
    out << ", ";
  }

  // member: price
  {
    out << "price: ";
    rosidl_generator_traits::value_to_yaml(msg.price, out);
    out << ", ";
  }

  // member: stock
  {
    out << "stock: ";
    rosidl_generator_traits::value_to_yaml(msg.stock, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OrderItem & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: product_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "product_id: ";
    rosidl_generator_traits::value_to_yaml(msg.product_id, out);
    out << "\n";
  }

  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: aisle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aisle: ";
    rosidl_generator_traits::value_to_yaml(msg.aisle, out);
    out << "\n";
  }

  // member: rack
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rack: ";
    rosidl_generator_traits::value_to_yaml(msg.rack, out);
    out << "\n";
  }

  // member: shelf_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shelf_level: ";
    rosidl_generator_traits::value_to_yaml(msg.shelf_level, out);
    out << "\n";
  }

  // member: qty
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "qty: ";
    rosidl_generator_traits::value_to_yaml(msg.qty, out);
    out << "\n";
  }

  // member: price
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "price: ";
    rosidl_generator_traits::value_to_yaml(msg.price, out);
    out << "\n";
  }

  // member: stock
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stock: ";
    rosidl_generator_traits::value_to_yaml(msg.stock, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OrderItem & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::msg::OrderItem & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::msg::OrderItem & msg)
{
  return robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::msg::OrderItem>()
{
  return "robot_interfaces::msg::OrderItem";
}

template<>
inline const char * name<robot_interfaces::msg::OrderItem>()
{
  return "robot_interfaces/msg/OrderItem";
}

template<>
struct has_fixed_size<robot_interfaces::msg::OrderItem>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_interfaces::msg::OrderItem>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_interfaces::msg::OrderItem>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_INTERFACES__MSG__DETAIL__ORDER_ITEM__TRAITS_HPP_
