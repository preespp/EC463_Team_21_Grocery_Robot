// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:msg/Order.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__ORDER__BUILDER_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__ORDER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/msg/detail/order__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_Order_items
{
public:
  explicit Init_Order_items(::robot_interfaces::msg::Order & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::msg::Order items(::robot_interfaces::msg::Order::_items_type arg)
  {
    msg_.items = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::msg::Order msg_;
};

class Init_Order_requester_id
{
public:
  explicit Init_Order_requester_id(::robot_interfaces::msg::Order & msg)
  : msg_(msg)
  {}
  Init_Order_items requester_id(::robot_interfaces::msg::Order::_requester_id_type arg)
  {
    msg_.requester_id = std::move(arg);
    return Init_Order_items(msg_);
  }

private:
  ::robot_interfaces::msg::Order msg_;
};

class Init_Order_role
{
public:
  explicit Init_Order_role(::robot_interfaces::msg::Order & msg)
  : msg_(msg)
  {}
  Init_Order_requester_id role(::robot_interfaces::msg::Order::_role_type arg)
  {
    msg_.role = std::move(arg);
    return Init_Order_requester_id(msg_);
  }

private:
  ::robot_interfaces::msg::Order msg_;
};

class Init_Order_order_id
{
public:
  Init_Order_order_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Order_role order_id(::robot_interfaces::msg::Order::_order_id_type arg)
  {
    msg_.order_id = std::move(arg);
    return Init_Order_role(msg_);
  }

private:
  ::robot_interfaces::msg::Order msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::msg::Order>()
{
  return robot_interfaces::msg::builder::Init_Order_order_id();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__ORDER__BUILDER_HPP_
