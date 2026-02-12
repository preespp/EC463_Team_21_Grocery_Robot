// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:srv/NewOrder.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__SRV__DETAIL__NEW_ORDER__BUILDER_HPP_
#define ROBOT_INTERFACES__SRV__DETAIL__NEW_ORDER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/srv/detail/new_order__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_NewOrder_Request_order
{
public:
  Init_NewOrder_Request_order()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_interfaces::srv::NewOrder_Request order(::robot_interfaces::srv::NewOrder_Request::_order_type arg)
  {
    msg_.order = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::srv::NewOrder_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::srv::NewOrder_Request>()
{
  return robot_interfaces::srv::builder::Init_NewOrder_Request_order();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_NewOrder_Response_message
{
public:
  explicit Init_NewOrder_Response_message(::robot_interfaces::srv::NewOrder_Response & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::srv::NewOrder_Response message(::robot_interfaces::srv::NewOrder_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::srv::NewOrder_Response msg_;
};

class Init_NewOrder_Response_accepted
{
public:
  Init_NewOrder_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NewOrder_Response_message accepted(::robot_interfaces::srv::NewOrder_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_NewOrder_Response_message(msg_);
  }

private:
  ::robot_interfaces::srv::NewOrder_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::srv::NewOrder_Response>()
{
  return robot_interfaces::srv::builder::Init_NewOrder_Response_accepted();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__SRV__DETAIL__NEW_ORDER__BUILDER_HPP_
