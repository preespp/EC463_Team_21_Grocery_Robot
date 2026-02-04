// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:msg/OrderItem.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__ORDER_ITEM__BUILDER_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__ORDER_ITEM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/msg/detail/order_item__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_OrderItem_stock
{
public:
  explicit Init_OrderItem_stock(::robot_interfaces::msg::OrderItem & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::msg::OrderItem stock(::robot_interfaces::msg::OrderItem::_stock_type arg)
  {
    msg_.stock = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::msg::OrderItem msg_;
};

class Init_OrderItem_price
{
public:
  explicit Init_OrderItem_price(::robot_interfaces::msg::OrderItem & msg)
  : msg_(msg)
  {}
  Init_OrderItem_stock price(::robot_interfaces::msg::OrderItem::_price_type arg)
  {
    msg_.price = std::move(arg);
    return Init_OrderItem_stock(msg_);
  }

private:
  ::robot_interfaces::msg::OrderItem msg_;
};

class Init_OrderItem_qty
{
public:
  explicit Init_OrderItem_qty(::robot_interfaces::msg::OrderItem & msg)
  : msg_(msg)
  {}
  Init_OrderItem_price qty(::robot_interfaces::msg::OrderItem::_qty_type arg)
  {
    msg_.qty = std::move(arg);
    return Init_OrderItem_price(msg_);
  }

private:
  ::robot_interfaces::msg::OrderItem msg_;
};

class Init_OrderItem_shelf_level
{
public:
  explicit Init_OrderItem_shelf_level(::robot_interfaces::msg::OrderItem & msg)
  : msg_(msg)
  {}
  Init_OrderItem_qty shelf_level(::robot_interfaces::msg::OrderItem::_shelf_level_type arg)
  {
    msg_.shelf_level = std::move(arg);
    return Init_OrderItem_qty(msg_);
  }

private:
  ::robot_interfaces::msg::OrderItem msg_;
};

class Init_OrderItem_rack
{
public:
  explicit Init_OrderItem_rack(::robot_interfaces::msg::OrderItem & msg)
  : msg_(msg)
  {}
  Init_OrderItem_shelf_level rack(::robot_interfaces::msg::OrderItem::_rack_type arg)
  {
    msg_.rack = std::move(arg);
    return Init_OrderItem_shelf_level(msg_);
  }

private:
  ::robot_interfaces::msg::OrderItem msg_;
};

class Init_OrderItem_aisle
{
public:
  explicit Init_OrderItem_aisle(::robot_interfaces::msg::OrderItem & msg)
  : msg_(msg)
  {}
  Init_OrderItem_rack aisle(::robot_interfaces::msg::OrderItem::_aisle_type arg)
  {
    msg_.aisle = std::move(arg);
    return Init_OrderItem_rack(msg_);
  }

private:
  ::robot_interfaces::msg::OrderItem msg_;
};

class Init_OrderItem_name
{
public:
  explicit Init_OrderItem_name(::robot_interfaces::msg::OrderItem & msg)
  : msg_(msg)
  {}
  Init_OrderItem_aisle name(::robot_interfaces::msg::OrderItem::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_OrderItem_aisle(msg_);
  }

private:
  ::robot_interfaces::msg::OrderItem msg_;
};

class Init_OrderItem_product_id
{
public:
  Init_OrderItem_product_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrderItem_name product_id(::robot_interfaces::msg::OrderItem::_product_id_type arg)
  {
    msg_.product_id = std::move(arg);
    return Init_OrderItem_name(msg_);
  }

private:
  ::robot_interfaces::msg::OrderItem msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::msg::OrderItem>()
{
  return robot_interfaces::msg::builder::Init_OrderItem_product_id();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__ORDER_ITEM__BUILDER_HPP_
