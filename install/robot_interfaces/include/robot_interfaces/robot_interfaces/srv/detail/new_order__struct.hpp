// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_interfaces:srv/NewOrder.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__SRV__DETAIL__NEW_ORDER__STRUCT_HPP_
#define ROBOT_INTERFACES__SRV__DETAIL__NEW_ORDER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'order'
#include "robot_interfaces/msg/detail/order__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_interfaces__srv__NewOrder_Request __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__srv__NewOrder_Request __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct NewOrder_Request_
{
  using Type = NewOrder_Request_<ContainerAllocator>;

  explicit NewOrder_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : order(_init)
  {
    (void)_init;
  }

  explicit NewOrder_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : order(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _order_type =
    robot_interfaces::msg::Order_<ContainerAllocator>;
  _order_type order;

  // setters for named parameter idiom
  Type & set__order(
    const robot_interfaces::msg::Order_<ContainerAllocator> & _arg)
  {
    this->order = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::srv::NewOrder_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::srv::NewOrder_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::srv::NewOrder_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::srv::NewOrder_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::srv::NewOrder_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::srv::NewOrder_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::srv::NewOrder_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::srv::NewOrder_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::srv::NewOrder_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::srv::NewOrder_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__srv__NewOrder_Request
    std::shared_ptr<robot_interfaces::srv::NewOrder_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__srv__NewOrder_Request
    std::shared_ptr<robot_interfaces::srv::NewOrder_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NewOrder_Request_ & other) const
  {
    if (this->order != other.order) {
      return false;
    }
    return true;
  }
  bool operator!=(const NewOrder_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NewOrder_Request_

// alias to use template instance with default allocator
using NewOrder_Request =
  robot_interfaces::srv::NewOrder_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_interfaces


#ifndef _WIN32
# define DEPRECATED__robot_interfaces__srv__NewOrder_Response __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__srv__NewOrder_Response __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct NewOrder_Response_
{
  using Type = NewOrder_Response_<ContainerAllocator>;

  explicit NewOrder_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
      this->message = "";
    }
  }

  explicit NewOrder_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
      this->message = "";
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::srv::NewOrder_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::srv::NewOrder_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::srv::NewOrder_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::srv::NewOrder_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::srv::NewOrder_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::srv::NewOrder_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::srv::NewOrder_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::srv::NewOrder_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::srv::NewOrder_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::srv::NewOrder_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__srv__NewOrder_Response
    std::shared_ptr<robot_interfaces::srv::NewOrder_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__srv__NewOrder_Response
    std::shared_ptr<robot_interfaces::srv::NewOrder_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NewOrder_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const NewOrder_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NewOrder_Response_

// alias to use template instance with default allocator
using NewOrder_Response =
  robot_interfaces::srv::NewOrder_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_interfaces

namespace robot_interfaces
{

namespace srv
{

struct NewOrder
{
  using Request = robot_interfaces::srv::NewOrder_Request;
  using Response = robot_interfaces::srv::NewOrder_Response;
};

}  // namespace srv

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__SRV__DETAIL__NEW_ORDER__STRUCT_HPP_
