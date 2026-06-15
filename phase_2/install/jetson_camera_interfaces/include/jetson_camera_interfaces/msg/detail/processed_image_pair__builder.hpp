// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jetson_camera_interfaces:msg/ProcessedImagePair.idl
// generated code does not contain a copyright notice

#ifndef JETSON_CAMERA_INTERFACES__MSG__DETAIL__PROCESSED_IMAGE_PAIR__BUILDER_HPP_
#define JETSON_CAMERA_INTERFACES__MSG__DETAIL__PROCESSED_IMAGE_PAIR__BUILDER_HPP_

#include "jetson_camera_interfaces/msg/detail/processed_image_pair__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace jetson_camera_interfaces
{

namespace msg
{

namespace builder
{

class Init_ProcessedImagePair_undistorted_image
{
public:
  explicit Init_ProcessedImagePair_undistorted_image(::jetson_camera_interfaces::msg::ProcessedImagePair & msg)
  : msg_(msg)
  {}
  ::jetson_camera_interfaces::msg::ProcessedImagePair undistorted_image(::jetson_camera_interfaces::msg::ProcessedImagePair::_undistorted_image_type arg)
  {
    msg_.undistorted_image = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetson_camera_interfaces::msg::ProcessedImagePair msg_;
};

class Init_ProcessedImagePair_raw_image
{
public:
  explicit Init_ProcessedImagePair_raw_image(::jetson_camera_interfaces::msg::ProcessedImagePair & msg)
  : msg_(msg)
  {}
  Init_ProcessedImagePair_undistorted_image raw_image(::jetson_camera_interfaces::msg::ProcessedImagePair::_raw_image_type arg)
  {
    msg_.raw_image = std::move(arg);
    return Init_ProcessedImagePair_undistorted_image(msg_);
  }

private:
  ::jetson_camera_interfaces::msg::ProcessedImagePair msg_;
};

class Init_ProcessedImagePair_header
{
public:
  Init_ProcessedImagePair_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ProcessedImagePair_raw_image header(::jetson_camera_interfaces::msg::ProcessedImagePair::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ProcessedImagePair_raw_image(msg_);
  }

private:
  ::jetson_camera_interfaces::msg::ProcessedImagePair msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetson_camera_interfaces::msg::ProcessedImagePair>()
{
  return jetson_camera_interfaces::msg::builder::Init_ProcessedImagePair_header();
}

}  // namespace jetson_camera_interfaces

#endif  // JETSON_CAMERA_INTERFACES__MSG__DETAIL__PROCESSED_IMAGE_PAIR__BUILDER_HPP_
