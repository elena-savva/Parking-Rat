// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jetson_camera_interfaces:msg/ProcessedImagePair.idl
// generated code does not contain a copyright notice

#ifndef JETSON_CAMERA_INTERFACES__MSG__DETAIL__PROCESSED_IMAGE_PAIR__TRAITS_HPP_
#define JETSON_CAMERA_INTERFACES__MSG__DETAIL__PROCESSED_IMAGE_PAIR__TRAITS_HPP_

#include "jetson_camera_interfaces/msg/detail/processed_image_pair__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'raw_image'
// Member 'undistorted_image'
#include "sensor_msgs/msg/detail/compressed_image__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<jetson_camera_interfaces::msg::ProcessedImagePair>()
{
  return "jetson_camera_interfaces::msg::ProcessedImagePair";
}

template<>
inline const char * name<jetson_camera_interfaces::msg::ProcessedImagePair>()
{
  return "jetson_camera_interfaces/msg/ProcessedImagePair";
}

template<>
struct has_fixed_size<jetson_camera_interfaces::msg::ProcessedImagePair>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::CompressedImage>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<jetson_camera_interfaces::msg::ProcessedImagePair>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::CompressedImage>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<jetson_camera_interfaces::msg::ProcessedImagePair>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // JETSON_CAMERA_INTERFACES__MSG__DETAIL__PROCESSED_IMAGE_PAIR__TRAITS_HPP_
