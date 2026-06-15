// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from jetson_camera_interfaces:msg/ProcessedImagePair.idl
// generated code does not contain a copyright notice

#ifndef JETSON_CAMERA_INTERFACES__MSG__DETAIL__PROCESSED_IMAGE_PAIR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define JETSON_CAMERA_INTERFACES__MSG__DETAIL__PROCESSED_IMAGE_PAIR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "jetson_camera_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "jetson_camera_interfaces/msg/detail/processed_image_pair__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace jetson_camera_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jetson_camera_interfaces
cdr_serialize(
  const jetson_camera_interfaces::msg::ProcessedImagePair & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jetson_camera_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  jetson_camera_interfaces::msg::ProcessedImagePair & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jetson_camera_interfaces
get_serialized_size(
  const jetson_camera_interfaces::msg::ProcessedImagePair & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jetson_camera_interfaces
max_serialized_size_ProcessedImagePair(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace jetson_camera_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jetson_camera_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, jetson_camera_interfaces, msg, ProcessedImagePair)();

#ifdef __cplusplus
}
#endif

#endif  // JETSON_CAMERA_INTERFACES__MSG__DETAIL__PROCESSED_IMAGE_PAIR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
