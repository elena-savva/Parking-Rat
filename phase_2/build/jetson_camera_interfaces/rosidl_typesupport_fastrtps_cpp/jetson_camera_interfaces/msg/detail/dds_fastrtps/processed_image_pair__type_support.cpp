// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from jetson_camera_interfaces:msg/ProcessedImagePair.idl
// generated code does not contain a copyright notice
#include "jetson_camera_interfaces/msg/detail/processed_image_pair__rosidl_typesupport_fastrtps_cpp.hpp"
#include "jetson_camera_interfaces/msg/detail/processed_image_pair__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs

namespace sensor_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const sensor_msgs::msg::CompressedImage &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  sensor_msgs::msg::CompressedImage &);
size_t get_serialized_size(
  const sensor_msgs::msg::CompressedImage &,
  size_t current_alignment);
size_t
max_serialized_size_CompressedImage(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace sensor_msgs

namespace sensor_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const sensor_msgs::msg::CompressedImage &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  sensor_msgs::msg::CompressedImage &);
size_t get_serialized_size(
  const sensor_msgs::msg::CompressedImage &,
  size_t current_alignment);
size_t
max_serialized_size_CompressedImage(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace sensor_msgs


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
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: raw_image
  sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.raw_image,
    cdr);
  // Member: undistorted_image
  sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.undistorted_image,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jetson_camera_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  jetson_camera_interfaces::msg::ProcessedImagePair & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: raw_image
  sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.raw_image);

  // Member: undistorted_image
  sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.undistorted_image);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jetson_camera_interfaces
get_serialized_size(
  const jetson_camera_interfaces::msg::ProcessedImagePair & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: raw_image

  current_alignment +=
    sensor_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.raw_image, current_alignment);
  // Member: undistorted_image

  current_alignment +=
    sensor_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.undistorted_image, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jetson_camera_interfaces
max_serialized_size_ProcessedImagePair(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        full_bounded, current_alignment);
    }
  }

  // Member: raw_image
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        sensor_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_CompressedImage(
        full_bounded, current_alignment);
    }
  }

  // Member: undistorted_image
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        sensor_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_CompressedImage(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _ProcessedImagePair__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const jetson_camera_interfaces::msg::ProcessedImagePair *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ProcessedImagePair__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<jetson_camera_interfaces::msg::ProcessedImagePair *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ProcessedImagePair__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const jetson_camera_interfaces::msg::ProcessedImagePair *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ProcessedImagePair__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_ProcessedImagePair(full_bounded, 0);
}

static message_type_support_callbacks_t _ProcessedImagePair__callbacks = {
  "jetson_camera_interfaces::msg",
  "ProcessedImagePair",
  _ProcessedImagePair__cdr_serialize,
  _ProcessedImagePair__cdr_deserialize,
  _ProcessedImagePair__get_serialized_size,
  _ProcessedImagePair__max_serialized_size
};

static rosidl_message_type_support_t _ProcessedImagePair__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ProcessedImagePair__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace jetson_camera_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_jetson_camera_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<jetson_camera_interfaces::msg::ProcessedImagePair>()
{
  return &jetson_camera_interfaces::msg::typesupport_fastrtps_cpp::_ProcessedImagePair__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, jetson_camera_interfaces, msg, ProcessedImagePair)() {
  return &jetson_camera_interfaces::msg::typesupport_fastrtps_cpp::_ProcessedImagePair__handle;
}

#ifdef __cplusplus
}
#endif
