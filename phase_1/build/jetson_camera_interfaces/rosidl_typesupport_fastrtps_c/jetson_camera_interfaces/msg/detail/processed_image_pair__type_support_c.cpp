// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from jetson_camera_interfaces:msg/ProcessedImagePair.idl
// generated code does not contain a copyright notice
#include "jetson_camera_interfaces/msg/detail/processed_image_pair__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "jetson_camera_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "jetson_camera_interfaces/msg/detail/processed_image_pair__struct.h"
#include "jetson_camera_interfaces/msg/detail/processed_image_pair__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "sensor_msgs/msg/detail/compressed_image__functions.h"  // raw_image, undistorted_image
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_jetson_camera_interfaces
size_t get_serialized_size_sensor_msgs__msg__CompressedImage(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_jetson_camera_interfaces
size_t max_serialized_size_sensor_msgs__msg__CompressedImage(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_jetson_camera_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sensor_msgs, msg, CompressedImage)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_jetson_camera_interfaces
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_jetson_camera_interfaces
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_jetson_camera_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _ProcessedImagePair__ros_msg_type = jetson_camera_interfaces__msg__ProcessedImagePair;

static bool _ProcessedImagePair__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ProcessedImagePair__ros_msg_type * ros_message = static_cast<const _ProcessedImagePair__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: raw_image
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, CompressedImage
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->raw_image, cdr))
    {
      return false;
    }
  }

  // Field name: undistorted_image
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, CompressedImage
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->undistorted_image, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _ProcessedImagePair__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ProcessedImagePair__ros_msg_type * ros_message = static_cast<_ProcessedImagePair__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: raw_image
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, CompressedImage
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->raw_image))
    {
      return false;
    }
  }

  // Field name: undistorted_image
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, CompressedImage
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->undistorted_image))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_jetson_camera_interfaces
size_t get_serialized_size_jetson_camera_interfaces__msg__ProcessedImagePair(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ProcessedImagePair__ros_msg_type * ros_message = static_cast<const _ProcessedImagePair__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name raw_image

  current_alignment += get_serialized_size_sensor_msgs__msg__CompressedImage(
    &(ros_message->raw_image), current_alignment);
  // field.name undistorted_image

  current_alignment += get_serialized_size_sensor_msgs__msg__CompressedImage(
    &(ros_message->undistorted_image), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _ProcessedImagePair__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_jetson_camera_interfaces__msg__ProcessedImagePair(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_jetson_camera_interfaces
size_t max_serialized_size_jetson_camera_interfaces__msg__ProcessedImagePair(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_std_msgs__msg__Header(
        full_bounded, current_alignment);
    }
  }
  // member: raw_image
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_sensor_msgs__msg__CompressedImage(
        full_bounded, current_alignment);
    }
  }
  // member: undistorted_image
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_sensor_msgs__msg__CompressedImage(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _ProcessedImagePair__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_jetson_camera_interfaces__msg__ProcessedImagePair(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_ProcessedImagePair = {
  "jetson_camera_interfaces::msg",
  "ProcessedImagePair",
  _ProcessedImagePair__cdr_serialize,
  _ProcessedImagePair__cdr_deserialize,
  _ProcessedImagePair__get_serialized_size,
  _ProcessedImagePair__max_serialized_size
};

static rosidl_message_type_support_t _ProcessedImagePair__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ProcessedImagePair,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, jetson_camera_interfaces, msg, ProcessedImagePair)() {
  return &_ProcessedImagePair__type_support;
}

#if defined(__cplusplus)
}
#endif
