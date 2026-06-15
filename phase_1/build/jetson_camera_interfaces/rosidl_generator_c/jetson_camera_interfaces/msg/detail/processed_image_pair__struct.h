// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jetson_camera_interfaces:msg/ProcessedImagePair.idl
// generated code does not contain a copyright notice

#ifndef JETSON_CAMERA_INTERFACES__MSG__DETAIL__PROCESSED_IMAGE_PAIR__STRUCT_H_
#define JETSON_CAMERA_INTERFACES__MSG__DETAIL__PROCESSED_IMAGE_PAIR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'raw_image'
// Member 'undistorted_image'
#include "sensor_msgs/msg/detail/compressed_image__struct.h"

// Struct defined in msg/ProcessedImagePair in the package jetson_camera_interfaces.
typedef struct jetson_camera_interfaces__msg__ProcessedImagePair
{
  std_msgs__msg__Header header;
  sensor_msgs__msg__CompressedImage raw_image;
  sensor_msgs__msg__CompressedImage undistorted_image;
} jetson_camera_interfaces__msg__ProcessedImagePair;

// Struct for a sequence of jetson_camera_interfaces__msg__ProcessedImagePair.
typedef struct jetson_camera_interfaces__msg__ProcessedImagePair__Sequence
{
  jetson_camera_interfaces__msg__ProcessedImagePair * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetson_camera_interfaces__msg__ProcessedImagePair__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JETSON_CAMERA_INTERFACES__MSG__DETAIL__PROCESSED_IMAGE_PAIR__STRUCT_H_
