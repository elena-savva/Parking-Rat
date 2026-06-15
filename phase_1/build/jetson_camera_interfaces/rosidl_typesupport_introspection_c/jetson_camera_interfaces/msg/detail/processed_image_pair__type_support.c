// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from jetson_camera_interfaces:msg/ProcessedImagePair.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "jetson_camera_interfaces/msg/detail/processed_image_pair__rosidl_typesupport_introspection_c.h"
#include "jetson_camera_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "jetson_camera_interfaces/msg/detail/processed_image_pair__functions.h"
#include "jetson_camera_interfaces/msg/detail/processed_image_pair__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `raw_image`
// Member `undistorted_image`
#include "sensor_msgs/msg/compressed_image.h"
// Member `raw_image`
// Member `undistorted_image`
#include "sensor_msgs/msg/detail/compressed_image__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ProcessedImagePair__rosidl_typesupport_introspection_c__ProcessedImagePair_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  jetson_camera_interfaces__msg__ProcessedImagePair__init(message_memory);
}

void ProcessedImagePair__rosidl_typesupport_introspection_c__ProcessedImagePair_fini_function(void * message_memory)
{
  jetson_camera_interfaces__msg__ProcessedImagePair__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ProcessedImagePair__rosidl_typesupport_introspection_c__ProcessedImagePair_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jetson_camera_interfaces__msg__ProcessedImagePair, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "raw_image",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jetson_camera_interfaces__msg__ProcessedImagePair, raw_image),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "undistorted_image",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jetson_camera_interfaces__msg__ProcessedImagePair, undistorted_image),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ProcessedImagePair__rosidl_typesupport_introspection_c__ProcessedImagePair_message_members = {
  "jetson_camera_interfaces__msg",  // message namespace
  "ProcessedImagePair",  // message name
  3,  // number of fields
  sizeof(jetson_camera_interfaces__msg__ProcessedImagePair),
  ProcessedImagePair__rosidl_typesupport_introspection_c__ProcessedImagePair_message_member_array,  // message members
  ProcessedImagePair__rosidl_typesupport_introspection_c__ProcessedImagePair_init_function,  // function to initialize message memory (memory has to be allocated)
  ProcessedImagePair__rosidl_typesupport_introspection_c__ProcessedImagePair_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ProcessedImagePair__rosidl_typesupport_introspection_c__ProcessedImagePair_message_type_support_handle = {
  0,
  &ProcessedImagePair__rosidl_typesupport_introspection_c__ProcessedImagePair_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_jetson_camera_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jetson_camera_interfaces, msg, ProcessedImagePair)() {
  ProcessedImagePair__rosidl_typesupport_introspection_c__ProcessedImagePair_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ProcessedImagePair__rosidl_typesupport_introspection_c__ProcessedImagePair_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, CompressedImage)();
  ProcessedImagePair__rosidl_typesupport_introspection_c__ProcessedImagePair_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, CompressedImage)();
  if (!ProcessedImagePair__rosidl_typesupport_introspection_c__ProcessedImagePair_message_type_support_handle.typesupport_identifier) {
    ProcessedImagePair__rosidl_typesupport_introspection_c__ProcessedImagePair_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ProcessedImagePair__rosidl_typesupport_introspection_c__ProcessedImagePair_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
