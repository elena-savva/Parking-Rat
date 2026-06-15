// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from jetson_camera_interfaces:msg/ProcessedImagePair.idl
// generated code does not contain a copyright notice

#ifndef JETSON_CAMERA_INTERFACES__MSG__DETAIL__PROCESSED_IMAGE_PAIR__FUNCTIONS_H_
#define JETSON_CAMERA_INTERFACES__MSG__DETAIL__PROCESSED_IMAGE_PAIR__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "jetson_camera_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "jetson_camera_interfaces/msg/detail/processed_image_pair__struct.h"

/// Initialize msg/ProcessedImagePair message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * jetson_camera_interfaces__msg__ProcessedImagePair
 * )) before or use
 * jetson_camera_interfaces__msg__ProcessedImagePair__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_camera_interfaces
bool
jetson_camera_interfaces__msg__ProcessedImagePair__init(jetson_camera_interfaces__msg__ProcessedImagePair * msg);

/// Finalize msg/ProcessedImagePair message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_camera_interfaces
void
jetson_camera_interfaces__msg__ProcessedImagePair__fini(jetson_camera_interfaces__msg__ProcessedImagePair * msg);

/// Create msg/ProcessedImagePair message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * jetson_camera_interfaces__msg__ProcessedImagePair__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_camera_interfaces
jetson_camera_interfaces__msg__ProcessedImagePair *
jetson_camera_interfaces__msg__ProcessedImagePair__create();

/// Destroy msg/ProcessedImagePair message.
/**
 * It calls
 * jetson_camera_interfaces__msg__ProcessedImagePair__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_camera_interfaces
void
jetson_camera_interfaces__msg__ProcessedImagePair__destroy(jetson_camera_interfaces__msg__ProcessedImagePair * msg);

/// Check for msg/ProcessedImagePair message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_camera_interfaces
bool
jetson_camera_interfaces__msg__ProcessedImagePair__are_equal(const jetson_camera_interfaces__msg__ProcessedImagePair * lhs, const jetson_camera_interfaces__msg__ProcessedImagePair * rhs);

/// Copy a msg/ProcessedImagePair message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_camera_interfaces
bool
jetson_camera_interfaces__msg__ProcessedImagePair__copy(
  const jetson_camera_interfaces__msg__ProcessedImagePair * input,
  jetson_camera_interfaces__msg__ProcessedImagePair * output);

/// Initialize array of msg/ProcessedImagePair messages.
/**
 * It allocates the memory for the number of elements and calls
 * jetson_camera_interfaces__msg__ProcessedImagePair__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_camera_interfaces
bool
jetson_camera_interfaces__msg__ProcessedImagePair__Sequence__init(jetson_camera_interfaces__msg__ProcessedImagePair__Sequence * array, size_t size);

/// Finalize array of msg/ProcessedImagePair messages.
/**
 * It calls
 * jetson_camera_interfaces__msg__ProcessedImagePair__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_camera_interfaces
void
jetson_camera_interfaces__msg__ProcessedImagePair__Sequence__fini(jetson_camera_interfaces__msg__ProcessedImagePair__Sequence * array);

/// Create array of msg/ProcessedImagePair messages.
/**
 * It allocates the memory for the array and calls
 * jetson_camera_interfaces__msg__ProcessedImagePair__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_camera_interfaces
jetson_camera_interfaces__msg__ProcessedImagePair__Sequence *
jetson_camera_interfaces__msg__ProcessedImagePair__Sequence__create(size_t size);

/// Destroy array of msg/ProcessedImagePair messages.
/**
 * It calls
 * jetson_camera_interfaces__msg__ProcessedImagePair__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_camera_interfaces
void
jetson_camera_interfaces__msg__ProcessedImagePair__Sequence__destroy(jetson_camera_interfaces__msg__ProcessedImagePair__Sequence * array);

/// Check for msg/ProcessedImagePair message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_camera_interfaces
bool
jetson_camera_interfaces__msg__ProcessedImagePair__Sequence__are_equal(const jetson_camera_interfaces__msg__ProcessedImagePair__Sequence * lhs, const jetson_camera_interfaces__msg__ProcessedImagePair__Sequence * rhs);

/// Copy an array of msg/ProcessedImagePair messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_camera_interfaces
bool
jetson_camera_interfaces__msg__ProcessedImagePair__Sequence__copy(
  const jetson_camera_interfaces__msg__ProcessedImagePair__Sequence * input,
  jetson_camera_interfaces__msg__ProcessedImagePair__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // JETSON_CAMERA_INTERFACES__MSG__DETAIL__PROCESSED_IMAGE_PAIR__FUNCTIONS_H_
