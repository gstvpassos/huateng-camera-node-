// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from huateng_camera_node:srv/SetFloat32.idl
// generated code does not contain a copyright notice
#include "huateng_camera_node/srv/detail/set_float32__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
huateng_camera_node__srv__SetFloat32_Request__init(huateng_camera_node__srv__SetFloat32_Request * msg)
{
  if (!msg) {
    return false;
  }
  // data
  return true;
}

void
huateng_camera_node__srv__SetFloat32_Request__fini(huateng_camera_node__srv__SetFloat32_Request * msg)
{
  if (!msg) {
    return;
  }
  // data
}

bool
huateng_camera_node__srv__SetFloat32_Request__are_equal(const huateng_camera_node__srv__SetFloat32_Request * lhs, const huateng_camera_node__srv__SetFloat32_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data
  if (lhs->data != rhs->data) {
    return false;
  }
  return true;
}

bool
huateng_camera_node__srv__SetFloat32_Request__copy(
  const huateng_camera_node__srv__SetFloat32_Request * input,
  huateng_camera_node__srv__SetFloat32_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // data
  output->data = input->data;
  return true;
}

huateng_camera_node__srv__SetFloat32_Request *
huateng_camera_node__srv__SetFloat32_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  huateng_camera_node__srv__SetFloat32_Request * msg = (huateng_camera_node__srv__SetFloat32_Request *)allocator.allocate(sizeof(huateng_camera_node__srv__SetFloat32_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(huateng_camera_node__srv__SetFloat32_Request));
  bool success = huateng_camera_node__srv__SetFloat32_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
huateng_camera_node__srv__SetFloat32_Request__destroy(huateng_camera_node__srv__SetFloat32_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    huateng_camera_node__srv__SetFloat32_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
huateng_camera_node__srv__SetFloat32_Request__Sequence__init(huateng_camera_node__srv__SetFloat32_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  huateng_camera_node__srv__SetFloat32_Request * data = NULL;

  if (size) {
    data = (huateng_camera_node__srv__SetFloat32_Request *)allocator.zero_allocate(size, sizeof(huateng_camera_node__srv__SetFloat32_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = huateng_camera_node__srv__SetFloat32_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        huateng_camera_node__srv__SetFloat32_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
huateng_camera_node__srv__SetFloat32_Request__Sequence__fini(huateng_camera_node__srv__SetFloat32_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      huateng_camera_node__srv__SetFloat32_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

huateng_camera_node__srv__SetFloat32_Request__Sequence *
huateng_camera_node__srv__SetFloat32_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  huateng_camera_node__srv__SetFloat32_Request__Sequence * array = (huateng_camera_node__srv__SetFloat32_Request__Sequence *)allocator.allocate(sizeof(huateng_camera_node__srv__SetFloat32_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = huateng_camera_node__srv__SetFloat32_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
huateng_camera_node__srv__SetFloat32_Request__Sequence__destroy(huateng_camera_node__srv__SetFloat32_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    huateng_camera_node__srv__SetFloat32_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
huateng_camera_node__srv__SetFloat32_Request__Sequence__are_equal(const huateng_camera_node__srv__SetFloat32_Request__Sequence * lhs, const huateng_camera_node__srv__SetFloat32_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!huateng_camera_node__srv__SetFloat32_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
huateng_camera_node__srv__SetFloat32_Request__Sequence__copy(
  const huateng_camera_node__srv__SetFloat32_Request__Sequence * input,
  huateng_camera_node__srv__SetFloat32_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(huateng_camera_node__srv__SetFloat32_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    huateng_camera_node__srv__SetFloat32_Request * data =
      (huateng_camera_node__srv__SetFloat32_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!huateng_camera_node__srv__SetFloat32_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          huateng_camera_node__srv__SetFloat32_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!huateng_camera_node__srv__SetFloat32_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
huateng_camera_node__srv__SetFloat32_Response__init(huateng_camera_node__srv__SetFloat32_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
huateng_camera_node__srv__SetFloat32_Response__fini(huateng_camera_node__srv__SetFloat32_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
huateng_camera_node__srv__SetFloat32_Response__are_equal(const huateng_camera_node__srv__SetFloat32_Response * lhs, const huateng_camera_node__srv__SetFloat32_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
huateng_camera_node__srv__SetFloat32_Response__copy(
  const huateng_camera_node__srv__SetFloat32_Response * input,
  huateng_camera_node__srv__SetFloat32_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

huateng_camera_node__srv__SetFloat32_Response *
huateng_camera_node__srv__SetFloat32_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  huateng_camera_node__srv__SetFloat32_Response * msg = (huateng_camera_node__srv__SetFloat32_Response *)allocator.allocate(sizeof(huateng_camera_node__srv__SetFloat32_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(huateng_camera_node__srv__SetFloat32_Response));
  bool success = huateng_camera_node__srv__SetFloat32_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
huateng_camera_node__srv__SetFloat32_Response__destroy(huateng_camera_node__srv__SetFloat32_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    huateng_camera_node__srv__SetFloat32_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
huateng_camera_node__srv__SetFloat32_Response__Sequence__init(huateng_camera_node__srv__SetFloat32_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  huateng_camera_node__srv__SetFloat32_Response * data = NULL;

  if (size) {
    data = (huateng_camera_node__srv__SetFloat32_Response *)allocator.zero_allocate(size, sizeof(huateng_camera_node__srv__SetFloat32_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = huateng_camera_node__srv__SetFloat32_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        huateng_camera_node__srv__SetFloat32_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
huateng_camera_node__srv__SetFloat32_Response__Sequence__fini(huateng_camera_node__srv__SetFloat32_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      huateng_camera_node__srv__SetFloat32_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

huateng_camera_node__srv__SetFloat32_Response__Sequence *
huateng_camera_node__srv__SetFloat32_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  huateng_camera_node__srv__SetFloat32_Response__Sequence * array = (huateng_camera_node__srv__SetFloat32_Response__Sequence *)allocator.allocate(sizeof(huateng_camera_node__srv__SetFloat32_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = huateng_camera_node__srv__SetFloat32_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
huateng_camera_node__srv__SetFloat32_Response__Sequence__destroy(huateng_camera_node__srv__SetFloat32_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    huateng_camera_node__srv__SetFloat32_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
huateng_camera_node__srv__SetFloat32_Response__Sequence__are_equal(const huateng_camera_node__srv__SetFloat32_Response__Sequence * lhs, const huateng_camera_node__srv__SetFloat32_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!huateng_camera_node__srv__SetFloat32_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
huateng_camera_node__srv__SetFloat32_Response__Sequence__copy(
  const huateng_camera_node__srv__SetFloat32_Response__Sequence * input,
  huateng_camera_node__srv__SetFloat32_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(huateng_camera_node__srv__SetFloat32_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    huateng_camera_node__srv__SetFloat32_Response * data =
      (huateng_camera_node__srv__SetFloat32_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!huateng_camera_node__srv__SetFloat32_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          huateng_camera_node__srv__SetFloat32_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!huateng_camera_node__srv__SetFloat32_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
