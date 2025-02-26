// Copyright (c) 2025 Carologistics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "stm32h7xx_hal.h"

#include <cstring>
#include <limits.h>
#include <shared_mem_data.h>

#if COMMAND_MAX_BYTES + FEEDBACK_MAX_BYTES > 1024
#error Shrared memory buffer size exceeds 1024 bytes
#endif

#define M4_SEMAPHORE_INDEX 2

#define M7_SEMAPHORE_INDEX 3

#if defined(CORE_CM7)
#define CORE_ID 0 // Process ID for CM7
#elif defined(CORE_CM4)
#define CORE_ID 1 // Process ID for CM4
#else
#error "Unknown core type"
#endif
struct _shared {
  uint8_t m4_buffer[M4_BUFFER_SIZE], m7_buffer[M7_BUFFER_SIZE];
  int m4_buffer_size, m7_buffer_size;
};
#include <stdint.h>
#include <stdio.h>

// Extern declarations for the start and end of the shared memory region
extern uint32_t __shared_start__;
extern uint32_t __shared_end__;

/** Setup for SRAM3
 *
 * */
#if defined(CORE_CM7)
struct _shared *_shared_data_start = (_shared *)(0x30046000);
#else
struct _shared *_shared_data_start = (_shared *)(0x10046000);
#endif

/** Setup for SRAM4
#if defined(CORE_CM7)
struct _shared *_shared_data_start = (_shared *) (0x38000000);
#else
struct _shared *_shared_data_start = (_shared *)(0x38000000);
#endif
*/

void MPU_Config(void) {
  MPU_Region_InitTypeDef MPU_InitStruct;

  // Disable MPU to configure it
  HAL_MPU_Disable();

  // Configure shared memory as non-cacheable
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
#if defined(CORE_CM7)
  MPU_InitStruct.Number = MPU_REGION_NUMBER7;
#else
  MPU_InitStruct.Number = MPU_REGION_NUMBER7;
#endif
  MPU_InitStruct.BaseAddress = (uint32_t)_shared_data_start;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0; // TODO: maybe TEX level 0?
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

#if defined(CORE_CM7)
  // Clean and invalidate the data cache for the shared region
  SCB_CleanDCache_by_Addr((uint32_t *)_shared_data_start,
                          sizeof(*_shared_data_start));
  SCB_InvalidateDCache_by_Addr((uint32_t *)_shared_data_start,
                               sizeof(*_shared_data_start));
#endif
  // Enable MPU again
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

uint32_t mem_shared_data() { return (uint32_t)_shared_data_start; }

uint32_t mem_m4_data() { return (uint32_t) & (_shared_data_start->m4_buffer); }

uint32_t mem_m7_data() { return (uint32_t) & (_shared_data_start->m7_buffer); }

uint32_t mem_m4_data_size() {
  return (uint32_t) & (_shared_data_start->m4_buffer_size);
}

uint32_t mem_m7_data_size() {
  return (uint32_t) & (_shared_data_start->m7_buffer_size);
}

void core_share_init(void) {
  _shared_data_start->m4_buffer_size = 0;
  _shared_data_start->m7_buffer_size = 0;
#if defined(CORE_CM7)
  // Clean and invalidate the data cache for the shared region
  SCB_CleanDCache_by_Addr((uint32_t *)_shared_data_start,
                          sizeof(*_shared_data_start));
  SCB_InvalidateDCache_by_Addr((uint32_t *)_shared_data_start,
                               sizeof(*_shared_data_start));
#endif
  HAL_HSEM_Release(M4_SEMAPHORE_INDEX, CORE_ID);
  HAL_HSEM_Release(M7_SEMAPHORE_INDEX, CORE_ID);
}
void HSEM_Init(void) {
  MPU_Config();
  // Enable HSEM Clock (typically already enabled)
  __HAL_RCC_HSEM_CLK_ENABLE();
}
int m7_has_data(void) {
  int n_items;

  // Try to acquire the semaphore
  if (HAL_HSEM_Take(M7_SEMAPHORE_INDEX, CORE_ID) != HAL_OK) {
    // Semaphore is already taken by another core
    return -1;
  }

  // Critical section
  n_items = _shared_data_start->m7_buffer_size;

  // Release the semaphore
  HAL_HSEM_Release(M7_SEMAPHORE_INDEX, CORE_ID);

  return n_items;
}

int get_from_m7(void *buffer, int size) {
  /* Try to acquire the semaphore */
  if (HAL_HSEM_Take(M7_SEMAPHORE_INDEX, CORE_ID) != HAL_OK) {
    /* Return error code if semaphore is already taken */
    return -1; // Semaphore not acquired
  }

  if (_shared_data_start->m7_buffer_size == 0) {
    HAL_HSEM_Release(M7_SEMAPHORE_INDEX, CORE_ID);
    return 0; // No data available
  }

  /* Limit size to the number of items available in the buffer */
  if (size >= _shared_data_start->m7_buffer_size) {
    size = _shared_data_start->m7_buffer_size;
  }

  /* Copy data from shared memory buffer to local buffer */
  memcpy(buffer, _shared_data_start->m7_buffer, size);

  /* Clear the shared buffer size (no more data available) */
  _shared_data_start->m7_buffer_size = 0;

  /* Release the semaphore */
  HAL_HSEM_Release(M7_SEMAPHORE_INDEX, CORE_ID);

  /* Return the number of items read */
  return size;
}

int put_to_m7(const void *data, const int size) {

#if defined(CORE_CM7)
  SCB_InvalidateDCache_by_Addr((uint32_t *)_shared_data_start,
                               sizeof(*_shared_data_start));
#endif
  /* Try to acquire the semaphore */
  if (HAL_HSEM_Take(M4_SEMAPHORE_INDEX, CORE_ID) != HAL_OK) {
    /* Return error code if semaphore is already taken */
    return -1; // Semaphore not acquired
  }

  /* Ensure the data size fits within the shared buffer */
  if (size > M4_BUFFER_SIZE * sizeof(int)) {
    /* Release semaphore before returning */
    HAL_HSEM_Release(M4_SEMAPHORE_INDEX, CORE_ID);
    return -1; // Data size exceeds buffer capacity
  }
  __DMB(); // Ensure all previous memory accesses are completed
  __DSB(); // Ensure synchronization before proceeding
  /* Copy the data into the shared memory buffer */
  memcpy(_shared_data_start->m4_buffer, data, size);

  /* Store the actual size of the data being transferred */
  _shared_data_start->m4_buffer_size = size;

#if defined(CORE_CM7)
  SCB_InvalidateDCache_by_Addr((uint32_t *)_shared_data_start,
                               sizeof(*_shared_data_start));
#endif
  /* Release the semaphore */
  HAL_HSEM_Release(M4_SEMAPHORE_INDEX, CORE_ID);

  /* Return the number of bytes transferred */
  return size;
}

int m4_has_data(void) {
  int n_items;

  // Try to acquire the semaphore
  if (HAL_HSEM_Take(M4_SEMAPHORE_INDEX, CORE_ID) != HAL_OK) {
    // Semaphore is already taken by another core
    return -1;
  }
#if defined(CORE_CM7)

  SCB_InvalidateDCache_by_Addr((uint32_t *)_shared_data_start,
                               sizeof(*_shared_data_start));

  SCB_CleanDCache_by_Addr((uint32_t *)_shared_data_start,
                          sizeof(*_shared_data_start));
#endif
  __DMB(); // Data Memory Barrier
  __DSB(); // Data Synchronization Barrier
  __ISB(); // Instruction Synchronization Barrier

  // Critical section
  n_items = _shared_data_start->m4_buffer_size;

  // Release the semaphore
  HAL_HSEM_Release(M4_SEMAPHORE_INDEX, CORE_ID);

  return n_items;
}

int get_from_m4(void *data, int size) {
  /* Try to acquire the semaphore */
  if (HAL_HSEM_Take(M4_SEMAPHORE_INDEX, CORE_ID) != HAL_OK) {
    /* Return error code if semaphore is already taken */
    return -1; // Semaphore not acquired
  }
#if defined(CORE_CM7)

  SCB_InvalidateDCache_by_Addr((uint32_t *)_shared_data_start,
                               sizeof(*_shared_data_start));

  SCB_CleanDCache_by_Addr((uint32_t *)_shared_data_start,
                          sizeof(*_shared_data_start));
#endif
  __DMB(); // Data Memory Barrier
  __DSB(); // Data Synchronization Barrier
  __ISB(); // Instruction Synchronization Barrier
  if (_shared_data_start->m4_buffer_size == 0) {
    HAL_HSEM_Release(M4_SEMAPHORE_INDEX, CORE_ID);
    return 0; // No data available
  }

  /* Ensure the size does not exceed the available data size in shared memory */
  if (size > _shared_data_start->m4_buffer_size) {
    size = _shared_data_start->m4_buffer_size; // Adjust size to available data
  }

  /* Copy data from shared memory buffer to local buffer */
  memcpy(data, _shared_data_start->m4_buffer, size);
  _shared_data_start->m4_buffer_size = 0; // Clear the buffer
#if defined(CORE_CM7)

  SCB_InvalidateDCache_by_Addr((uint32_t *)_shared_data_start,
                               sizeof(*_shared_data_start));

  SCB_CleanDCache_by_Addr((uint32_t *)_shared_data_start,
                          sizeof(*_shared_data_start));
#endif
  __DMB(); // Data Memory Barrier
  __DSB(); // Data Synchronization Barrier
  __ISB(); // Instruction Synchronization Barrier

  /* Release the semaphore */
  HAL_HSEM_Release(M4_SEMAPHORE_INDEX, CORE_ID);

  /* Return the number of bytes read */
  return size;
}

int put_to_m4(const void *buffer, const int size) {
  /* Try to acquire the semaphore */
  if (HAL_HSEM_Take(M7_SEMAPHORE_INDEX, CORE_ID) != HAL_OK) {
    /* Return error code if semaphore is already taken */
    return -1; // Semaphore not acquired
  }

  /* Limit the size of the buffer to the destination buffer's capacity */
  _shared_data_start->m7_buffer_size =
      (size <= M7_BUFFER_SIZE) ? size : M7_BUFFER_SIZE;

  /* Copy from source buffer to shared memory buffer */
  memcpy(_shared_data_start->m7_buffer, buffer,
         _shared_data_start->m7_buffer_size);

  /* Release the semaphore */
  HAL_HSEM_Release(M7_SEMAPHORE_INDEX, CORE_ID);

  /* Return the number of items transferred */
  return _shared_data_start->m7_buffer_size;
}
