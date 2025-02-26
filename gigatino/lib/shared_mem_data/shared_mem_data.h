#pragma once

#ifndef SHARED_MEM_DATA_H_
#define SHARED_MEM_DATA_H_

#ifndef COMMAND_MAX_BYTES
#define COMMAND_MAX_BYTES 512
#endif

#ifndef FEEDBACK_MAX_BYTES
#define FEEDBACK_MAX_BYTES 512
#endif

#ifndef M4_BUFFER_SIZE
#define M4_BUFFER_SIZE COMMAND_MAX_BYTES
#endif

#ifndef M7_BUFFER_SIZE
#define M7_BUFFER_SIZE FEEDBACK_MAX_BYTES
#endif

enum CommandID {
  NO_COMMAND = 0,
  MOVE = 1,
  STOP = 2,
  CALIBRATE = 3,
  PID_UPDATE = 4
};

// Ext -Ethernet-> M4 -Shared Mem-> M7
typedef struct command {
  CommandID command_id;
  union {
    float stepper_positions[4];
    float pid_params[4];
  };
  union {
    uint8_t stepper_mask;

    uint8_t pid_motor_id;
  };
  float servo_positions[2];
  union {
    uint8_t servo_mask;

    uint8_t pid_param_mask;
  };
  size_t command_index;
} __attribute__((aligned(8))) Command;
typedef union command_buffer {
  Command cmd;                     // Command structure
  uint8_t buffer[sizeof(Command)]; // Raw data buffer
} CommandBuffer;

// M7 -Shared Mem-> M4 -Ethernet-> Ext
typedef struct feedback {
  float stepper_positions[4];
  float servo_positions[2];
  bool stepper_directions[4];
  bool stepper_endstops[4];
  bool wp_sensor;
  bool busy;
  bool referenced;
  size_t command_index;
} __attribute__((aligned(8))) Feedback;
typedef union feedback_buffer {
  Feedback feedback;                // Command structure
  uint8_t buffer[sizeof(Feedback)]; // Raw data buffer
} FeedbackBuffer;

void MPU_Config(void);
void HSEM_Init(void);

uint32_t mem_m4_data(void);
uint32_t mem_m7_data(void);
uint32_t mem_m4_data_size(void);
uint32_t mem_m7_data_size(void);

/**
 * @brief Used to clear the buffer locks and their size holders
 *        It has to be called in only one of the cores
 */
void core_share_init(void);

/**
 * @brief Send data from M7 to M4
 * @param buffer
 * @param size
 * @return -1 if lock is not acquired, otherwise how many items were transfered
 */
int put_to_m4(const void *buffer, const int size);

/**
 * @brief Get data from M4
 * @param buffer
 * @param size
 * @return -1 if lock is not acquired, otherwise how many bytes were read
 */
int get_from_m4(void *data, int size);

/**
 * @brief Verify whether CM4 has data ready for CM7 to read
 * @return -1 if lock is not acquired, otherwise how many bytes are available
 * for reading
 */
int m4_has_data(void);

/**
 * @brief Send data from M4 to M7
 * @param buffer
 * @param size
 * @return -1 if lock is not acquired, otherwise how many bytes were transfered
 */
int put_to_m7(const void *data, const int size);

/**
 * @brief Get data from M7
 * @param buffer
 * @param size
 * @return -1 if lock is not acquired, otherwise how many bytes were read
 */
int get_from_m7(void *buffer, int size);

/**
 * @brief Verify whether M7 has data ready for M4 to read
 * @return -1 if lock is not acquired, otherwise how many bytes are available
 * for reading
 */
int m7_has_data(void);
#endif
