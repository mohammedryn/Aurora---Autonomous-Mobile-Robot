idf_component_register(
    SRCS
        "main.c"
        "encoder.c"
        "motor.c"
        "pid.c"
        "tasks/task_encoder_read.c"
        "tasks/task_pid_control.c"
        "tasks/task_imu_read.c"
        "tasks/task_tof_read.c"
        "tasks/task_serial_comms.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer freertos serial_protocol
)
