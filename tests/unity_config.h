/* unity_config.h — pio_sim Unity configuration */
#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

/* Use 32-bit integers for assertions (matches embedded target) */
#define UNITY_INT_WIDTH 32

/* Use float, not double — matches Cortex-M33 hardware FPU */
#define UNITY_EXCLUDE_DOUBLE

/* Print fixture name with results for clarity */
#define UNITY_INCLUDE_PRINT_FORMATTED

#endif /* UNITY_CONFIG_H */
