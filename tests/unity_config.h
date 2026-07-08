/* unity_config.h — pio_sim Unity configuration */
#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

/* Use 32-bit integers for assertions (the simulated registers are 32-bit) */
#define UNITY_INT_WIDTH 32

/* These host-side tests assert no floating-point values; exclude double
 * support to keep the Unity build minimal. */
#define UNITY_EXCLUDE_DOUBLE

/* Print fixture name with results for clarity */
#define UNITY_INCLUDE_PRINT_FORMATTED

#endif /* UNITY_CONFIG_H */
