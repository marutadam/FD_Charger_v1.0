
#ifndef PARAM_TYPES_H
#define PARAM_TYPES_H

typedef struct {
    uint8_t can_id;          // 1 byte
    uint16_t Kp;             // 2 bytes
    float enable_thresh;   // 4 bytes
    float disable_thresh;  // 4 bytes
    uint8_t duty_max;      // 1 byte
    uint16_t min_on_ms;    // 2 bytes
    float storage_volt;    // 4 bytes
} FlashParams;


/** Configuration for the P regulator with hysteresis and min on-time */
typedef struct {
    float Kp;              // proportional gain (duty % per volt)
    float Ki;              // integral gain (duty % per volt-second)
    float Kd;              // derivative gain (duty % per volt per second)
    float enable_thresh;   // volts above target to start balancing
    float disable_thresh;  // volts above target to stop balancing (hysteresis)
    uint16_t min_on_ms;    // minimum on time in milliseconds
    uint8_t duty_max;      // maximum duty percent (0-100)
    float integral_limit;  // clamp for integral term (volt-seconds)
    float storage_volt;  // voltage to maintain when charging is done
} BalanceControllerCfg;

#endif // PARAM_TYPES_H
