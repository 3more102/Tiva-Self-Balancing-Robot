#ifndef TIVA_NAVIGATION_H
#define TIVA_NAVIGATION_H

#include <stdbool.h>
#include <stdint.h>

/* Portable obstacle policy. Distances are millimetres; timestamps are ms. */
#define NAV_FRONT_STOP_MM 350U
#define NAV_FRONT_CLEAR_MM 500U
#define NAV_MAX_RANGE_MM 4000U /* Initial sensor-independent upper bound; tune to module datasheet. */
#define NAV_SIDE_CLEAR_MM 500U
#define NAV_FRONT_MAX_AGE_MS 250U
#define NAV_SIDE_MAX_AGE_MS 750U
#define NAV_TURN_DURATION_MS 350U
#define NAV_TURN_RETRY_MS 100U
#define NAV_CLEAR_CONFIRMATIONS 3U
#define NAV_MAX_TURN_ATTEMPTS 2U
#define NAV_CRUISE_LEAN_DEG 1.5f
#define NAV_TURN_BIAS 0.12f

typedef enum {
    NAV_DISABLED = 0,
    NAV_HOLD = 1,
    NAV_CRUISE = 2,
    NAV_TURN_LEFT = 3,
    NAV_TURN_RIGHT = 4
} NavigationMode;

typedef struct {
    bool autonomy_enabled;
    bool front_valid;
    bool left_valid;
    bool right_valid;
    uint16_t front_mm;
    uint16_t left_mm;
    uint16_t right_mm;
    uint32_t front_sample_ms;
    uint32_t left_sample_ms;
    uint32_t right_sample_ms;
} NavigationInput;

typedef struct {
    NavigationMode mode;
    uint32_t state_since_ms;
    uint32_t last_front_sample_ms;
    uint8_t clear_samples;
    uint8_t turn_attempts;
    int8_t last_turn_direction; /* -1 left, +1 right, 0 none */
    bool has_front_sample;
} Navigation;

typedef struct {
    NavigationMode mode;
    float pitch_target_deg;
    float yaw_bias;
    bool travel_permitted;
} NavigationCommand;

void navigation_init(Navigation *navigation);
NavigationCommand navigation_update(Navigation *navigation,
                                    const NavigationInput *input,
                                    uint32_t now_ms);

#endif
