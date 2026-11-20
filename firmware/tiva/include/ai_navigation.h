#ifndef GYROGUARD_AI_NAVIGATION_H
#define GYROGUARD_AI_NAVIGATION_H

#include <stdbool.h>
#include <stdint.h>

#define AI_FRONT_MAX_AGE_MS 250U
#define AI_SIDE_MAX_AGE_MS 750U
#define AI_FRONT_CLEAR_MM 500U
#define AI_SIDE_CLEAR_MM 500U
#define AI_MAX_DISTANCE_MM 2000.0f
#define AI_MAX_CLOSING_MPS 2.0f
#define AI_MAX_YAW_DPS 180.0f
#define AI_MIN_CONFIDENCE 0.40f

typedef enum {
    AI_ACTION_FORWARD = 0,
    AI_ACTION_TURN_LEFT = 1,
    AI_ACTION_TURN_RIGHT = 2,
    AI_ACTION_STOP = 3,
    AI_ACTION_REASSESS = 4
} AiAction;

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
    float yaw_rate_dps;
} AiSensorFrame;

typedef struct {
    bool has_previous_front;
    uint16_t previous_front_mm;
    uint32_t previous_front_sample_ms;
} AiNavigationState;

typedef struct {
    AiAction action;
    float confidence;
    float probabilities[5];
    float features[5];
    bool safety_override;
} AiNavigationCommand;

void ai_navigation_init(AiNavigationState *state);
AiNavigationCommand ai_navigation_step(AiNavigationState *state,
                                       const AiSensorFrame *frame,
                                       uint32_t now_ms);

/* Native-C inference API. Inputs are normalized to [0,1], [-1,1]. */
bool ai_model_predict(const float features[5], float probabilities[5]);

#endif /* GYROGUARD_AI_NAVIGATION_H */
