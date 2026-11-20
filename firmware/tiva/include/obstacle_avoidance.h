#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H
#include <stdbool.h>
#include <stdint.h>
#define OBSTACLE_MIN_VALID_CM 2.0f
#define OBSTACLE_MAX_VALID_CM 300.0f
typedef enum {
    OBSTACLE_CLEAR = 0,
    OBSTACLE_TURNING = 1,
    OBSTACLE_WAIT_CLEAR = 2
} ObstacleMode;
typedef struct {
    ObstacleMode mode;
    float stop_distance_cm;
    float clear_distance_cm;
    float steering_bias;
    float turn_duration_s;
    float turn_elapsed_s;
    float last_distance_cm;
    int8_t turn_direction;
    int8_t next_turn_direction;
    uint32_t turn_count;
    uint32_t invalid_sample_count;
    bool last_distance_valid;
} ObstacleAvoidance;
void obstacle_avoidance_init(ObstacleAvoidance *nav);
float obstacle_avoidance_step(ObstacleAvoidance *nav, float dt_s,
                              bool sample_ready, bool sample_valid,
                              float distance_cm);
#endif
