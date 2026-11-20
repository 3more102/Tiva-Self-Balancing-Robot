#include "obstacle_avoidance.h"
#include <math.h>
#include <stddef.h>
#define NAV_STOP_DISTANCE_CM 25.0f
#define NAV_CLEAR_DISTANCE_CM 40.0f
#define NAV_STEERING_BIAS 0.16f
#define NAV_TURN_DURATION_S 0.65f
#define NAV_MAX_STEP_S 0.020f
void obstacle_avoidance_init(ObstacleAvoidance *nav)
{
    if (nav == NULL) return;
    nav->mode = OBSTACLE_CLEAR;
    nav->stop_distance_cm = NAV_STOP_DISTANCE_CM;
    nav->clear_distance_cm = NAV_CLEAR_DISTANCE_CM;
    nav->steering_bias = NAV_STEERING_BIAS;
    nav->turn_duration_s = NAV_TURN_DURATION_S;
    nav->turn_elapsed_s = 0.0f;
    nav->last_distance_cm = 0.0f;
    nav->turn_direction = 1;
    nav->next_turn_direction = 1;
    nav->turn_count = 0U;
    nav->invalid_sample_count = 0U;
    nav->last_distance_valid = false;
}
float obstacle_avoidance_step(ObstacleAvoidance *nav, float dt_s,
                              bool sample_ready, bool sample_valid,
                              float distance_cm)
{
    if (nav == NULL || !isfinite(dt_s) || dt_s <= 0.0f ||
        dt_s > NAV_MAX_STEP_S) return 0.0f;
    if (sample_ready) {
        if (sample_valid && isfinite(distance_cm) &&
            distance_cm >= OBSTACLE_MIN_VALID_CM &&
            distance_cm <= OBSTACLE_MAX_VALID_CM) {
            nav->last_distance_cm = distance_cm;
            nav->last_distance_valid = true;
        } else {
            nav->last_distance_valid = false;
            ++nav->invalid_sample_count;
        }
    }
    if (nav->mode == OBSTACLE_CLEAR && nav->last_distance_valid &&
        nav->last_distance_cm < nav->stop_distance_cm) {
        nav->mode = OBSTACLE_TURNING;
        nav->turn_elapsed_s = 0.0f;
        nav->turn_direction = nav->next_turn_direction;
        nav->next_turn_direction = (int8_t)(-nav->next_turn_direction);
        ++nav->turn_count;
    }
    if (nav->mode == OBSTACLE_TURNING) {
        nav->turn_elapsed_s += dt_s;
        if (nav->turn_elapsed_s >= nav->turn_duration_s) {
            nav->mode = OBSTACLE_WAIT_CLEAR;
            nav->turn_elapsed_s = nav->turn_duration_s;
            return 0.0f;
        }
        return (float)nav->turn_direction * nav->steering_bias;
    }
    if (nav->mode == OBSTACLE_WAIT_CLEAR &&
        nav->last_distance_valid &&
        nav->last_distance_cm >= nav->clear_distance_cm) {
        nav->mode = OBSTACLE_CLEAR;
    }
    return 0.0f;
}
