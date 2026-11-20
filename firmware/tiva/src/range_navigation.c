#include "range_navigation.h"

#include <stddef.h>

static void copy_sample(const RangeSample *sample, bool *valid,
                        uint16_t *distance_mm, uint32_t *sample_ms)
{
    *valid = sample->has_sample && sample->valid;
    *distance_mm = *valid ? sample->distance_mm : 0U;
    *sample_ms = sample->sample_ms;
}

bool range_navigation_build_input(const RangeScanner *scanner,
                                  bool autonomy_enabled,
                                  NavigationInput *input)
{
    RangeSample front;
    RangeSample left;
    RangeSample right;

    if (scanner == NULL || input == NULL) return false;

    if (!range_scanner_get_sample(scanner, RANGE_SENSOR_FRONT, &front) ||
        !range_scanner_get_sample(scanner, RANGE_SENSOR_LEFT, &left) ||
        !range_scanner_get_sample(scanner, RANGE_SENSOR_RIGHT, &right)) {
        return false;
    }

    input->autonomy_enabled = autonomy_enabled;
    copy_sample(&front, &input->front_valid, &input->front_mm,
                &input->front_sample_ms);
    copy_sample(&left, &input->left_valid, &input->left_mm,
                &input->left_sample_ms);
    copy_sample(&right, &input->right_valid, &input->right_mm,
                &input->right_sample_ms);
    return true;
}
