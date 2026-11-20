#ifndef TIVA_RANGE_NAVIGATION_H
#define TIVA_RANGE_NAVIGATION_H

#include <stdbool.h>

#include "navigation.h"
#include "range_scanner.h"

/*
 * Copy the scanner's latest front/left/right results into NavigationInput.
 * The caller supplies the separate autonomy-enable decision. Missing or
 * invalid sensor samples are always marked invalid with distance zero.
 *
 * Freshness is intentionally checked by navigation_update() using sample_ms
 * and its own per-sensor age limits.
 */
bool range_navigation_build_input(const RangeScanner *scanner,
                                  bool autonomy_enabled,
                                  NavigationInput *input);

#endif
