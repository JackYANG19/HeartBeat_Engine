#pragma once

#include <stdint.h>

/**
 * @brief Initialize the DRV2605L haptic driver.
 *        Must be called after i2c_master_init().
 */
void haptic_init(void);

/**
 * @brief Play one of the built-in haptic effects.
 * @param effect_id  ID of the effect (0–123 in the library).
 */
void haptic_play_effect(uint8_t effect_id);

/**
 * @brief Stop any currently playing effect immediately.
 */
void haptic_stop(void);

/**
 * @brief Play a custom weak click haptic effect.
 */
void haptic_play_custom_weak_click(void);
