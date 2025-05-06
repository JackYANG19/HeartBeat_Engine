#include "haptic.h"
#include "i2c_utils.h"
#include "esp_log.h"

#define TAG "HAPTIC"

// Change this based on motor type
#define IS_LRA true  // false for ERM

#define HAPTIC_I2C_ADDR 0x5A

#define REG_STATUS       0x00
#define REG_MODE         0x01
#define REG_LIBRARY_SEL  0x03
#define REG_WAVESEQ1     0x04
#define REG_WAVESEQ2     0x05
#define REG_GO           0x0C
#define REG_FEEDBACK     0x1A
#define REG_CONTROL1     0x1B

static esp_err_t write8(uint8_t reg, uint8_t val) {
    return i2c_write_reg(I2C_NUM_0, HAPTIC_I2C_ADDR, reg, &val, 1);
}

static esp_err_t read8(uint8_t reg, uint8_t *val) {
    return i2c_read_reg(I2C_NUM_0, HAPTIC_I2C_ADDR, reg, val, 1);
}

void haptic_init(void)
{
    ESP_LOGI(TAG, "Probing DRV2605L...");

    uint8_t who = 0;
    if (read8(REG_STATUS, &who) != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed");
        return;
    }

    ESP_LOGI(TAG, "STATUS register = 0x%02X", who);

    // Put into standby before config
    write8(REG_MODE, 0x40);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Select library 1 (LRA)
    write8(REG_LIBRARY_SEL, 0x01);

    // Set motor type
    if (IS_LRA) {
        write8(REG_FEEDBACK, 0xB6);  // LRA mode (bit 7 = 1)
        ESP_LOGI(TAG, "Configured for LRA mode");
    } else {
        write8(REG_FEEDBACK, 0x36);  // ERM mode (bit 7 = 0)
        ESP_LOGI(TAG, "Configured for ERM mode");
    }

    // Clear sequence
    write8(REG_WAVESEQ1, 0x00);
    write8(REG_WAVESEQ2, 0x00);

    // Set to internal trigger (active mode)
    write8(REG_MODE, 0x00);

    ESP_LOGI(TAG, "Haptic driver initialized");
}

void haptic_play_effect(uint8_t effect_id)
{
    // Load effect into slot 1
    write8(REG_WAVESEQ1, effect_id);

    // Verify it
    uint8_t readback = 0;
    read8(REG_WAVESEQ1, &readback);
    ESP_LOGI(TAG, "WAVESEQ1 readback = 0x%02X", readback);

    // GO
    write8(REG_GO, 0x01);
    ESP_LOGI(TAG, "Triggered effect %d", effect_id);

    // Give it time to complete
    vTaskDelay(pdMS_TO_TICKS(20));
}

void haptic_play_custom_weak_click(void)
{
    ESP_LOGI(TAG, "Playing custom weak click via RAM...");

    // 1. Set mode to RAM playback
    write8(REG_MODE, 0x03);  // RAM mode
    vTaskDelay(pdMS_TO_TICKS(2));

    // 2. (Optional) Set library
    write8(REG_LIBRARY_SEL, 0x01);  // still recommended

    // 3. Write waveform to RAM
    write8(0xFD, 0x01);       // Start RAM write
    write8(0xFD, 0x1F);       // Amplitude 5% (0x0F = ~5%)
    write8(0xFD, 0x00);       // End of waveform
    ESP_LOGI(TAG, "Custom waveform loaded to RAM");

    // 4. Configure waveform sequence to use RAM[0]
    write8(REG_WAVESEQ1, 0x00);  // RAM address 0
    write8(REG_WAVESEQ2, 0x00);  // End marker

    // 5. GO!
    write8(REG_GO, 0x01);
    ESP_LOGI(TAG, "GO triggered");
}


void haptic_stop(void)
{
    // Write 0 to GO bit
    uint8_t mode = 0;
    if (read8(REG_MODE, &mode) == ESP_OK) {
        mode &= ~0x01;
        write8(REG_MODE, mode);
    }
}
