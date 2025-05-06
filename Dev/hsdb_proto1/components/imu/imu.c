#include "imu.h"
#include "lsm6dso_reg.h"
#include "i2c_utils.h"
#include "esp_log.h"


static const char *TAG = "IMU";

static stmdev_ctx_t dev_ctx = {0};

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

void imu_log_reading(void)
{
    lsm6dso_status_reg_t status;
    lsm6dso_status_reg_get(&dev_ctx, &status);

    if (status.xlda && status.gda) {
        int16_t acc_raw[3], gyro_raw[3];
        float acc_mg[3], gyro_mdps[3];
        float acc_g[3], gyro_dps[3];

        // Read raw data
        lsm6dso_acceleration_raw_get(&dev_ctx, acc_raw);
        lsm6dso_angular_rate_raw_get(&dev_ctx, gyro_raw);

        // Convert to physical units
        acc_mg[0] = lsm6dso_from_fs2_to_mg(acc_raw[0]);
        acc_mg[1] = lsm6dso_from_fs2_to_mg(acc_raw[1]);
        acc_mg[2] = lsm6dso_from_fs2_to_mg(acc_raw[2]);

        gyro_mdps[0] = lsm6dso_from_fs2000_to_mdps(gyro_raw[0]);
        gyro_mdps[1] = lsm6dso_from_fs2000_to_mdps(gyro_raw[1]);
        gyro_mdps[2] = lsm6dso_from_fs2000_to_mdps(gyro_raw[2]);

        acc_g[0] = acc_mg[0] / 1000;
        acc_g[1] = acc_mg[1] / 1000;
        acc_g[2] = acc_mg[2] / 1000;
        gyro_dps[0] = gyro_mdps[0] / 1000;
        gyro_dps[1] = gyro_mdps[1] / 1000;
        gyro_dps[2] = gyro_mdps[2] / 1000;

        // Log both accelerometer and gyro
        ESP_LOGI("IMU", "Accel [mg]:  X=%10.3f Y=%10.3f Z=%10.3f | Gyro [mdps]: X=%10.3f Y=%10.3f Z=%10.3f",
                 acc_mg[0], acc_mg[1], acc_mg[2],
                 gyro_mdps[0], gyro_mdps[1], gyro_mdps[2]);
        // ESP_LOGI("IMU", "Accel [g]:  X=%8.3f Y=%8.3f Z=%8.3f | Gyro [dps]: X=%10.3f Y=%10.3f Z=%10.3f",
        //     acc_g[0], acc_g[1], acc_g[2],
        //     gyro_dps[0], gyro_dps[1], gyro_dps[2]);
    }
}

void imu_task(void *arg)
{
    ESP_LOGI("IMU", "IMU task started");
    while (1) {
        imu_log_reading();
        vTaskDelay(pdMS_TO_TICKS(100));  // log every 100 ms
    }
}


void imu_init(void)
{
    // Driver context
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = (void *)0x6A;

    uint8_t whoami = 0;
    lsm6dso_device_id_get(&dev_ctx, &whoami);
    if (whoami != LSM6DSO_ID) {
        ESP_LOGE(TAG, "IMU not found! Got: 0x%02X", whoami);
        return;
    }

    lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);
    uint8_t rst;
    do { lsm6dso_reset_get(&dev_ctx, &rst); } while (rst);

    lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_52Hz);
    lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_52Hz);
    lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);
    lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_2000dps);
    
    ESP_LOGI("IMU", "imu_init done");

    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    uint8_t dev_addr = (uint32_t)(uintptr_t)handle;
    // call shared helper
    return i2c_write_reg(I2C_NUM_0, dev_addr, reg, bufp, len) == ESP_OK
           ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    uint8_t dev_addr = (uint32_t)(uintptr_t)handle;
    return i2c_read_reg(I2C_NUM_0, dev_addr, reg, bufp, len) == ESP_OK
           ? 0 : -1;
}




