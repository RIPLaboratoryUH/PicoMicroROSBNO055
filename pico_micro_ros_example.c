#include <stdio.h>
#include <string.h>
#include <inttypes.h>  // for PRIu64

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "picow_udp_transports.h" 
#include "pico/cyw43_arch.h"

// --- BNO055 definitions ---
#define BNO055_ADDR        0x28
#define BNO055_OPR_MODE    0x3D
#define BNO055_EULER_H_LSB 0x1A

// --- Global micro-ROS objects ---
rcl_publisher_t publisher;
std_msgs__msg__String pub_msg;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

// Buffer for building the message
char pub_msg_buffer[128];

// I2C instance
i2c_inst_t *i2c = i2c0;

// Wifi info
char ssid[] = "riplab";
char pass[] = "Aut0m@tion";

// --- BNO055 functions ---
void i2c_init_bno055() {
    i2c_init(i2c, 400000);
    gpio_set_function(4, GPIO_FUNC_I2C); // SDA
    gpio_set_function(5, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(4);
    gpio_pull_up(5);

    sleep_ms(650);
    uint8_t config[2] = { BNO055_OPR_MODE, 0x00 }; // CONFIG mode
    i2c_write_blocking(i2c, BNO055_ADDR, config, 2, false);
    sleep_ms(50);
    uint8_t ndof[2] = { BNO055_OPR_MODE, 0x0C }; // NDOF mode
    i2c_write_blocking(i2c, BNO055_ADDR, ndof, 2, false);
    sleep_ms(50);
}

int bno055_read_euler(float *heading, float *roll, float *pitch) {
    uint8_t reg = BNO055_EULER_H_LSB;
    uint8_t data[6];
    if (i2c_write_blocking(i2c, BNO055_ADDR, &reg, 1, true) != 1) return -1;
    if (i2c_read_blocking(i2c, BNO055_ADDR, data, 6, false) != 6) return -1;

    int16_t h = (int16_t)(data[0] | (data[1] << 8));
    int16_t r = (int16_t)(data[2] | (data[3] << 8));
    int16_t p = (int16_t)(data[4] | (data[5] << 8));

    *heading = (float)h / 16.0f;
    *roll    = (float)r / 16.0f;
    *pitch   = (float)p / 16.0f;

    return 0;
}

// --- micro-ROS setup ---
void setup_transport() {
    rmw_uros_set_custom_transport(
        false,
        &picow_params,
        picow_udp_transport_open,
        picow_udp_transport_close,
        picow_udp_transport_write,
        picow_udp_transport_read
    );
}

void setup_ros() {
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "imu_reading"
    );

    rclc_executor_init(&executor, &support.context, 0, &allocator); // no subscriptions
}

// --- Main ---
int main() {
    stdio_init_all();
    cyw43_arch_init_with_country(CYW43_COUNTRY_USA);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    cyw43_arch_enable_sta_mode();
    cyw43_arch_wifi_connect_timeout_ms(ssid, pass, CYW43_AUTH_WPA2_AES_PSK, 10000);

    i2c_init_bno055();
    setup_transport();

    // Wait for agent successful ping
    const int timeout_ms = 1000;
    const uint8_t attempts = 60;
    rcl_ret_t ret = 0;
    int loop = 0;
    for (; loop < attempts; loop++) {
        ret = rmw_uros_ping_agent(timeout_ms, 1);
        if (ret == RCL_RET_OK) break;
    }
    if (loop == attempts) return ret;

    setup_ros();

    absolute_time_t last_pub_time = get_absolute_time();

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        cyw43_arch_poll();

        // Publish once per second
        if (absolute_time_diff_us(last_pub_time, get_absolute_time()) >= 1000000) {
            last_pub_time = get_absolute_time();

            float h, r, p;
            int status = bno055_read_euler(&h, &r, &p);
            int len;
            if (status == 0) {
                len = snprintf(pub_msg_buffer, sizeof(pub_msg_buffer),
                    "H: %.2f, R: %.2f, P: %.2f", h, r, p);
            } else {
                len = snprintf(pub_msg_buffer, sizeof(pub_msg_buffer),
                    "read error");
            }

            pub_msg.data.data = pub_msg_buffer;
            pub_msg.data.size = len;
            pub_msg.data.capacity = sizeof(pub_msg_buffer);

            rcl_publish(&publisher, &pub_msg, NULL);
        }
    }

    cyw43_arch_deinit();
    return 0;
}

