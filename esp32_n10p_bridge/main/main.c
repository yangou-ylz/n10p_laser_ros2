/*
 * ESP32-S3 N10P 雷达数据无线转发固件
 *
 * Phase 1: UART 接收验证 (诊断版)
 *   - UART1 以 460800-8N1 接收 N10P 原始帧
 *   - 从连续字节流中同步帧头 0xA5 0x5A
 *   - 收集完整 108 字节帧，验证 CRC8
 *   - 串口监视器输出统计数据（帧率、有效帧率）
 *   - 诊断: 无论是否收到数据，5 秒报一次心跳
 *
 * 硬件连接 (ESP32-S3 开发板 ↔ N10P TTL 串口):
 *   IO18 (UART1 RX) ← N10P TX
 *   IO17 (UART1 TX) → N10P RX (控制命令, 可选)
 *   GND ↔ GND
 *   5V  ← N10P 5V 供电 (或 USB 供电)
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

static const char *TAG = "n10p_uart";

/* ---- UART 配置 ---- */
#define UART_PORT           UART_NUM_1
#define UART_TX_PIN         17    /* 连 N10P RX (控制命令, 可选) */
#define UART_RX_PIN         18    /* 连 N10P TX */
#define UART_BAUD_RATE      460800
#define UART_BUF_SIZE       (108 * 20)  /* 软件环形缓冲区: 20 帧 */

/* ---- N10P 帧参数 ---- */
#define N10P_FRAME_SIZE     108
#define N10P_FRAME_HEADER0  0xA5
#define N10P_FRAME_HEADER1  0x5A

/* ---- 全局统计 ---- */
static uint32_t g_total_frames    = 0;  /* 总帧数 */
static uint32_t g_valid_frames    = 0;  /* 通过 CRC 的帧数 */
static uint32_t g_sync_lost       = 0;  /* 帧同步丢失次数 */
static uint32_t g_total_bytes     = 0;  /* 总接收字节数 */
static int64_t  g_last_report_us  = 0;  /* 上次报告时间 */
static int64_t  g_start_us        = 0;  /* 任务启动时间 */

/*
 * CRC8: 累加和取低 8 位 (与 lslidar_driver N10_CalCRC8 一致)
 */
static uint8_t n10p_crc8(const uint8_t *data, int len) {
    int sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

/*
 * UART 初始化
 */
static void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate           = UART_BAUD_RATE,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk          = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    /* 设置 RX FIFO 超时: 108 字节以内不超时 (460800bps 下 108B ≈ 2.3ms) */
    ESP_ERROR_CHECK(uart_set_rx_timeout(UART_PORT, 3));

    ESP_LOGI(TAG, "UART1 init OK: baud=%d, rx_pin=%d, tx_pin=%d",
             UART_BAUD_RATE, UART_RX_PIN, UART_TX_PIN);
}

/*
 * 帧接收任务: 从 UART 字节流中提取完整的 N10P 帧
 */
typedef enum {
    STATE_WAIT_HEADER0,
    STATE_WAIT_HEADER1,
    STATE_COLLECT,
} frame_state_t;

static void uart_rx_task(void *arg) {
    uint8_t buf[256];
    uint8_t frame[N10P_FRAME_SIZE];
    int     frame_idx = 0;
    frame_state_t state = STATE_WAIT_HEADER0;
    uint8_t first_byte = 0;
    bool    has_data   = false;

    g_start_us = esp_timer_get_time();
    g_last_report_us = g_start_us;

    while (1) {
        int len = uart_read_bytes(UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (len > 0) {
            g_total_bytes += len;
            if (!has_data) {
                has_data = true;
                first_byte = buf[0];
                ESP_LOGI(TAG, "!!! 首次收到数据: len=%d 首字节=0x%02X '%c'", len, buf[0],
                         (buf[0] >= 32 && buf[0] < 127) ? buf[0] : '?');
                ESP_LOGI(TAG, "  前8字节: %02X %02X %02X %02X %02X %02X %02X %02X",
                         buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
            }

            for (int i = 0; i < len; i++) {
                uint8_t byte = buf[i];

                switch (state) {
                case STATE_WAIT_HEADER0:
                    if (byte == N10P_FRAME_HEADER0) {
                        frame[0] = byte;
                        state = STATE_WAIT_HEADER1;
                    }
                    break;

                case STATE_WAIT_HEADER1:
                    if (byte == N10P_FRAME_HEADER1) {
                        frame[1] = byte;
                        frame_idx = 2;
                        state = STATE_COLLECT;
                    } else {
                        g_sync_lost++;
                        if (byte == N10P_FRAME_HEADER0) {
                            frame[0] = byte;
                        } else {
                            state = STATE_WAIT_HEADER0;
                        }
                    }
                    break;

                case STATE_COLLECT:
                    frame[frame_idx++] = byte;
                    if (frame_idx >= N10P_FRAME_SIZE) {
                        g_total_frames++;
                        uint8_t expected_crc = n10p_crc8(frame, N10P_FRAME_SIZE - 1);
                        if (frame[N10P_FRAME_SIZE - 1] == expected_crc) {
                            g_valid_frames++;
                        }
                        state = STATE_WAIT_HEADER0;
                    }
                    break;
                }
            }
        }

        /* 定时输出统计 */
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - g_last_report_us;

        /* 每 5 秒至少报一次 (心跳) */
        if (elapsed > 5000000) {
            float secs = elapsed / 1000000.0f;
            float byte_rate = g_total_bytes / secs;

            if (g_total_bytes > 0) {
                float fps = g_total_frames / secs;
                ESP_LOGI(TAG, "统计(%ds): 字节=%lu (%.0fB/s) 帧: total=%lu valid=%lu lost=%lu fps=%.1f",
                         (int)secs, g_total_bytes, byte_rate,
                         g_total_frames, g_valid_frames, g_sync_lost, fps);
                if (g_valid_frames > 0) {
                    ESP_LOGI(TAG, "  最新帧: %02X %02X %02X %02X %02X %02X %02X %02X ...",
                             frame[0], frame[1], frame[2], frame[3],
                             frame[4], frame[5], frame[6], frame[7]);
                }
            } else {
                int total_secs = (int)((now - g_start_us) / 1000000);
                ESP_LOGW(TAG, "!!! 心跳: 已等待 %ds, UART 收到 0 字节 !!!", total_secs);
                ESP_LOGW(TAG, "  检查: N10P是否上电/电机是否转/TX→IO18/GND连通");
            }

            g_total_bytes    = 0;
            g_total_frames   = 0;
            g_valid_frames   = 0;
            g_sync_lost      = 0;
            g_last_report_us = now;
        }
    }
}

void app_main(void) {
    /* 初始化 NVS (WiFi 后需用) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_LOGI(TAG, "=== ESP32-S3 N10P 雷达无线转发固件 v2 (诊断版) ===");
    ESP_LOGI(TAG, "Phase 1: UART 接收验证");
    ESP_LOGI(TAG, "硬件: ESP32-S3 N16R8, UART1 RX=IO%d TX=IO%d, %d-8N1",
             UART_RX_PIN, UART_TX_PIN, UART_BAUD_RATE);

    uart_init();

    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", 4096, NULL, 10, NULL, 1);

    ESP_LOGI(TAG, "等待 N10P 雷达数据... (每5秒报心跳)");
}
