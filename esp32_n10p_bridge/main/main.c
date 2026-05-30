/*
 * ESP32-S3 N10P 雷达数据无线转发固件
 *
 * Phase 2: UART 接收 + WiFi TCP 转发
 *   - Core 1: UART1 460800-8N1 接收 N10P 帧 → FreeRTOS 队列
 *   - Core 0: WiFi Station + TCP Server (port 8888) → 转发队列中的帧
 *
 * TCP 帧格式 (每帧 8+108=116 字节):
 *   [2B sync 0xA55A] [4B seq LE] [2B len LE] [108B N10P raw]
 *
 * 硬件连接:
 *   IO18 (UART1 RX) ← N10P CH9102 TX
 *   IO17 (UART1 TX) → N10P CH9102 RX (控制命令, 可选)
 *   GND ↔ GND
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "n10p_bridge";

/* ============ WiFi 配置 (修改为你自己的路由器) ============ */
#define WIFI_SSID      "YLZ"
#define WIFI_PASS      "yy060315"
#define WIFI_MAX_RETRY 10

/* ============ TCP Server 配置 ============ */
#define TCP_PORT        8888
#define TCP_LISTEN_BACKLOG 1

/* ============ UART 配置 ============ */
#define UART_PORT       UART_NUM_1
#define UART_TX_PIN     17
#define UART_RX_PIN     18
#define UART_BAUD_RATE  460800
#define UART_BUF_SIZE   (108 * 20)

/* ============ N10P 帧参数 ============ */
#define N10P_FRAME_SIZE     108
#define N10P_FRAME_HEADER0  0xA5
#define N10P_FRAME_HEADER1  0x5A

/* TCP 直接发送 N10P 原始帧 (无额外包装), lslidar_driver 自带帧同步 */

/* ============ 队列 ============ */
#define FRAME_QUEUE_LEN 32
static QueueHandle_t g_frame_queue;

/* ============ 统计 ============ */
static uint32_t g_total_frames   = 0;
static uint32_t g_valid_frames   = 0;
static uint32_t g_tcp_sent       = 0;
static int64_t  g_last_report_us = 0;

/* ---- CRC8 ---- */
static uint8_t n10p_crc8(const uint8_t *data, int len) {
    int sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return (uint8_t)(sum & 0xFF);
}

/* ---- UART 初始化 ---- */
static void uart_init(void) {
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_rx_timeout(UART_PORT, 3));
    ESP_LOGI(TAG, "UART1 init OK: %d-8N1, RX=IO%d TX=IO%d",
             UART_BAUD_RATE, UART_RX_PIN, UART_TX_PIN);
}

/* ====================================================================
 * Core 1: UART 帧接收任务 (同 Phase 1, 但收帧后推入队列)
 * ==================================================================== */
typedef enum { ST_WAIT_H0, ST_WAIT_H1, ST_COLLECT } fsm_state_t;

static void uart_rx_task(void *arg) {
    uint8_t buf[256];
    uint8_t frame[N10P_FRAME_SIZE];
    int     idx = 0;
    fsm_state_t st = ST_WAIT_H0;

    while (1) {
        int len = uart_read_bytes(UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(10));
        if (len <= 0) continue;

        for (int i = 0; i < len; i++) {
            uint8_t b = buf[i];
            switch (st) {
            case ST_WAIT_H0:
                if (b == N10P_FRAME_HEADER0) { frame[0] = b; st = ST_WAIT_H1; }
                break;
            case ST_WAIT_H1:
                if (b == N10P_FRAME_HEADER1) { frame[1] = b; idx = 2; st = ST_COLLECT; }
                else if (b == N10P_FRAME_HEADER0) { frame[0] = b; }
                else { st = ST_WAIT_H0; }
                break;
            case ST_COLLECT:
                frame[idx++] = b;
                if (idx >= N10P_FRAME_SIZE) {
                    g_total_frames++;
                    uint8_t crc = n10p_crc8(frame, N10P_FRAME_SIZE - 1);
                    if (frame[N10P_FRAME_SIZE - 1] == crc) {
                        g_valid_frames++;
                        /* 有效帧推入队列 (非阻塞, 满了就丢) */
                        uint8_t *copy = malloc(N10P_FRAME_SIZE);
                        if (copy) {
                            memcpy(copy, frame, N10P_FRAME_SIZE);
                            if (xQueueSend(g_frame_queue, &copy, 0) != pdTRUE) {
                                free(copy);
                            }
                        }
                    }
                    st = ST_WAIT_H0;
                }
                break;
            }
        }

        /* 每秒统计 */
        int64_t now = esp_timer_get_time();
        if (now - g_last_report_us > 1000000) {
            float secs = (now - g_last_report_us) / 1000000.0f;
            ESP_LOGI(TAG, "UART: total=%lu valid=%lu (%.0ffps) TCP发送=%lu 队列剩余=%d",
                     g_total_frames, g_valid_frames,
                     g_valid_frames / secs, g_tcp_sent,
                     (int)uxQueueMessagesWaiting(g_frame_queue));
            g_total_frames = 0;
            g_valid_frames = 0;
            g_tcp_sent    = 0;
            g_last_report_us = now;
        }
    }
}

/* ====================================================================
 * Core 0: TCP Server 任务
 * ==================================================================== */
static void tcp_server_task(void *arg) {
    int listen_sock, client_sock = -1;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

    while (1) {
        /* 创建监听 socket */
        listen_sock = socket(AF_INET, SOCK_STREAM, 0);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "socket failed: %d", errno);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        int opt = 1;
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        server_addr.sin_port = htons(TCP_PORT);

        if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            ESP_LOGE(TAG, "bind failed: %d", errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        if (listen(listen_sock, TCP_LISTEN_BACKLOG) < 0) {
            ESP_LOGE(TAG, "listen failed: %d", errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        ESP_LOGI(TAG, "TCP Server 监听端口 %d, 等待客户端连接...", TCP_PORT);

        /* 接受客户端连接 */
        client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "accept failed: %d", errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        ESP_LOGI(TAG, "客户端已连接! 开始转发数据...");
        close(listen_sock);  /* 只接受一个连接 */

        /* 转发循环: 队列取帧 → 直接发送 108 字节原始 N10P 帧 */
        uint8_t *frame;
        while (1) {
            if (xQueueReceive(g_frame_queue, &frame, pdMS_TO_TICKS(1000)) != pdTRUE) {
                continue;
            }

            /* 直接发送 N10P 原始帧 (TCP 流式传输, lslidar_driver 做帧同步) */
            int sent = send(client_sock, frame, N10P_FRAME_SIZE, 0);
            free(frame);
            if (sent <= 0) {
                ESP_LOGW(TAG, "客户端断开 (sent=%d errno=%d)", sent, errno);
                free(frame);  /* 这帧丢弃 */
                break;
            }
            g_tcp_sent++;
        }

        /* 客户端断开, 清理 */
        close(client_sock);
        client_sock = -1;
        ESP_LOGI(TAG, "等待新客户端连接...");
    }
}

/* ====================================================================
 * WiFi 事件回调
 * ==================================================================== */
static int g_wifi_retry = 0;
static EventGroupHandle_t g_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (g_wifi_retry < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            g_wifi_retry++;
            ESP_LOGW(TAG, "WiFi 断连, 重试 %d/%d", g_wifi_retry, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(g_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "WiFi 已连接! IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        g_wifi_retry = 0;
        xEventGroupSetBits(g_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void) {
    g_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t inst_any_id, inst_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &inst_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &inst_got_ip));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi Station 启动, 连接 SSID: %s", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(g_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi 连接成功!");
    } else {
        ESP_LOGE(TAG, "WiFi 连接失败!");
    }
}

/* ====================================================================
 * 主入口
 * ==================================================================== */
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_LOGI(TAG, "=== N10P 雷达 WiFi 无线转发固件 v4 ===");
    ESP_LOGI(TAG, "Phase 3: UART → WiFi TCP 原始帧转发 (无包装)");
    ESP_LOGI(TAG, "硬件: ESP32-S3 N16R8, UART1 RX=IO%d, %d-8N1",
             UART_RX_PIN, UART_BAUD_RATE);

    /* 初始化队列 */
    g_frame_queue = xQueueCreate(FRAME_QUEUE_LEN, sizeof(uint8_t *));
    assert(g_frame_queue != NULL);

    /* 初始化 UART */
    uart_init();

    /* 连接 WiFi (阻塞, 连接成功后才继续) */
    wifi_init_sta();

    /* 启动 TCP Server (Core 0, lwIP 默认在此核) */
    xTaskCreate(tcp_server_task, "tcp_srv", 4096, NULL, 5, NULL);

    /* 启动 UART 接收任务 (Core 1, WiFi 协议栈不占用) */
    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", 4096, NULL, 10, NULL, 1);

    ESP_LOGI(TAG, "=== 系统就绪 === 等待雷达数据和客户端连接...");
}
