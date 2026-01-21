#include <string.h>

#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "queuemonitor.h"
#include "wifi_esp32.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "WIFI_UDP"
#include "debug_cf.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
#include "esp_mac.h"
#endif

#ifdef CONFIG_REMOTE_SERVER_ENABLE
#include "remote_server.h"
#endif

#define UDP_SERVER_PORT 2390
#define UDP_SERVER_BUFSIZE 64

static struct sockaddr_storage source_addr;

static char WIFI_SSID[32] = "";
static char WIFI_PWD[64] = CONFIG_WIFI_PASSWORD;
static uint8_t WIFI_CH = CONFIG_WIFI_CHANNEL;
#define WIFI_MAX_STA_CONN CONFIG_WIFI_MAX_STA_CONN

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

static int sock;
static xQueueHandle udpDataRx;
static xQueueHandle udpDataTx;

static bool isInit = false;
static bool isUDPInit = false;
static bool isUDPConnected = false;

#ifdef CONFIG_WIFI_ENABLE_STA_MODE
// STA 模式相关变量
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static bool isSTAConnected = false;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#endif

static esp_err_t udp_server_create(void *arg);

static uint8_t calculate_cksum(void *data, size_t len)
{
    unsigned char *c = data;
    int i;
    unsigned char cksum = 0;

    for (i = 0; i < len; i++)
    {
        cksum += *(c++);
    }

    return cksum;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    // AP 事件处理
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        DEBUG_PRINT_LOCAL("station" MACSTR "join, AID=%d", MAC2STR(event->mac), event->aid);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        DEBUG_PRINT_LOCAL("station" MACSTR "leave, AID=%d", MAC2STR(event->mac), event->aid);
    }

#ifdef CONFIG_WIFI_ENABLE_STA_MODE
    // STA 事件处理
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
        DEBUG_PRINT_LOCAL("STA started, connecting to router...");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        isSTAConnected = false;
        if (s_retry_num < CONFIG_WIFI_STA_RETRY_MAX)
        {
            esp_wifi_connect();
            s_retry_num++;
            DEBUG_PRINT_LOCAL("Retry to connect to the AP (%d/%d)", s_retry_num, CONFIG_WIFI_STA_RETRY_MAX);
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            DEBUG_PRINT_LOCAL("Failed to connect to AP after %d retries", CONFIG_WIFI_STA_RETRY_MAX);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        DEBUG_PRINT_LOCAL("Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        isSTAConnected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
#endif
}

bool wifiTest(void)
{
    return isInit;
};

bool wifiGetDataBlocking(UDPPacket *in)
{
    /* command step - receive  02  from udp rx queue */
    while (xQueueReceive(udpDataRx, in, portMAX_DELAY) != pdTRUE)
    {
        vTaskDelay(M2T(10));
    }; // Don't return until we get some data on the UDP

    return true;
};

bool wifiSendData(uint32_t size, uint8_t *data)
{
    UDPPacket outStage = {0};
    outStage.size = size;
    memcpy(outStage.data, data, size);
    // Dont' block when sending
    return (xQueueSend(udpDataTx, &outStage, M2T(100)) == pdTRUE);
};

static esp_err_t udp_server_create(void *arg)
{
    if (isUDPInit)
    {
        return ESP_OK;
    }

    static struct sockaddr_in dest_addr = {0};
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_SERVER_PORT);

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        DEBUG_PRINT_LOCAL("Unable to create socket: errno %d", errno);
        return ESP_FAIL;
    }
    DEBUG_PRINT_LOCAL("Socket created");

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0)
    {
        DEBUG_PRINT_LOCAL("Socket unable to bind: errno %d", errno);
    }
    DEBUG_PRINT_LOCAL("Socket bound, port %d", UDP_SERVER_PORT);

    isUDPInit = true;
    return ESP_OK;
}

static void udp_server_rx_task(void *pvParameters)
{
    socklen_t socklen = sizeof(source_addr);
    char rx_buffer[UDP_SERVER_BUFSIZE];
    UDPPacket inPacket = {0};

    while (true)
    {
        if (isUDPInit == false)
        {
            vTaskDelay(20);
            continue;
        }
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);
        /* command step - receive  01 from Wi-Fi UDP */
        if (len < 0)
        {
            DEBUG_PRINT_LOCAL("recvfrom failed: errno %d", errno);
            continue;
        }
        else if (len > WIFI_RX_TX_PACKET_SIZE)
        {
            DEBUG_PRINT_LOCAL("Received data length = %d > %d", len, WIFI_RX_TX_PACKET_SIZE);
            continue;
        }
        else
        {
            uint8_t cksum = rx_buffer[len - 1];
            // remove cksum, do not belong to CRTP
            // check packet
            if (cksum == calculate_cksum(rx_buffer, len - 1))
            {
                // copy part of the UDP packet, the size not include cksum
                inPacket.size = len - 1;
                memcpy(inPacket.data, rx_buffer, inPacket.size);
                xQueueSend(udpDataRx, &inPacket, M2T(10));
                if (!isUDPConnected)
                    isUDPConnected = true;
            }
            else
            {
                DEBUG_PRINT_LOCAL("udp packet cksum unmatched");
            }

#ifdef DEBUG_UDP
            printf("\nReceived size = %d cksum = %02X\n", inPacket.size, cksum);
            for (size_t i = 0; i < inPacket.size; i++)
            {
                printf("%02X ", inPacket.data[i]);
            }
            printf("\n");
#endif
        }
    }
}

static void udp_server_tx_task(void *pvParameters)
{
    UDPPacket outPacket = {0};
    while (TRUE)
    {
        if (isUDPInit == false)
        {
            vTaskDelay(20);
            continue;
        }
        if ((xQueueReceive(udpDataTx, &outPacket, portMAX_DELAY) == pdTRUE) && isUDPConnected)
        {
            // append cksum to the packet
            outPacket.data[outPacket.size] = calculate_cksum(outPacket.data, outPacket.size);
            outPacket.size += 1;

            int err = sendto(sock, outPacket.data, outPacket.size, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
            if (err < 0)
            {
                DEBUG_PRINT_LOCAL("Error occurred during sending: errno %d", errno);
                continue;
            }
#ifdef DEBUG_UDP
            printf("\nSend size = %d checksum = %02X\n", outPacket.size, outPacket.data[outPacket.size - 1]);
            for (size_t i = 0; i < outPacket.size; i++)
            {
                printf("%02X ", outPacket.data[i]);
            }
            printf("\n");
#endif
        }
    }
}

void wifiInit(void)
{
    if (isInit)
    {
        return;
    }
    // This should probably be reduced to a CRTP packet size
    udpDataRx = xQueueCreate(16, sizeof(UDPPacket));
    DEBUG_QUEUE_MONITOR_REGISTER(udpDataRx);
    udpDataTx = xQueueCreate(16, sizeof(UDPPacket));
    DEBUG_QUEUE_MONITOR_REGISTER(udpDataTx);

#ifdef CONFIG_WIFI_ENABLE_STA_MODE
    // ============ STA+AP 模式 ============
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_t *ap_netif = NULL;
    esp_netif_t *sta_netif = NULL;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 创建 STA 和 AP 网络接口
    sta_netif = esp_netif_create_default_wifi_sta();
    ap_netif = esp_netif_create_default_wifi_ap();

    uint8_t mac[6];
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 注册事件处理器
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    // 设置 STA+AP 模式
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    // 配置 AP
    ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));
    sprintf(WIFI_SSID, "%s_%02X%02X%02X%02X%02X%02X", CONFIG_WIFI_BASE_SSID, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    wifi_config_t ap_config = {
        .ap = {
            .channel = WIFI_CH,
            .max_connection = WIFI_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };
    memcpy(ap_config.ap.ssid, WIFI_SSID, strlen(WIFI_SSID) + 1);
    ap_config.ap.ssid_len = strlen(WIFI_SSID);
    memcpy(ap_config.ap.password, WIFI_PWD, strlen(WIFI_PWD) + 1);
    if (strlen(WIFI_PWD) == 0)
    {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    // 配置 STA
    wifi_config_t sta_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };
    memcpy(sta_config.sta.ssid, CONFIG_WIFI_STA_SSID, strlen(CONFIG_WIFI_STA_SSID) + 1);
    memcpy(sta_config.sta.password, CONFIG_WIFI_STA_PASSWORD, strlen(CONFIG_WIFI_STA_PASSWORD) + 1);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));

    // 启动 Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());

    // 配置 AP 的静态 IP
    esp_netif_ip_info_t ip_info = {
        .ip.addr = ipaddr_addr("192.168.43.42"),
        .netmask.addr = ipaddr_addr("255.255.255.0"),
        .gw.addr = ipaddr_addr("192.168.43.42"),
    };
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    DEBUG_PRINT_LOCAL("Wi-Fi STA+AP mode initialized");
    DEBUG_PRINT_LOCAL("AP SSID: %s, password: %s", WIFI_SSID, WIFI_PWD);
    DEBUG_PRINT_LOCAL("STA connecting to: %s", CONFIG_WIFI_STA_SSID);

    // 等待 STA 连接结果
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           M2T(10000)); // 10秒超时

    if (bits & WIFI_CONNECTED_BIT)
    {
        DEBUG_PRINT_LOCAL("Connected to router: %s", CONFIG_WIFI_STA_SSID);
        // 远程服务器任务将在 system.c 中启动
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        DEBUG_PRINT_LOCAL("Failed to connect to router, AP mode still available");
    }
    else
    {
        DEBUG_PRINT_LOCAL("STA connection timeout, AP mode still available");
    }

#else
    // ============ 仅 AP 模式（原有逻辑）============
    esp_netif_t *ap_netif = NULL;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ap_netif = esp_netif_create_default_wifi_ap();
    uint8_t mac[6];

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));
    sprintf(WIFI_SSID, "%s_%02X%02X%02X%02X%02X%02X", CONFIG_WIFI_BASE_SSID, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    wifi_config_t wifi_config = {
        .ap = {
            .channel = WIFI_CH,
            .max_connection = WIFI_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    memcpy(wifi_config.ap.ssid, WIFI_SSID, strlen(WIFI_SSID) + 1);
    wifi_config.ap.ssid_len = strlen(WIFI_SSID);
    memcpy(wifi_config.ap.password, WIFI_PWD, strlen(WIFI_PWD) + 1);

    if (strlen(WIFI_PWD) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_netif_ip_info_t ip_info = {
        .ip.addr = ipaddr_addr("192.168.43.42"),
        .netmask.addr = ipaddr_addr("255.255.255.0"),
        .gw.addr = ipaddr_addr("192.168.43.42"),
    };
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));
    DEBUG_PRINT_LOCAL("wifi_init_softap complete.SSID:%s password:%s", WIFI_SSID, WIFI_PWD);
#endif

    // UDP 服务器（本地控制，两种模式都需要）
    if (udp_server_create(NULL) == ESP_FAIL)
    {
        DEBUG_PRINT_LOCAL("UDP server create socket failed");
    }
    else
    {
        DEBUG_PRINT_LOCAL("UDP server create socket succeed");
    }
    xTaskCreate(udp_server_tx_task, UDP_TX_TASK_NAME, UDP_TX_TASK_STACKSIZE, NULL, UDP_TX_TASK_PRI, NULL);
    xTaskCreate(udp_server_rx_task, UDP_RX_TASK_NAME, UDP_RX_TASK_STACKSIZE, NULL, UDP_RX_TASK_PRI, NULL);
    isInit = true;
}

#ifdef CONFIG_WIFI_ENABLE_STA_MODE
/**
 * @brief 检查 STA 是否已连接到路由器
 */
bool wifiIsSTAConnected(void)
{
    return isSTAConnected;
}
#endif