/*
 *
 * ACL_Client_ESP32Rev1-IDF
 *
 * This is a stand-alone program for an ESP32 based board (such as an
 * Adafruit Feather). This program should work on other ESP32 boards. It
 * uses the second UART for communication with a ID Innovations ID-12LA
 * or ID-20LA RFID reader. This must be built under the ESP-IDF
 * toolchain.
 *
 * This sketch will wait for an RFID to be sent by the reader and then
 * validate it with the Creator Hub's ACL server API via WiFi
 *
 * Depending on whether the MOMENTARY flag is set or not, it will
 * wait for an RFID to be no longer in range of the RFID reader before
 * deasserting the relay output.
 *
 * This program can cache a copy of the current list of valid RFIDs for
 * this machine/door (configurable).
 *
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "mqtt_client.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "wificonfig.h"

/* This uses configurations that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define WIFI_SSID "mywifissid"
*/

// Constants that aren't configurable (yet)
//
#define RANGE_CHECK_TIME 100

// Board-specific constants
//
#define RFID_TXD  (GPIO_NUM_17)
#define RFID_RXD  (GPIO_NUM_16)
#define RFID_RTS  (UART_PIN_NO_CHANGE)
#define RFID_CTS  (UART_PIN_NO_CHANGE)

#define GPIO_INPUT_GPIO0          0
#define GPIO_INPUT_IN_RANGE       4
#define GPIO_INPUT_PIN_SEL (1ULL<<GPIO_INPUT_GPIO0 | 1ULL<<GPIO_INPUT_IN_RANGE)

#define GPIO_OUTPUT_RELAY_POWER   14
#define GPIO_OUTPUT_CONNECTED_LED 25
#define GPIO_OUTPUT_ACCESS_LED    26
#define GPIO_OUTPUT_FAIL_LED      27
#define GPIO_OUTPUT_PIN_SEL ((1ULL<<GPIO_OUTPUT_RELAY_POWER) | (1ULL<<GPIO_OUTPUT_ACCESS_LED) | (1ULL<<GPIO_OUTPUT_CONNECTED_LED) | (1ULL<<GPIO_OUTPUT_FAIL_LED))
/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;


#define BUF_SIZE (1024)

uint8_t *read_data; // storage for UART read


// list of access points to try
//

struct ap_entry {
    char *ssid;
    char *password;
};
struct ap_entry *ap_list = NULL;
int ap_count = 0;
int ap_idx = 0;

// load AP credentials into list from wifi configuration structure
static void load_aps (void) {
    // first, find number of APs
    if (strlen(wificonfig_vals_wifi.ap1_ssid) > 0) ap_count++;
    if (strlen(wificonfig_vals_wifi.ap2_ssid) > 0) ap_count++;
    if (strlen(wificonfig_vals_wifi.ap3_ssid) > 0) ap_count++;
    if (strlen(wificonfig_vals_wifi.ap4_ssid) > 0) ap_count++;

    if (ap_count > 0)
        ap_list = malloc (sizeof(struct ap_entry) * ap_count);

    ap_idx = 0;
    if (strlen(wificonfig_vals_wifi.ap1_ssid) > 0) {
       ap_list[ap_idx].ssid = wificonfig_vals_wifi.ap1_ssid;
       ap_list[ap_idx++].password = wificonfig_vals_wifi.ap1_pswd;
    }
    if (strlen(wificonfig_vals_wifi.ap2_ssid) > 0) {
       ap_list[ap_idx].ssid = wificonfig_vals_wifi.ap2_ssid;
       ap_list[ap_idx++].password = wificonfig_vals_wifi.ap2_pswd;
    }
    if (strlen(wificonfig_vals_wifi.ap3_ssid) > 0) {
       ap_list[ap_idx].ssid = wificonfig_vals_wifi.ap3_ssid;
       ap_list[ap_idx++].password = wificonfig_vals_wifi.ap3_pswd;
    }
    if (strlen(wificonfig_vals_wifi.ap4_ssid) > 0) {
       ap_list[ap_idx].ssid = wificonfig_vals_wifi.ap4_ssid;
       ap_list[ap_idx++].password = wificonfig_vals_wifi.ap4_pswd;
    }
}

const char *TAG = "ACL_Client";

static int access_state = false;

void blinkit_high(int pin, int count, int wait){
    for (int i=count; i>0; i--) {
        gpio_set_level (pin, 1);
        vTaskDelay(wait / portTICK_RATE_MS);
        gpio_set_level (pin, 0);
        if (i > 1)
            vTaskDelay(wait / portTICK_RATE_MS);
    }
}

void blinkit_low(int pin, int count, int wait){
    for (int i=count; i>0; i--) {
        gpio_set_level (pin, 0);
        vTaskDelay(wait / portTICK_RATE_MS);
        gpio_set_level (pin, 1);
        if (i > 1)
            vTaskDelay(wait / portTICK_RATE_MS);
  }
}

void good_rfid_sequence() {
  // turn relay on
  gpio_set_level (GPIO_OUTPUT_RELAY_POWER, 1);
  gpio_set_level (GPIO_OUTPUT_ACCESS_LED, 1);
  gpio_set_level (GPIO_OUTPUT_FAIL_LED, 0);
  ESP_LOGI (TAG, "Turning on relay power");
}

void bad_rfid_sequence () {
   // turn relay off and flash LED
  gpio_set_level (GPIO_OUTPUT_RELAY_POWER, 0);
  gpio_set_level (GPIO_OUTPUT_ACCESS_LED, 0);
  ESP_LOGI (TAG, "Signaling bad RFID");
  blinkit_high (GPIO_OUTPUT_FAIL_LED, 5, 250);
}

void no_rfid_sequence () {
  // turn relay off
  gpio_set_level (GPIO_OUTPUT_RELAY_POWER, 0);
  gpio_set_level (GPIO_OUTPUT_ACCESS_LED, 0);
  gpio_set_level (GPIO_OUTPUT_FAIL_LED, 0);
}


// Validate data from RFID reader against CRC data
//
int check_crc (unsigned long data, unsigned long crc) {
  unsigned long res = 0;
  ESP_LOGV (TAG, "CRC check: data=%lx, crc=%lx", data, crc);
  for (int i = 0; i < 2; i++) {
    res ^= (crc & 0xff);
    crc = crc >> 8;
  }
  for (int i = 0; i < 4; i++) { 
    res ^= (data & 0xff);
    data = data >> 8;
  }

  ESP_LOGV (TAG, "CRC result=%lx", res);
  return (res == 0);
}

// get RFID from reader (via hw serial port)
// Return value: True  - valid RFID read
//               False - no read data (no RFID) or invalid (bad CRC)  RFID read
//
int get_RFID (unsigned long *rfid) {
    *rfid = 0;
    unsigned long crc = 0;
    unsigned char in_byte = 0;
    int count = 0;
    int reading = false;
    int found = false;

    int len = uart_read_bytes(UART_NUM_1, read_data, BUF_SIZE, 20 / portTICK_RATE_MS);
    if (len > 0) {
        ESP_LOGV (TAG, "RFID read length=%d", len);
        //printf ("Raw data: '");
        //for (int i = 0; i < len; i++) {
        //    printf ("%c", read_data[i]);
        //}
        //printf ("'\n");

        for (int i = 0; i < len; i++) {
            //printf ("partial crc, rfid = %lu %lu\n", crc, *rfid);
            in_byte = read_data[i];
//printf ("in_byte = %d, 0x%x, '%c'\n", in_byte, in_byte, in_byte);
            if (in_byte == 0x2) {
                // Detected the "start" char
                //
                reading = true;
                count = 0;
                *rfid = 0x0;
                crc = 0x0;
            } else if (reading && (count < 3)) {
                // First two nibbles in hex chars are not part of the card number, but do take part in CRC
                //
                unsigned char hex_digit = (in_byte < 'A') ? (in_byte - '0') : (in_byte - 'A' + 0xa);
                crc = (crc << 4) | hex_digit;

            } else if (reading && (count >= 3) && (count <= 10)) {
                // Next 8 nibbles are the card number in hex chars
                //
                unsigned char hex_digit = (in_byte < 'A') ? (in_byte - '0') : (in_byte - 'A' + 0xa);
                *rfid = (*rfid << 4) | hex_digit;
} else if (reading && (count > 10) && (count < 13)) {
                // Last 2 nibbles are the CRC in hex chars
                //
                unsigned char hex_digit = (in_byte < 'A') ? (in_byte - '0') : (in_byte - 'A' + 0xa);
                crc = (crc << 4) | hex_digit;

            } else if (reading && (count == 13)) {
                // Last char should be the "stop" char, but we don't actually care what it
                // is, it merely has to be the 13th char after the start char
                //
                reading = false;
                found = true;
                break;
            }
            if (reading)
                count++;
        }
    }

    found = found && (len >= 14) && check_crc(*rfid, crc);
    return (found);
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    ESP_LOGI(TAG, "event_handler: Event dispatched from event loop base=%s, event_id=%d", event_base, event_id);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, wificonfig_vals_wifi.hostname);
        ESP_ERROR_CHECK(esp_wifi_connect());
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Got disconnected");
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        gpio_set_level(GPIO_OUTPUT_CONNECTED_LED, 0);
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);

        // connect to next AP on list
        ap_idx++;
        if (ap_idx >= ap_count) {
            ap_idx = 0;
        }
        wifi_config_t wifi_config = {
            .sta = {
                .ssid = "",
                .password = "",
            },
        };
        strncpy((char *)wifi_config.sta.ssid, ap_list[ap_idx].ssid, 32);
        strncpy((char *)wifi_config.sta.password, ap_list[ap_idx].password, 64);

        ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
        ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
        ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
        ESP_ERROR_CHECK( esp_wifi_connect() );
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        gpio_set_level(GPIO_OUTPUT_CONNECTED_LED, 1);
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    }
}


static void initialize_wifi (void) {
    esp_netif_init();
    wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_NONE) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
        },
    };

    ap_idx = 0;
    strncpy((char *)wifi_config.sta.ssid, (char *)ap_list[ap_idx].ssid, 32);
    strncpy((char *)wifi_config.sta.password, (char *)ap_list[ap_idx].password, 64);

    ESP_LOGI(TAG, "Setting WiFi configuration SSID '%s'...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static esp_mqtt_client_handle_t mqtt_client = NULL;
static int mqtt_connected = false;

static void publish_status (char *subtopic, int val) {
    if ((mqtt_client == NULL) || (!mqtt_connected)) {
        return;
    }
    char topic[128];
    sprintf (topic, "stat/%s/%s", wificonfig_vals_mqtt.topic, subtopic);
    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, val ? "ON" : "OFF", 0, 1, 0);
    ESP_LOGI(TAG, "publish successful, msg_id=%d", msg_id);
}

static void mqtt_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ESP_LOGI(TAG, "mqtt_event_handler: Event dispatched from event loop base=%s, event_id=%d", event_base, event_id);

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            mqtt_connected = true;
            // subscriptions would go here, but not needed for RFID reader
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            mqtt_connected = false;
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            ESP_LOGI(TAG, "TOPIC=%.*s", event->topic_len, event->topic);
            ESP_LOGI(TAG, "DATA=%.*s", event->data_len, event->data);
            break;

        default:
            ESP_LOGI(TAG, "Other event id:%d", event_id);
            break;
    }
}


static void initialize_mqtt () {

    char uri[128];

    mqtt_client = NULL;
    mqtt_connected = false;
    if (strcmp (wificonfig_vals_mqtt.host, "") == 0) {
        return;
    }

    sprintf (uri, "mqtt://%s", wificonfig_vals_mqtt.host);

    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = uri,
        .port = wificonfig_vals_mqtt.port,
        .client_id = wificonfig_vals_mqtt.client,
        .username = wificonfig_vals_mqtt.user,
        .password = wificonfig_vals_mqtt.pswd,
    };

    // wait for Wifi connection
    while ((xEventGroupGetBits (wifi_event_group) & CONNECTED_BIT) == 0) {
        vTaskDelay(100 / portTICK_RATE_MS);
    }


    mqtt_client = esp_mqtt_client_init (&mqtt_cfg);
    if (mqtt_client != NULL) {
        ESP_ERROR_CHECK(esp_mqtt_client_register_event (mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client));
        ESP_ERROR_CHECK(esp_mqtt_client_start (mqtt_client));
    } else {
        ESP_LOGE(TAG, "initialize_mqtt: Unable to initialize MQTT client");
    }
}



static void initialize_uart(void) {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, RFID_TXD, RFID_RXD, RFID_RTS, RFID_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    read_data = (uint8_t *) malloc(BUF_SIZE);
}

void initialize_pins (void) {
    gpio_config_t io_conf;

    // Configure Inputs

    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO21/32
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);


    // Configure Outputs

    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO21/32
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(GPIO_OUTPUT_RELAY_POWER, 0);
    gpio_set_level(GPIO_OUTPUT_CONNECTED_LED, 0);
    gpio_set_level(GPIO_OUTPUT_ACCESS_LED, 0);
    gpio_set_level(GPIO_OUTPUT_FAIL_LED, 0);

    // Flash all LEDs on for 500ms
    //
    gpio_set_level(GPIO_OUTPUT_CONNECTED_LED, 1);
    gpio_set_level(GPIO_OUTPUT_ACCESS_LED, 1);
    gpio_set_level(GPIO_OUTPUT_FAIL_LED, 1);
    vTaskDelay(500 / portTICK_RATE_MS);
    gpio_set_level(GPIO_OUTPUT_CONNECTED_LED, 0);
    gpio_set_level(GPIO_OUTPUT_ACCESS_LED, 0);
    gpio_set_level(GPIO_OUTPUT_FAIL_LED, 0);
}

static void strobe_leds (void *pvParameters) {
    while (1) {
        gpio_set_level(GPIO_OUTPUT_CONNECTED_LED, 1);
        vTaskDelay(200 / portTICK_RATE_MS);
        gpio_set_level(GPIO_OUTPUT_CONNECTED_LED, 0);
        gpio_set_level(GPIO_OUTPUT_ACCESS_LED, 1);
        vTaskDelay(200 / portTICK_RATE_MS);
        gpio_set_level(GPIO_OUTPUT_ACCESS_LED, 0);
        gpio_set_level(GPIO_OUTPUT_FAIL_LED, 1);
        vTaskDelay(200 / portTICK_RATE_MS);
        gpio_set_level(GPIO_OUTPUT_FAIL_LED, 0);
        vTaskDelay(200 / portTICK_RATE_MS);
    }
}

static void check_gpio0 (void *pvParameters) {
    int last_level = 1;
    int64_t pressed_time = 0;
    while (1) {
        int new_level = gpio_get_level(GPIO_INPUT_GPIO0);
        if (new_level != last_level) {
            int64_t curr_time = esp_timer_get_time ();
            if (new_level == 1) {
                int64_t curr_time = esp_timer_get_time ();
                if ((curr_time - pressed_time) > 3000000) {
                    trigger_wificonfig();
                }
            } else {
                pressed_time = curr_time;
            }
        }
        last_level = new_level;
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}


// Return a single newline-terminated line from a socket
//
char recv_buf[256];
char *buf_pos;
int recv_len;
int recv_done;
void read_line_socket_init () {
    bzero(recv_buf, sizeof(recv_buf));
    buf_pos = recv_buf;
    recv_len = 0;
    recv_done = 0;
}
int read_line_socket (char *line, int s) {
    int r;
    char c;
    char *start_line = line;
    ESP_LOGV(TAG, "read_line_socket: <enter>");
    if (recv_done) {
        ESP_LOGV(TAG, "read_line_socket: recv_done");
        return (0);
    }
    while (1) {
        if (recv_len == 0) {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            if (r > 256) {
                ESP_LOGE(TAG, "read_line_socket: recv_buf overflow!");
                return (-1);
            }
            if (r < 0) {
                ESP_LOGV(TAG, "read_line_socket: recv_len==0, r<0, return 0, line = '%s'", start_line);
                return (0);
            }
            if (r == 0) {
                *line = '\0';
                recv_done = 1;
                ESP_LOGV(TAG, "read_line_socket: recv_len==0, r==0, recv_done, line = '%s'", start_line);
                return (buf_pos != recv_buf);
            }
            recv_len = r;
            buf_pos = recv_buf;
        }
        c = *buf_pos;
        buf_pos++;
        recv_len--;
        if (c == '\n') {
            *line = '\0';
            ESP_LOGV(TAG, "read_line_socket: line = '%s'", start_line);
            return (1);
        } else if (c != '\r') {
            *line = c;
            line++;
        }
    }
}

int http_code;
int http_code_done;
int http_header_done;

int open_server (int *s, char *path)  {
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;

    ESP_LOGV(TAG, "open_server: path = '%s'", path);

    http_code = 0;
    http_code_done = false;
    http_header_done = false;
    read_line_socket_init();

    if ((xEventGroupGetBits (wifi_event_group) & CONNECTED_BIT) == 0) {
        ESP_LOGI(TAG, "Not Connected\n");
        return (-1);
    }
    gpio_set_level(GPIO_OUTPUT_CONNECTED_LED, 1);

    char port_str[80];
    sprintf (port_str, "%u", wificonfig_vals_rfid.port);
    int err = getaddrinfo(wificonfig_vals_rfid.host, port_str, &hints, &res);

    if (err != 0 || res == NULL) {
        ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
        if (res != NULL)
            freeaddrinfo(res);
        return (-1);
    }

    /* Code to print the resolved IP.
    Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
    addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
    ESP_LOGD(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

    *s = socket(res->ai_family, res->ai_socktype, 0);
    if(*s < 0) {
        ESP_LOGE(TAG, "... Failed to allocate socket.");
        freeaddrinfo(res);
        return (-1);
    }
    ESP_LOGD(TAG, "... allocated socket");

    if(connect(*s, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
        close(*s);
        freeaddrinfo(res);
        return (-1);
    }

    ESP_LOGD(TAG, "... connected");
    freeaddrinfo(res);

    char request[256];
    sprintf(request, "GET %s HTTP/1.1\r\nHost: %s\r\nUser-Agent: esp-idf/1.0 esp32\r\n\r\n", path, wificonfig_vals_rfid.host);
    ESP_LOGV(TAG, "request = '%s'", request);

    if (write(*s, request, strlen(request)) < 0) {
        ESP_LOGE(TAG, "... socket send failed");
        close(*s);
        return (-1);
    }
    ESP_LOGD(TAG, "... socket send success");

    struct timeval receiving_timeout;
    receiving_timeout.tv_sec = 5;
    receiving_timeout.tv_usec = 0;
    if (setsockopt(*s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
            sizeof(receiving_timeout)) < 0) {
        ESP_LOGE(TAG, "... failed to set socket receiving timeout");
        close(*s);
        return (-1);
    }
    ESP_LOGD(TAG, "... set socket receiving timeout success");

    return (0);
}


// Read one line of the body of a response from the server
// return value:  1 for sucessful read
//                0 for done reading
//               -1 for error
//
int read_server (char *body, int s) {

    /* Read HTTP response */
    body[0] = '\0';
    char line[256];
    int r;
    bzero(line, sizeof(line));
    while ((r = read_line_socket (line, s)) > 0) {
        ESP_LOGV(TAG, "Socket data = '%s'", line);
        if (strlen(line) == 0) {
            http_header_done = true;
        } else if (http_header_done) {
            strcpy(body, line);
            return (1);
        } else if (!http_code_done) {
            sscanf (line, "%*s %d %*s", &http_code);
            http_code_done = true;
            if (http_code != 200) {
               ESP_LOGE(TAG, "Bad HTTP return code (%d)", http_code);
            }
        }
    }
    if (r < 0) {
        ESP_LOGE(TAG, "...socket data not available");
        return (-1);
    }

    ESP_LOGI(TAG, "Done reading");
    return (0);
}

// Cached ACLs are stored in a linked list
//
typedef struct acl_item_s {
    unsigned long acl;
    struct acl_item_s *next;
} acl_item;

int acl_loaded = false;
acl_item *acl_cache = NULL;
acl_item *acl_last = NULL;

void insert_acl (unsigned long acl) {
    acl_item *new_entry = (acl_item *) malloc(sizeof(acl_item));
    new_entry->acl = acl;
    new_entry->next = NULL;
    if (acl_cache == NULL) {
        acl_cache = new_entry;
        acl_last = new_entry;
    } else {
        acl_last->next = new_entry;
        acl_last = new_entry;
    }
}

void free_acls (acl_item *pnt) {
    while (pnt != NULL) {
        acl_item *next = pnt->next;
        free (pnt);
        pnt = next;
    }
}

// Load ACL from server into a cache
//
int read_acl () {

    ESP_LOGI(TAG, "Reading ACL into cache");

    acl_item *old_acl_cache = acl_cache; // save list in case things go south
    acl_item *old_acl_last = acl_last;
    int old_acl_loaded = acl_loaded;

    acl_cache = NULL;
    acl_loaded = false;

    int s;
    char path[256];
    char response[256];
    sprintf(path, "/api/get-acl-0/%s", wificonfig_vals_rfid.acl);
    if (open_server (&s, path) != 0) {
        ESP_LOGI(TAG, "http open failed");
        blinkit_low(GPIO_OUTPUT_CONNECTED_LED, 5, 250);
        acl_cache = old_acl_cache;
        acl_last = old_acl_last;
        acl_loaded = old_acl_loaded;
        return (false);
    }
    int err;
    int cnt = 0;
    // the response should be a list of acls
    //
    while ((err = read_server (response, s)) > 0) {
        ESP_LOGV(TAG, "http response body: %s", response);
        unsigned long new_acl;
        int vld = sscanf (response, "%lu", &new_acl);
        if (vld == 1) {
            cnt++;
            insert_acl (new_acl);
        }
    }
    ESP_LOGD(TAG, "... done reading from socket.");
    close(s);
    if ((err == 0) && (cnt > 0)) {
        ESP_LOGI(TAG, "New ACL successfully cached %d entries", cnt);
        free_acls(old_acl_cache);
        acl_loaded = true;
        ESP_LOGI(TAG, "Heap size = %d", xPortGetFreeHeapSize());
        return (true);
    } else if ((err == 0) && (cnt == 0)) {
        ESP_LOGE(TAG, "http read returned no valid ACLs");
        free_acls(acl_cache);
        acl_cache = old_acl_cache;
        acl_last = old_acl_last;
        acl_loaded = old_acl_loaded;
        blinkit_low(GPIO_OUTPUT_CONNECTED_LED, 3, 250);
    } else {
        ESP_LOGE(TAG, "http read failed");
        free_acls(acl_cache);
        acl_cache = old_acl_cache;
        acl_last = old_acl_last;
        acl_loaded = old_acl_loaded;
        blinkit_low(GPIO_OUTPUT_CONNECTED_LED, 7, 250);
    }
    ESP_LOGI(TAG, "Heap size = %d", xPortGetFreeHeapSize());
    return (false);
}

// Check RFID against cached ACL
//
int query_rfid_cache (unsigned long rfid) {
  ESP_LOGI(TAG, "Checking cache for RFID=%lu ...", rfid);

  acl_item *pnt = acl_cache;
  while (pnt != NULL) {
      if (pnt->acl == rfid) {
          ESP_LOGI (TAG, "CACHE: ACL OK");
          return (1);
      }
      pnt = pnt->next;
  }
  ESP_LOGI (TAG, "CACHE: ACL Not Found");
  return (0);
}

// Validate the RFID with the server - this will always send at least one
// http GET request
//
// Return 1 if good
//        0 if bad
//       -1 if error on request
int query_rfid (unsigned long rfid, int update_cache) {

    if ((xEventGroupGetBits (wifi_event_group) & CONNECTED_BIT) == 0) {
        // If we aren't connected, no point in continuing
        //
        return (-1);
    }

    if (wificonfig_vals_rfid.cache) {
        // Cached

        // always load ACL cache when we can since we don't know when the list will change
        //
        if (update_cache) {
            int server_up = read_acl();
            if (!server_up) {
                ESP_LOGE (TAG, "Server failure");
                return(-1);
            }
        }
    }

    ESP_LOGI (TAG, "Querying the server (to log)...");

    int s;
    char path[256];
    char response[256];
    sprintf(path, "/api/check-access-0/%s/%lu", wificonfig_vals_rfid.acl, rfid);
    if (open_server (&s, path) != 0) {
        ESP_LOGE(TAG, "http open failed");
        blinkit_low(GPIO_OUTPUT_CONNECTED_LED, 5, 250);
        return (-1);
    }
    // there should only be one line of response: True or False
    //
    int err = read_server (response, s);
    ESP_LOGD(TAG, "... done reading from socket.");
    ESP_LOGI(TAG, "response='%s'", response);
    close(s);
    if (err >= 0) {
        ESP_LOGV(TAG, "http response body: %s", response);
        if (strcmp(response,"True") == 0) {
            return (true);
        } if (strcmp(response,"False") == 0) {
            return (false);
        } else {
            ESP_LOGE(TAG, "Bad response from server");
            blinkit_low(GPIO_OUTPUT_CONNECTED_LED, 7, 250);
        }
    } else {
        ESP_LOGE(TAG, "http read failed");
        blinkit_low(GPIO_OUTPUT_CONNECTED_LED, 7, 250);
    }
    return (-1);
}

static void rfid_main_loop (void *pvParameters)
{
    int result;
    unsigned long cardno;

    // Wait for the callback to set the CONNECTED_BIT in the
    //    event group.
    //
    // We only do this at startup
    //
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to AP");
    if (wificonfig_vals_rfid.cache) {
        if (!read_acl()) {
            ESP_LOGE (TAG, "Server failure");
        }
    }

    access_state = 0;

    while(1) {

        // see if data is coming from RFID reader
        //
        if (get_RFID(&cardno))  {
            ESP_LOGI(TAG, "cardno = %lu", cardno);
            int used_cache = false;

            if (wificonfig_vals_rfid.cache) {
                // Cache
                if (acl_loaded && ((result = query_rfid_cache (cardno)) >= 0)) {
                    used_cache = true;
                } else {
                    result = query_rfid (cardno, true);
                    used_cache = false;
                }
            } else {
                // No Cache
                result = query_rfid (cardno, false);
            }

            if (result == 1) {
                good_rfid_sequence();
                access_state = 1;
                publish_status ("ACCESS", 1);

                // We used the cache, go to the server to recache and log
                if (used_cache) {
                    query_rfid (cardno, true);
                }

                if (wificonfig_vals_rfid.moment) {
                    // Momentary
                    vTaskDelay(wificonfig_vals_rfid.mtime * 1000 / portTICK_RATE_MS);
                    ESP_LOGI (TAG, "Turning off relay power");
                    no_rfid_sequence();
                    access_state = 0;
                    publish_status ("ACCESS", 0);
                } else {
                    // wait for IN_RANGE signal to drop
                    while (gpio_get_level(GPIO_INPUT_IN_RANGE)) {
                        vTaskDelay(RANGE_CHECK_TIME / portTICK_RATE_MS);
                    }
                    ESP_LOGI (TAG, "In Range signal dropped");
                    ESP_LOGI (TAG, "Turning off relay power and logging");
                    no_rfid_sequence();
                    access_state = 0;
                    publish_status ("ACCESS", 0);
                    query_rfid(0, false);
                }

                // eat any card reads that happened when door was unlocked
                //
                uart_flush_input(UART_NUM_1);
            } else if (result == 0) {
                bad_rfid_sequence();
                if (used_cache) {
                    query_rfid (cardno, true);
                }
            } else {
                // problem with the server
                ESP_LOGI (TAG, "Server error. Turning off relay power");
                no_rfid_sequence();
            }

        }

       // chill for 100 mSec
       //
       vTaskDelay(100 / portTICK_RATE_MS);
    }
}

// send periodic updates
//
static void update_loop (void *pvParameters) {

    while (1) {
        if (wificonfig_vals_mqtt.update != 0) {
            publish_status ("STATE", access_state);
            vTaskDelay((60000 * wificonfig_vals_mqtt.update) / portTICK_RATE_MS);
        } else {
            // paranoia (should never get here)
            vTaskDelay(60000 / portTICK_RATE_MS);
        }
    }
}


void app_main()
{
    initialize_uart();
    initialize_pins();

    // tasks related to wifi-based configuration
    TaskHandle_t xBlinkHandle = NULL;
    xTaskCreate(&strobe_leds, "strobe_leds", 4096, NULL, 5, &xBlinkHandle);
    wificonfig();
    if (xBlinkHandle != NULL) {
        vTaskDelete (xBlinkHandle);
        gpio_set_level(GPIO_OUTPUT_CONNECTED_LED, 0);
        gpio_set_level(GPIO_OUTPUT_ACCESS_LED, 0);
        gpio_set_level(GPIO_OUTPUT_FAIL_LED, 0);
    }
    load_aps();
    if (ap_count == 0) {
        // if no valid ssids -> go back to config mode
        ESP_LOGI (TAG, "Whoa! No ssids configured");
        trigger_wificonfig();
    }
    xTaskCreate(&check_gpio0, "check_gpio0", 4096, NULL, 5, NULL);

    ESP_ERROR_CHECK( esp_event_loop_create_default() );


    initialize_wifi();
    initialize_mqtt();

    xTaskCreate(&rfid_main_loop, "rfid_main_loop", 4096, NULL, 5, NULL);

    if ((mqtt_client != NULL) && (wificonfig_vals_mqtt.update != 0)) {
        xTaskCreate(&update_loop, "update_loop", 4096, NULL, 5, NULL);
    }

}
