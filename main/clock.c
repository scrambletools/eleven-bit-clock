/*
 * 11-bit Clock example implementation
 * 
 * This code implements a clock that displays time on
 * an 11-pixel LED strip. The time is represented in
 * binary with 1 bit for AM/PM, 4 bits for hours (0-12),
 * and 6 bits for minutes (0-59). Time is obtained from
 * an NTP server which can be configured via web interface.
 * Other config options allow color customization with
 * multiple presets and a clock password to prevent
 * unauthorized access.
 * 
 * This code also implements a wifi station and fallback
 * AP mode with captive portal to configure the wifi
 * station login credentials. If the device is unable to
 * connect to wifi after multiple tries, it will fall back
 * to AP mode and flash the leds to indicate this mode.
 * 
 * It also implements use of the ESP ADC for touch input.
 * When the user touches the clock (some metal part connected
 * to the configured GPIO), the clock will display the last
 * quad of its IP address allowing the user to identify the
 * device IP to access the web interface.
 * 
 * This codes has been tested on the ESP32-C3, but should
 * work on other ESP32 variants as well.
*/
#include <string.h>
#include <ctype.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/FreeRTOSConfig.h>
#include <esp_mac.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif_net_stack.h>
#include <esp_netif.h>
#include <time.h>
#include <sys/time.h>
#include "esp_netif_sntp.h"
#include "lwip/ip_addr.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_http_server.h"
#include "dns_server.h"
#include "led_strip.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"

// GPIO assignment for touch input
#define TOUCH_ADC_CHANNEL 4
// GPIO assignment for touch input comparator
#define TOUCH_COMP_CHANNEL 3
// GPIO assignment for LED strip
#define LED_STRIP_GPIO_PIN  2
// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 11
// Timeout for identify mode
#define IDENTIFY_TIMEOUT 20000
// Touch delta threshold 
// (difference between touch and non-touch readings)
#define TOUCH_DELTA_THRESHOLD 320
// Touch jitter threshold 
// (difference between consecutive readings while touching)
#define TOUCH_JITTER_THRESHOLD 150
// Touch count threshold 
// (number of readings while touching to trigger identify mode)
#define TOUCH_COUNT_THRESHOLD 3

/* WiFi configuration that you can set via project configuration menu.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_ESP_WIFI_STA_SSID "mywifissid"
*/

/* STA Configuration */
#define EXAMPLE_ESP_WIFI_STA_SSID           CONFIG_ESP_WIFI_REMOTE_AP_SSID
#define EXAMPLE_ESP_WIFI_STA_PASSWD         CONFIG_ESP_WIFI_REMOTE_AP_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY           CONFIG_ESP_MAXIMUM_STA_RETRY

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_WAPI_PSK
#endif

/* AP Configuration */
#define EXAMPLE_ESP_WIFI_AP_SSID            CONFIG_ESP_WIFI_AP_SSID
#define EXAMPLE_ESP_WIFI_AP_PASSWD          CONFIG_ESP_WIFI_AP_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL            CONFIG_ESP_WIFI_AP_CHANNEL
#define EXAMPLE_MAX_STA_CONN                CONFIG_ESP_MAX_STA_CONN_AP

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/*DHCP server option*/
#define DHCPS_OFFER_DNS    0x02

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/* Google API URLs */
#define GOOGLE_GEOLOCATION_API_URL "https://www.googleapis.com/geolocation/v1/geolocate?key=%s"
/* example geolocation post data 
Content-Type: application/json
Body: {
  "considerIp": "false",
  "wifiAccessPoints": [
    {
      "macAddress": "3c:37:86:5d:75:d4",
      "signalStrength": -35,
      "signalToNoiseRatio": 0
    },
    {
      "macAddress": "30:86:2d:c4:29:d0",
      "signalStrength": -35,
      "signalToNoiseRatio": 0
    }
  ]
}
*/
/* example geolocation response
{
  "location": {
    "lat": 37.4241173,
    "lng": -122.0915717
  },
  "accuracy": 20
}
*/
#define GOOGLE_TIMEZONE_API_URL "https://maps.googleapis.com/maps/api/timezone/json?location=%f,%f&timestamp=%d&key=%s"
/* example request params
location=39.6034810%2C-119.6822510&timestamp=1733428634
*/
/* example timezone response
{
  "dstOffset": 3600,
  "rawOffset": -28800,
  "status": "OK",
  "timeZoneId": "America/Los_Angeles",
  "timeZoneName": "Pacific Daylight Time",
}
*/

/* ADC defines for touch detection */

#define EXAMPLE_ADC_UNIT                    ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit)         #unit
#define EXAMPLE_ADC_UNIT_STR(unit)          _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_0
#define EXAMPLE_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#define EXAMPLE_READ_LEN                    256

/* Root page form fields validation status */
#define FORM_VAL_STATUS_OK     0
#define FORM_VAL_STATUS_RESTART 1
#define FORM_VAL_STATUS_ERROR  -1
#define FORM_VAL_STATUS_AUTH_INVALID  -2
#define FORM_VAL_STATUS_FIELD_INVALID  -3

/* HTML templates (copied from source files to flash) */

// root.html
extern const char root_template_start[] asm("_binary_root_html_start");
extern const char root_template_end[] asm("_binary_root_html_end");

// setup_root.html
extern const char setup_root_template_start[] asm("_binary_setup_root_html_start");
extern const char setup_root_template_end[] asm("_binary_setup_root_html_end");

// styles.css
extern const char style_template_start[] asm("_binary_style_css_start");
extern const char style_template_end[] asm("_binary_style_css_end");

// response.html
extern const char response_template_start[] asm("_binary_response_html_start");
extern const char response_template_end[] asm("_binary_response_html_end");

/* Data structures */

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t w;
} color_t; // 4 bytes

typedef struct {
    char    name[16];
    color_t am_color;
    color_t pm_color;
    color_t hr0_color;
    color_t hr1_color;
    color_t min0_color;
    color_t min1_color;
} preset_t; // 46 bytes

typedef struct {
    char     ntp_server_1[32];
    char     ntp_server_2[32];
    int8_t   gmt_offset; // -12 to 12
    char     wifi_ssid[32];
    char     wifi_password[32];
    char     clock_password[32];
    uint8_t  active_preset; // 1, 2 or 3
    preset_t preset_1;
    preset_t preset_2;
    preset_t preset_3;
} config_t; // 268 bytes

/* HTTPd route types */   
typedef enum {
    ROUTE_ROOT,
    ROUTE_SETUP_ROOT,
    ROUTE_CONFIG,
    ROUTE_WIFI,
    ROUTE_CSS
} route_t;

/* App modes for app state and app event group */
typedef enum {
    APP_MODE_STARTUP = 0b00000001,
    APP_MODE_NORMAL = 0b00000010,
    APP_MODE_IDENTIFY = 0b00000100,
    APP_MODE_SETUP = 0b00001000
} app_mode_t;

/* Tags */
static const char *TAG     = "pyramid_clock";
static const char *TAG_AP  = "wifi_ap";
static const char *TAG_STA = "wifi_sta";

/* Rety counter for wifi connection */
static int s_retry_num = 0;

// LED strip type
// options: LED_MODEL_WS2812, LED_MODEL_SK6812
int led_type = LED_MODEL_SK6812;

// LED strip handle
led_strip_handle_t *led_strip;

// NVS handle
nvs_handle_t storage_handle;

// App configuration (stored in NVS)
config_t *app_config;

// App mode
app_mode_t app_mode = APP_MODE_STARTUP;

// Device IP address
esp_ip4_addr_t device_ip;

// ADC channels and task handle
// Using GPIO 2; if more touch inputs are needed, add to this array
static adc_channel_t channel[2] = {TOUCH_ADC_CHANNEL, TOUCH_COMP_CHANNEL}; 
static TaskHandle_t s_task_handle;

/* FreeRTOS event groups */
static EventGroupHandle_t s_wifi_event_group;
static EventGroupHandle_t s_app_event_group;

void info_lwm(char *task_name, char *comment) {
    TaskHandle_t task_handle = xTaskGetHandle(task_name);
    ESP_LOGI(task_name, "%s (lwm: %d)", comment, uxTaskGetStackHighWaterMark(task_handle));
}

void print_mem_stats() {
    ESP_LOGI(TAG, "-----------MEMORY STATS------------");
    TaskStatus_t sys_stat[10];
    UBaseType_t num_tasks = uxTaskGetSystemState(sys_stat, 10, NULL);
    for (UBaseType_t i = 0; i < num_tasks; i++) {
        ESP_LOGI(TAG, "%s lowest: %lu", sys_stat[i].pcTaskName, sys_stat[i].usStackHighWaterMark);
    }
    ESP_LOGI(TAG, "Heap free: %d", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    ESP_LOGI(TAG, "Heap max alloc: %d", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    ESP_LOGI(TAG, "-----------------------------------");
}

// Function to replace substrings in a string (used for html templates)
// You can limit the use of memory by setting low maxChunkSize
void replace_in_chunks(
    char *orig,
    size_t origMaxSize,
    const char *matchStrings[],
    const char *replacementStrings[],
    size_t numMatches,
    size_t maxChunkSize
) {
    typedef struct {
        size_t pos;        // position in orig string where match is found
        size_t matchLen;   // length of the match string
        size_t matchIndex; // which match string was found
    } MatchOccurrence;

    // Gather all match occurrences
    MatchOccurrence *occurrences = NULL;
    size_t occCap = 0, occCount = 0;

    for (size_t i = 0; i < numMatches; i++) {
        const char *mStr = matchStrings[i];
        size_t mLen = strlen(mStr);
        if (mLen == 0) continue; // skip empty match
        
        const char *searchStart = orig;
        while (1) {
            const char *found = strstr(searchStart, mStr);
            if (!found) break;

            // Expand occurrences array if needed
            if (occCount == occCap) {
                occCap = occCap == 0 ? 16 : occCap * 2;
                occurrences = realloc(occurrences, occCap * sizeof(MatchOccurrence));
            }
            occurrences[occCount].pos       = (size_t)(found - orig);
            occurrences[occCount].matchLen  = mLen;
            occurrences[occCount].matchIndex= i;
            occCount++;

            searchStart = found + 1; // move past this match
        }
    }
    if (occCount == 0) {
        free(occurrences);
        return; // No matches to replace
    }
    // Sort occurrences by position
    for (size_t i = 0; i < occCount; i++) {
        for (size_t j = i + 1; j < occCount; j++) {
            if (occurrences[j].pos < occurrences[i].pos) {
                MatchOccurrence tmp = occurrences[i];
                occurrences[i] = occurrences[j];
                occurrences[j] = tmp;
            }
        }
    }

    // Current length of the string
    size_t curLen = strlen(orig);
    long offset = 0; // how much we've shifted so far

    // Replace all occurrences
    for (size_t i = 0; i < occCount; i++) {
        size_t realPos = occurrences[i].pos + offset;
        if (realPos > curLen) break; // out of range if the string changed drastically

        size_t matchLen = occurrences[i].matchLen;
        const char *repl = replacementStrings[occurrences[i].matchIndex];
        size_t replLen = strlen(repl);

        // Check if the match is fully in range
        if (realPos + matchLen > curLen) continue;

        // How much the string size changes
        long delta = (long)replLen - (long)matchLen;

        // Make sure we won't overflow
        if (delta > 0 && (curLen + delta + 1) > origMaxSize) {
            ESP_LOGE(TAG, "Can't fit replacement - need %lu bytes, have %u", curLen + delta + 1, origMaxSize);
            continue;
        }

        // First, shift the rest of the string to make room (or close space)
        if (delta != 0) {
            // Calculate how many bytes we need to move
            size_t bytesToMove = curLen - (realPos + matchLen) + 1; // +1 for null terminator
            
            if (bytesToMove > 0) {
                // Move the existing content to make space for the larger replacement
                // or close the gap for a smaller replacement
                memmove(orig + realPos + replLen,
                    orig + realPos + matchLen,
                    bytesToMove);
            }
        }

        // Now copy the replacement string
        memcpy(orig + realPos, repl, replLen);

        // Update the current length and offset
        curLen += delta;
        offset += delta;

        // Add debug logging
        //ESP_LOGI(TAG, "Replaced match at pos %zu, delta: %ld, new len: %zu", realPos, delta, curLen);
    }
    free(occurrences);
}

/* Function to decode a URL-encoded string */
void url_decode(char *dst, const char *src) {
    char a, b;
    while (*src) {
        if ((*src == '%') && ((a = src[1]) && (b = src[2])) && (isxdigit(a) && isxdigit(b))) {
            if (a >= 'a') a -= 'a' - 'A';
            if (a >= 'A') a -= ('A' - 10);
            else a -= '0';
            if (b >= 'a') b -= 'a' - 'A';
            if (b >= 'A') b -= ('A' - 10);
            else b -= '0';
            *dst++ = 16 * a + b;
            src += 3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = '\0';
}

color_t hex_to_color(const char *hex) {
    color_t color = {0, 0, 0, 0}; // Initialize with default values

    if (hex[0] == '#') {
        hex++; // Skip the '#' character
    }

    if (strlen(hex) == 6) {
        char r_str[3] = {hex[0], hex[1], '\0'};
        char g_str[3] = {hex[2], hex[3], '\0'};
        char b_str[3] = {hex[4], hex[5], '\0'};

        color.r = (unsigned char)strtol(r_str, NULL, 16);
        color.g = (unsigned char)strtol(g_str, NULL, 16);
        color.b = (unsigned char)strtol(b_str, NULL, 16);
    }

    return color;
}

/* Set the timezone */
void set_timezone(int8_t offset) {
    char *tz_name = "UTC";
    char tz_str[8];
    if (offset > 0) {
        sprintf(tz_str, "%s%-d", tz_name, offset); // change offset direction
    } else {
        if (offset < 0) {
            offset = -offset;
            sprintf(tz_str, "%s%+d", tz_name, offset); // change offset direction
        } else {
            sprintf(tz_str, "%s+0", tz_name);
        }
    } 
    ESP_LOGI(TAG, "TZ: %s", tz_str);
    setenv("TZ", tz_str, 1);
    tzset();
}

/* Function to extract 's' and 'p' parameters from a URL-encoded string */
void extract_wifi_params(const char *query, char *s_value, char *p_value, size_t size) {
    char *query_copy = strdup(query);
    if (!query_copy) {
        perror("Failed to allocate memory");
        return;
    }
    ESP_LOGI(TAG, "query_copy: %s", query_copy);
    char *token = strtok(query_copy, "&");
    while (token != NULL) {
        // Manually find the '=' character
        char *key = token;
        char *value = strchr(token, '=');

        if (value) {
            *value = '\0'; // Null-terminate the key
            value++;       // Move to the start of the value

            char decoded_value[256];
            url_decode(decoded_value, value);
            ESP_LOGI(TAG, "value: %s", value);

            if (strcmp(key, "s") == 0) {
                memset(s_value, 0, size);
                memcpy(s_value, decoded_value, strlen(decoded_value));
            } else if (strcmp(key, "p") == 0) {
                memset(p_value, 0, size);
                memcpy(p_value, decoded_value, strlen(decoded_value));
            }
        }
        token = strtok(NULL, "&");
    }
    free(query_copy);
}

/* Function to set config from a URL-encoded string */
int set_config_from_params(const char *query, config_t *config) {

    int ret = FORM_VAL_STATUS_OK;
    char *query_copy = strdup(query);
    if (!query_copy) {
        perror("Failed to allocate memory");
        return FORM_VAL_STATUS_ERROR;
    }
    char *token = strtok(query_copy, "&");
    char *temp_password = "";
    while (token != NULL) {
        // Manually find the '=' character
        char *key = token;
        char *value = strchr(token, '=');

        if (value) {
            *value = '\0'; // Null-terminate the key
            value++;       // Move to the start of the value

            char decoded_value[256];
            url_decode(decoded_value, value);
            ESP_LOGI(TAG, "%s = %s", key, decoded_value);

            // check that the correct password is provided for authentication
            if (strcmp(key, "p") == 0) {
                if (strcmp(decoded_value, config->clock_password) != 0) {
                    ESP_LOGE(TAG, "Incorrect password");
                    ESP_LOGI(TAG, "config->clock_password = %s", config->clock_password);
                    return FORM_VAL_STATUS_AUTH_INVALID;
                }
            }

            // password and password confirm
            if (strcmp(key, "np") == 0 || strcmp(key, "npc") == 0) {
                // if the field is not empty, remember it or compare with existing temp_password
                if (strcmp(decoded_value, "") != 0) {
                    if (strcmp(temp_password, "") == 0) {
                        // if temp_password is not set, set it
                        temp_password = strdup(decoded_value);
                        if (!temp_password) {
                            ESP_LOGE(TAG, "Failed to allocate memory for temp_password");
                            return FORM_VAL_STATUS_ERROR;
                        }
                    }
                    else {
                        // if the password and confirm match, set the password
                        if (strcmp(temp_password, decoded_value) == 0) {
                            strncpy(config->clock_password, decoded_value, sizeof(config->clock_password) - 1);
                        }
                        else {
                            ESP_LOGE(TAG, "Password and confirm do not match");
                            return FORM_VAL_STATUS_FIELD_INVALID;
                        }
                    }
                }
                else {
                    // if password confirm is empty, but password_temp is not empty, then return error
                    if (strcmp(key, "npc") == 0 && strcmp(temp_password, "") != 0) {
                        ESP_LOGE(TAG, "Password confirm is empty, but password_temp is not empty");
                        return FORM_VAL_STATUS_FIELD_INVALID;
                    }
                }
            // ntp server 1
            } else if (strcmp(key, "ntp_server_1") == 0) {
                strncpy(config->ntp_server_1, decoded_value, sizeof(config->ntp_server_1) - 1);
            // ntp server 2
            } else if (strcmp(key, "ntp_server_2") == 0) {
                strncpy(config->ntp_server_2, decoded_value, sizeof(config->ntp_server_2) - 1);
            // clock gmt offset
            } else if (strcmp(key, "gmt_offset") == 0) {
                config->gmt_offset = atoi(decoded_value);
            // active preset
            } else if (strcmp(key, "active_preset") == 0) {
                config->active_preset = atoi(decoded_value);
            // preset 1 name
            } else if (strcmp(key, "p1_name") == 0) {
                strncpy(config->preset_1.name, decoded_value, sizeof(config->preset_1.name) - 1);
            // preset 1 AM color
            } else if (strcmp(key, "p1_am_color") == 0) {
                config->preset_1.am_color = hex_to_color(decoded_value);
            // preset 1 AM white
            } else if (strcmp(key, "p1_am_white") == 0) {
                config->preset_1.am_color.w = atoi(decoded_value);
            // preset 1 PM color
            } else if (strcmp(key, "p1_pm_color") == 0) {
                config->preset_1.pm_color = hex_to_color(decoded_value);
            // preset 1 PM white
            } else if (strcmp(key, "p1_pm_white") == 0) {
                config->preset_1.pm_color.w = atoi(decoded_value);
            // preset 1 hour color 0
            } else if (strcmp(key, "p1_hr0_color") == 0) {
                config->preset_1.hr0_color = hex_to_color(decoded_value);
            // preset 1 hour color 0 white
            } else if (strcmp(key, "p1_hr0_white") == 0) {
                config->preset_1.hr0_color.w = atoi(decoded_value);
            // preset 1 hour color 1
            } else if (strcmp(key, "p1_hr1_color") == 0) {
                config->preset_1.hr1_color = hex_to_color(decoded_value);
            // preset 1 hour color 1 white
            } else if (strcmp(key, "p1_hr1_white") == 0) {
                config->preset_1.hr1_color.w = atoi(decoded_value);
            // preset 1 min color 0
            } else if (strcmp(key, "p1_min0_color") == 0) {
                config->preset_1.min0_color = hex_to_color(decoded_value);
            // preset 1 min color 0 white
            } else if (strcmp(key, "p1_min0_white") == 0) {
                config->preset_1.min0_color.w = atoi(decoded_value);
            // preset 1 min color 1
            } else if (strcmp(key, "p1_min1_color") == 0) {
                config->preset_1.min1_color = hex_to_color(decoded_value);
            // preset 1 min color 1 white
            } else if (strcmp(key, "p1_min1_white") == 0) {
                config->preset_1.min1_color.w = atoi(decoded_value);
            // preset 2 name
            } else if (strcmp(key, "p2_name") == 0) {
                strncpy(config->preset_2.name, decoded_value, sizeof(config->preset_2.name) - 1);
            // preset 2 AM color
            } else if (strcmp(key, "p2_am_color") == 0) {
                config->preset_2.am_color = hex_to_color(decoded_value);
            // preset 2 AM white
            } else if (strcmp(key, "p2_am_white") == 0) {
                config->preset_2.am_color.w = atoi(decoded_value);
            // preset 2 PM color
            } else if (strcmp(key, "p2_pm_color") == 0) {
                config->preset_2.pm_color = hex_to_color(decoded_value);
            // preset 2 PM white
            } else if (strcmp(key, "p2_pm_white") == 0) {
                config->preset_2.pm_color.w = atoi(decoded_value);
            // preset 2 hour color 0
            } else if (strcmp(key, "p2_hr0_color") == 0) {
                config->preset_2.hr0_color = hex_to_color(decoded_value);
            // preset 2 hour color 0 white
            } else if (strcmp(key, "p2_hr0_white") == 0) {
                config->preset_2.hr0_color.w = atoi(decoded_value);
            // preset 2 hour color 1
            } else if (strcmp(key, "p2_hr1_color") == 0) {
                config->preset_2.hr1_color = hex_to_color(decoded_value);
            // preset 2 hour color 1 white
            } else if (strcmp(key, "p2_hr1_white") == 0) {
                config->preset_2.hr1_color.w = atoi(decoded_value);
            // preset 2 min color 0
            } else if (strcmp(key, "p2_min0_color") == 0) {
                config->preset_2.min0_color = hex_to_color(decoded_value);
            // preset 2 min color 0 white
            } else if (strcmp(key, "p2_min0_white") == 0) {
                config->preset_2.min0_color.w = atoi(decoded_value);
            // preset 2 min color 1
            } else if (strcmp(key, "p2_min1_color") == 0) {
                config->preset_2.min1_color = hex_to_color(decoded_value);
            // preset 2 min color 1 white
            } else if (strcmp(key, "p2_min1_white") == 0) {
                config->preset_2.min1_color.w = atoi(decoded_value);
            // preset 3 name
            } else if (strcmp(key, "p3_name") == 0) {
                strncpy(config->preset_3.name, decoded_value, sizeof(config->preset_3.name) - 1);
            // preset 3 AM color
            } else if (strcmp(key, "p3_am_color") == 0) {
                config->preset_3.am_color = hex_to_color(decoded_value);
            // preset 3 AM white
            } else if (strcmp(key, "p3_am_white") == 0) {
                config->preset_3.am_color.w = atoi(decoded_value);
            // preset 3 PM color
            } else if (strcmp(key, "p3_pm_color") == 0) {
                config->preset_3.pm_color = hex_to_color(decoded_value);
            // preset 3 PM white
            } else if (strcmp(key, "p3_pm_white") == 0) {
                config->preset_3.pm_color.w = atoi(decoded_value);
            // preset 3 hour color 0
            } else if (strcmp(key, "p3_hr0_color") == 0) {
                config->preset_3.hr0_color = hex_to_color(decoded_value);
            // preset 3 hour color 0 white
            } else if (strcmp(key, "p3_hr0_white") == 0) {
                config->preset_3.hr0_color.w = atoi(decoded_value);
            // preset 3 hour color 1
            } else if (strcmp(key, "p3_hr1_color") == 0) {
                config->preset_3.hr1_color = hex_to_color(decoded_value);
            // preset 3 hour color 1 white
            } else if (strcmp(key, "p3_hr1_white") == 0) {
                config->preset_3.hr1_color.w = atoi(decoded_value);
            // preset 3 min color 0
            } else if (strcmp(key, "p3_min0_color") == 0) {
                config->preset_3.min0_color = hex_to_color(decoded_value);
            // preset 3 min color 0 white
            } else if (strcmp(key, "p3_min0_white") == 0) {
                config->preset_3.min0_color.w = atoi(decoded_value);
            // preset 3 min color 1
            } else if (strcmp(key, "p3_min1_color") == 0) {
                config->preset_3.min1_color = hex_to_color(decoded_value);
            // preset 3 min color 1 white
            } else if (strcmp(key, "p3_min1_white") == 0) {
                config->preset_3.min1_color.w = atoi(decoded_value);
            } else if (strcmp(key, "clear_wifi") == 0 && strcmp(decoded_value, "on") == 0) {
                strncpy(config->wifi_ssid, "", sizeof(config->wifi_ssid) - 1);
                strncpy(config->wifi_password, "", sizeof(config->wifi_password) - 1);
                ESP_LOGI(TAG, "clearing wifi");
                ret = FORM_VAL_STATUS_RESTART;
            }
        }
        token = strtok(NULL, "&");
    }
    free(query_copy);
    
    // return true if device needs to restart due to config change
    return ret;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
        ESP_LOGI(TAG_AP, "Station "MACSTR" joined, AID=%d",
                    MAC2STR(event->mac), event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
        ESP_LOGI(TAG_AP, "Station "MACSTR" left, AID=%d, reason:%d",
                    MAC2STR(event->mac), event->aid, event->reason);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        memcpy(&device_ip, &event->ip_info.ip, sizeof(esp_ip4_addr_t));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* Initialize soft AP */
esp_netif_t *wifi_init_softap(void)
{
    esp_netif_t *esp_netif_ap = esp_netif_create_default_wifi_ap();
    wifi_config_t wifi_ap_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_AP_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_AP_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_AP_PASSWD,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    if (strlen(EXAMPLE_ESP_WIFI_AP_PASSWD) == 0) {
        wifi_ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));

    ESP_LOGI(TAG_AP, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_AP_SSID, EXAMPLE_ESP_WIFI_AP_PASSWD, EXAMPLE_ESP_WIFI_CHANNEL);

    return esp_netif_ap;
}

/* Initialize wifi station */
esp_netif_t *wifi_init_sta(void)
{
    esp_netif_t *esp_netif_sta = esp_netif_create_default_wifi_sta();
    wifi_config_t wifi_sta_config = {
        .sta = {
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .failure_retry_cnt = EXAMPLE_ESP_MAXIMUM_RETRY,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    memcpy(wifi_sta_config.sta.ssid, app_config->wifi_ssid, strlen(app_config->wifi_ssid));
    memcpy(wifi_sta_config.sta.password, app_config->wifi_password, strlen(app_config->wifi_password));

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config) );

    ESP_LOGI(TAG_STA, "wifi_init_sta finished. SSID:%s password:%s",
             app_config->wifi_ssid, app_config->wifi_password);

    return esp_netif_sta;
}

void softap_set_dns_addr(esp_netif_t *esp_netif_ap, esp_netif_t *esp_netif_sta) {
    esp_netif_dns_info_t dns;
    esp_netif_get_dns_info(esp_netif_sta, ESP_NETIF_DNS_MAIN, &dns);
    uint8_t dhcps_offer_option = DHCPS_OFFER_DNS;
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_stop(esp_netif_ap));
    ESP_ERROR_CHECK(esp_netif_dhcps_option(esp_netif_ap, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &dhcps_offer_option, sizeof(dhcps_offer_option)));
    ESP_ERROR_CHECK(esp_netif_set_dns_info(esp_netif_ap, ESP_NETIF_DNS_MAIN, &dns));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_start(esp_netif_ap));
}

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "NTP time synchronized");
    //ESP_LOGI(TAG, "NTP time synchronized, time: %s", ctime((const time_t *)tv->tv_sec));
}

/* Save config to NVS */
static int save_config(void) {

    // open NVS
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    int ret = nvs_open("storage", NVS_READWRITE, &storage_handle);
    if (ret != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
        ESP_ERROR_CHECK(ret);
    }

    size_t size = sizeof(config_t);
    ESP_LOGI(TAG_STA, "Saving config: %s %s, size: %d", app_config->wifi_ssid, app_config->wifi_password, size);
    esp_err_t err = nvs_set_blob(storage_handle, "app_config", app_config, size);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    printf("Committing updates in NVS ... ");
    err = nvs_commit(storage_handle);
    printf((err != ESP_OK) ? "NVS write failed!\n" : "NVS write done.\n");

    nvs_close(storage_handle);
    return err;
}

/* Get config from NVS */
static int load_config(void) {

    // open NVS
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    int ret = nvs_open("storage", NVS_READWRITE, &storage_handle);
    if (ret != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
        ESP_ERROR_CHECK(ret);
    }
    // get the config blob from NVS
    size_t size = sizeof(config_t);
    ret = nvs_get_blob(storage_handle, "app_config", app_config, &size);
    switch (ret) {
        case ESP_OK:
            //memcpy(&app_config, config_str, size);
            printf("READ config ssid = %s\n", app_config->wifi_ssid);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("app_config is not found in NVS!\n");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(ret));
    }
    nvs_close(storage_handle);
    return ret;
}

/* Set the captive portal URL */
static void dhcp_set_captiveportal_url(void) {
    // get the IP of the access point to redirect to
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);

    char ip_addr[16];
    inet_ntoa_r(ip_info.ip.addr, ip_addr, 16);
    ESP_LOGI(TAG, "Set up softAP with IP: %s", ip_addr);

    // turn the IP into a URI
    char* captiveportal_uri = (char*) malloc(32 * sizeof(char));
    assert(captiveportal_uri && "Failed to allocate captiveportal_uri");
    strcpy(captiveportal_uri, "http://");
    strcat(captiveportal_uri, ip_addr);

    // get a handle to configure DHCP with
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

    // set the DHCP option 114
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_stop(netif));
    ESP_ERROR_CHECK(esp_netif_dhcps_option(netif, ESP_NETIF_OP_SET, ESP_NETIF_CAPTIVEPORTAL_URI, captiveportal_uri, strlen(captiveportal_uri)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_start(netif));
}

/* HTTP GET setup root Handler
 * Loads the setup_root.html template, replaces all handlebar tokens
 * with the current config values and sends the result to the client */
static esp_err_t setup_root_get_handler(httpd_req_t *req) {

    const uint32_t page_size = setup_root_template_end - setup_root_template_start;

    size_t size = page_size + 64; // add 64 to the size to account for filling the template
    char* page = (char*) malloc(size); 
    assert(page && "Failed to allocate page");
    memset(page, 0, size);
    memcpy(page, setup_root_template_start, page_size);

    // make an array of match strings and an array of replacement strings
    const char* match_strings[] = {
        "{{wifi_ssid}}"
    };
    const char* replacement_strings[] = {
        app_config->wifi_ssid
    };
    // limit the chunk size to 64 bytes to avoid memory issues
    size_t max_chunk_size = 64;
    // replace substrings in the page
    replace_in_chunks(page, sizeof(page), match_strings, replacement_strings, sizeof(match_strings) / sizeof(match_strings[0]), max_chunk_size);

    // send the page
    httpd_resp_set_type(req, "text/html");
    int ret = httpd_resp_send(req, page, page_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send response");
        return ESP_FAIL;
    }
    info_lwm("httpd", "Served setup root");
    
    // free the page
    free(page);

    return ESP_OK;
}

/* HTTP GET root Handler
 * Loads the root.html template, replaces all handlebar tokens
 * with the current config values and sends the result to the client */
static esp_err_t root_get_handler(httpd_req_t *req) {

    // as this is a lengthy operation, we'll copy the app_config
    // in case it may be accessed by another task
    config_t config;
    memcpy(&config, app_config, sizeof(config_t));

    // setup array of match strings
    const size_t max_size_match_string = 20;
    const char *match_strings[] = {
        "{{ntp_server_1}}",
        "{{ntp_server_2}}",
        "{{gmt_offset}}",
        "{{active_preset}}",
        "{{p1_name}}",
        "{{p1_am_color}}",
        "{{p1_am_white}}",
        "{{p1_pm_color}}",
        "{{p1_pm_white}}",
        "{{p1_hr0_color}}",
        "{{p1_hr0_white}}",
        "{{p1_hr1_color}}",
        "{{p1_hr1_white}}",
        "{{p1_min0_color}}",
        "{{p1_min0_white}}",
        "{{p1_min1_color}}",
        "{{p1_min1_white}}",
        "{{p2_name}}",
        "{{p2_am_color}}",
        "{{p2_am_white}}",
        "{{p2_pm_color}}",
        "{{p2_pm_white}}",
        "{{p2_hr0_color}}",
        "{{p2_hr0_white}}",
        "{{p2_hr1_color}}",
        "{{p2_hr1_white}}",
        "{{p2_min0_color}}",
        "{{p2_min0_white}}",
        "{{p2_min1_color}}",
        "{{p2_min1_white}}",
        "{{p3_name}}",
        "{{p3_am_color}}",
        "{{p3_am_white}}",
        "{{p3_pm_color}}",
        "{{p3_pm_white}}",
        "{{p3_hr0_color}}",
        "{{p3_hr0_white}}",
        "{{p3_hr1_color}}",
        "{{p3_hr1_white}}",
        "{{p3_min0_color}}",
        "{{p3_min0_white}}",
        "{{p3_min1_color}}",
        "{{p3_min1_white}}",
    };
    const size_t num_match_strings = sizeof(match_strings) / sizeof(match_strings[0]);
    
    // build the replacement string for the gmt offset
    char gmt_offset_str[1260] = ""; // 50 * 25 + 10
    for (int i = 12; i >= -12; i--) {
        char option_str[60];
        char display_str[12];
        if (i == 0) {
            sprintf(display_str, "No offset");
        } else if (i > 0) {
            sprintf(display_str, "GMT +%d hrs", i);
        } else {
            sprintf(display_str, "GMT %d hrs", i);
        }
        if (i == config.gmt_offset) {
            sprintf(option_str, "<option value=\"%d\" selected>%s</option>\n", i, display_str);
        } else {
            sprintf(option_str, "<option value=\"%d\">%s</option>\n", i, display_str);
        }
        strcat(gmt_offset_str, option_str);
    }

    // build the replacement string for the active preset
    size_t num_presets = 3;
    char active_preset_str[160] = ""; // 50 * 3 + 10
    for (int i = 0; i < num_presets; i++) {
        int preset_num = i + 1;
        char option_str[50];
        if (preset_num == config.active_preset) {
            sprintf(option_str, "<option value=\"%d\" selected>%d</option>\n", preset_num, preset_num);
        } else {
            sprintf(option_str, "<option value=\"%d\">%d</option>\n", preset_num, preset_num);
        }
        strcat(active_preset_str, option_str);
    }

    // build the replacement string for the preset colors
    char preset_1_am_color_str[8] = "";
    sprintf(preset_1_am_color_str, "#%02X%02X%02X", config.preset_1.am_color.r, config.preset_1.am_color.g, config.preset_1.am_color.b);
    char preset_1_pm_color_str[8] = "";
    sprintf(preset_1_pm_color_str, "#%02X%02X%02X", config.preset_1.pm_color.r, config.preset_1.pm_color.g, config.preset_1.pm_color.b);
    char preset_1_hr0_color_str[8] = "";
    sprintf(preset_1_hr0_color_str, "#%02X%02X%02X", config.preset_1.hr0_color.r, config.preset_1.hr0_color.g, config.preset_1.hr0_color.b);
    char preset_1_hr1_color_str[8] = "";
    sprintf(preset_1_hr1_color_str, "#%02X%02X%02X", config.preset_1.hr1_color.r, config.preset_1.hr1_color.g, config.preset_1.hr1_color.b);
    char preset_1_min0_color_str[8] = "";
    sprintf(preset_1_min0_color_str, "#%02X%02X%02X", config.preset_1.min0_color.r, config.preset_1.min0_color.g, config.preset_1.min0_color.b);
    char preset_1_min1_color_str[8] = "";
    sprintf(preset_1_min1_color_str, "#%02X%02X%02X", config.preset_1.min1_color.r, config.preset_1.min1_color.g, config.preset_1.min1_color.b);
    char preset_2_am_color_str[8] = "";
    sprintf(preset_2_am_color_str, "#%02X%02X%02X", config.preset_2.am_color.r, config.preset_2.am_color.g, config.preset_2.am_color.b);
    char preset_2_pm_color_str[8] = "";
    sprintf(preset_2_pm_color_str, "#%02X%02X%02X", config.preset_2.pm_color.r, config.preset_2.pm_color.g, config.preset_2.pm_color.b);
    char preset_2_hr0_color_str[8] = "";
    sprintf(preset_2_hr0_color_str, "#%02X%02X%02X", config.preset_2.hr0_color.r, config.preset_2.hr0_color.g, config.preset_2.hr0_color.b);
    char preset_2_hr1_color_str[8] = "";
    sprintf(preset_2_hr1_color_str, "#%02X%02X%02X", config.preset_2.hr1_color.r, config.preset_2.hr1_color.g, config.preset_2.hr1_color.b);
    char preset_2_min0_color_str[8] = "";
    sprintf(preset_2_min0_color_str, "#%02X%02X%02X", config.preset_2.min0_color.r, config.preset_2.min0_color.g, config.preset_2.min0_color.b);
    char preset_2_min1_color_str[8] = "";
    sprintf(preset_2_min1_color_str, "#%02X%02X%02X", config.preset_2.min1_color.r, config.preset_2.min1_color.g, config.preset_2.min1_color.b);
    char preset_3_am_color_str[8] = "";
    sprintf(preset_3_am_color_str, "#%02X%02X%02X", config.preset_3.am_color.r, config.preset_3.am_color.g, config.preset_3.am_color.b);
    char preset_3_pm_color_str[8] = "";
    sprintf(preset_3_pm_color_str, "#%02X%02X%02X", config.preset_3.pm_color.r, config.preset_3.pm_color.g, config.preset_3.pm_color.b);
    char preset_3_hr0_color_str[8] = "";
    sprintf(preset_3_hr0_color_str, "#%02X%02X%02X", config.preset_3.hr0_color.r, config.preset_3.hr0_color.g, config.preset_3.hr0_color.b);
    char preset_3_hr1_color_str[8] = "";
    sprintf(preset_3_hr1_color_str, "#%02X%02X%02X", config.preset_3.hr1_color.r, config.preset_3.hr1_color.g, config.preset_3.hr1_color.b);
    char preset_3_min0_color_str[8] = "";
    sprintf(preset_3_min0_color_str, "#%02X%02X%02X", config.preset_3.min0_color.r, config.preset_3.min0_color.g, config.preset_3.min0_color.b);
    char preset_3_min1_color_str[8] = "";
    sprintf(preset_3_min1_color_str, "#%02X%02X%02X", config.preset_3.min1_color.r, config.preset_3.min1_color.g, config.preset_3.min1_color.b);

    // build the replacement strings for white leds
    char preset_1_am_white_str[4] = "";
    sprintf(preset_1_am_white_str, "%u", config.preset_1.am_color.w);
    char preset_1_pm_white_str[4] = "";
    sprintf(preset_1_pm_white_str, "%u", config.preset_1.pm_color.w);
    char preset_1_hr0_white_str[4] = "";
    sprintf(preset_1_hr0_white_str, "%u", config.preset_1.hr0_color.w);
    char preset_1_hr1_white_str[4] = "";
    sprintf(preset_1_hr1_white_str, "%u", config.preset_1.hr1_color.w);
    char preset_1_min0_white_str[4] = "";
    sprintf(preset_1_min0_white_str, "%u", config.preset_1.min0_color.w);
    char preset_1_min1_white_str[4] = "";
    sprintf(preset_1_min1_white_str, "%u", config.preset_1.min1_color.w);
    char preset_2_am_white_str[4] = "";
    sprintf(preset_2_am_white_str, "%u", config.preset_2.am_color.w);
    char preset_2_pm_white_str[4] = "";
    sprintf(preset_2_pm_white_str, "%u", config.preset_2.pm_color.w);
    char preset_2_hr0_white_str[4] = "";
    sprintf(preset_2_hr0_white_str, "%u", config.preset_2.hr0_color.w);
    char preset_2_hr1_white_str[4] = "";
    sprintf(preset_2_hr1_white_str, "%u", config.preset_2.hr1_color.w);
    char preset_2_min0_white_str[4] = "";
    sprintf(preset_2_min0_white_str, "%u", config.preset_2.min0_color.w);
    char preset_2_min1_white_str[4] = "";
    sprintf(preset_2_min1_white_str, "%u", config.preset_2.min1_color.w);
    char preset_3_am_white_str[4] = "";
    sprintf(preset_3_am_white_str, "%u", config.preset_3.am_color.w);
    char preset_3_pm_white_str[4] = "";
    sprintf(preset_3_pm_white_str, "%u", config.preset_3.pm_color.w);
    char preset_3_hr0_white_str[4] = "";
    sprintf(preset_3_hr0_white_str, "%u", config.preset_3.hr0_color.w);
    char preset_3_hr1_white_str[4] = "";
    sprintf(preset_3_hr1_white_str, "%u", config.preset_3.hr1_color.w);
    char preset_3_min0_white_str[4] = "";
    sprintf(preset_3_min0_white_str, "%u", config.preset_3.min0_color.w);
    char preset_3_min1_white_str[4] = "";
    sprintf(preset_3_min1_white_str, "%u", config.preset_3.min1_color.w);
    
    // setup replacement strings
    size_t max_size_replacement_string = 32; // not including the gmt_offset_str and active_preset_str
    const char *replacement_strings[] = {
        config.ntp_server_1,
        config.ntp_server_2,
        gmt_offset_str,
        active_preset_str,
        config.preset_1.name,
        preset_1_am_color_str,
        preset_1_am_white_str,
        preset_1_pm_color_str,
        preset_1_pm_white_str,
        preset_1_hr0_color_str,
        preset_1_hr0_white_str,
        preset_1_hr1_color_str,
        preset_1_hr1_white_str,
        preset_1_min0_color_str,
        preset_1_min0_white_str,
        preset_1_min1_color_str,
        preset_1_min1_white_str,
        config.preset_2.name,
        preset_2_am_color_str,
        preset_2_am_white_str,
        preset_2_pm_color_str,
        preset_2_pm_white_str,
        preset_2_hr0_color_str,
        preset_2_hr0_white_str,
        preset_2_hr1_color_str,
        preset_2_hr1_white_str,
        preset_2_min0_color_str,
        preset_2_min0_white_str,
        preset_2_min1_color_str,
        preset_2_min1_white_str,
        config.preset_3.name,
        preset_3_am_color_str,
        preset_3_am_white_str,
        preset_3_pm_color_str,
        preset_3_pm_white_str,
        preset_3_hr0_color_str,
        preset_3_hr0_white_str,
        preset_3_hr1_color_str,
        preset_3_hr1_white_str,
        preset_3_min0_color_str,
        preset_3_min0_white_str,
        preset_3_min1_color_str,
        preset_3_min1_white_str,
    };

    info_lwm("httpd", "allocating page");

    // copy the page to a new buffer in order to modify it
    const uint32_t page_size = root_template_end - root_template_start;
    ///ESP_LOGI(TAG, "page_size = %lu", page_size);
    const uint32_t size = page_size + (uint32_t)(num_match_strings 
        * (max_size_replacement_string - max_size_match_string) * 1.1 
        + sizeof(gmt_offset_str) + sizeof(active_preset_str)); // assume match strings are used 1.1 times on avg
    //ESP_LOGI(TAG, "size = %lu", size);

    char* page = (char*) malloc(size);
    assert(page && "Failed to allocate page");
    memset(page, 0, size);
    memcpy(page, root_template_start, page_size);

    // limit the chunk size to 64 bytes to reduce memory usage
    size_t max_chunk_size = 64;

    // replace substrings in the page
    replace_in_chunks(page, size, match_strings, replacement_strings, num_match_strings, max_chunk_size);
    // send the page
    httpd_resp_set_type(req, "text/html");
    int ret = httpd_resp_send(req, page, strlen(page));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send response");
        return ESP_FAIL;
    }
    info_lwm("httpd", "Served root");
    
    // free heap memory
    free(page);

    return ESP_OK;
}

/* HTTP POST Handler */
static esp_err_t wifi_post_handler(httpd_req_t *req) {

    ESP_LOGI(TAG, "Process wifi post");
    
    /* Destination buffer for content of HTTP POST request.
     * httpd_req_recv() accepts char* only, but content could
     * as well be any binary data (needs type casting).
     * In case of string data, null termination will be absent, and
     * content length would give length of string */
    size_t max_content_length = 70;
    char content[max_content_length];

    /* Truncate if content length is larger than the buffer */
    size_t recv_size = req->content_len;
    if (recv_size > max_content_length) {
        recv_size = max_content_length;
    }
    ESP_LOGI(TAG, "req->content_len = %d", req->content_len);
    ESP_LOGI(TAG, "recv_size = %d", recv_size);

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {  /* 0 return value indicates connection closed */
        /* Check if timeout occurred */
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            /* In case of timeout one can choose to retry calling
             * httpd_req_recv(), but to keep it simple, here we
             * respond with an HTTP 408 (Request Timeout) error */
            httpd_resp_send_408(req);
        }
        /* In case of error, returning ESP_FAIL will
         * ensure that the underlying socket is closed */
        return ESP_FAIL;
    }
    // make the last byte a null terminator
    content[recv_size] = '\0';
    ESP_LOGI(TAG, "Posted setup = %s", content);

    // extract wifi params and update config
    extract_wifi_params(content, app_config->wifi_ssid, app_config->wifi_password, 32);
    ret = save_config();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save config");
        return ESP_FAIL;
    }
    
    // Build the response message
    char response_message[100] = "Clock is restarting...";

    // copy the page to a new buffer in order to modify it
    const uint32_t page_size = response_template_end - response_template_start;
    //ESP_LOGI(TAG, "page_size = %lu", page_size);
    size_t size = page_size + strlen(response_message);
    char* page = (char*) malloc(size);
    assert(page && "Failed to allocate page");
    memset(page, 0, size);
    memcpy(page, response_template_start, page_size);

    // limit the chunk size to 64 bytes to reduce memory usage
    size_t max_chunk_size = 64;

    // replace substrings in the page
    const char *match_strings[] = {"{{response_message}}"};
    const char *replacement_strings[] = {response_message};
    size_t num_match_strings = 1;
    replace_in_chunks(page, size, match_strings, replacement_strings, num_match_strings, max_chunk_size);
    // send the page
    httpd_resp_set_type(req, "text/html");
    ret = httpd_resp_send(req, page, strlen(page));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send response");
    }
    // free heap memory
    free(page);

    vTaskDelay(100);
    esp_restart();
}

/* HTTP config POST Handler */
static esp_err_t config_post_handler(httpd_req_t *req) {

    bool restart = false;

    ESP_LOGI(TAG, "Process config post");

    /* Destination buffer for content of HTTP POST request.
     * httpd_req_recv() accepts char* only, but content could
     * as well be any binary data (needs type casting).
     * In case of string data, null termination will be absent, and
     * content length would give length of string 
     * The size of the buffer is the max allowed length of the post data.
     * Also, make sure CONFIG_HTTPD_MAX_REQ_HDR_LEN is large
     * enough to handle the post data.
     */
    size_t max_content_length = 1200;
    char content[max_content_length];

    /* Truncate if content length is larger than the buffer */
    size_t recv_size = req->content_len;
    if (recv_size > max_content_length) {
        recv_size = max_content_length;
    }
    ESP_LOGI(TAG, "req->content_len = %d", req->content_len);
    ESP_LOGI(TAG, "recv_size = %d", recv_size);

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {  /* 0 return value indicates connection closed */
        /* Check if timeout occurred */
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            /* In case of timeout one can choose to retry calling
             * httpd_req_recv(), but to keep it simple, here we
             * respond with an HTTP 408 (Request Timeout) error */
            httpd_resp_send_408(req);
        }
        /* In case of error, returning ESP_FAIL will
         * ensure that the underlying socket is closed */
        return ESP_FAIL;
    }
    content[recv_size] = '\0';
    ESP_LOGI(TAG, "Posted config: %s", content);

    // create a new config
    config_t *new_config = (config_t *)malloc(sizeof(config_t));
    assert(new_config && "Failed to allocate new config");
    memcpy(new_config, app_config, sizeof(config_t));

    // remember old gmt offset for comparison
    int old_gmt_offset = new_config->gmt_offset;

    // update app config from the posted params
    ret = set_config_from_params(content, new_config);
    if (ret == FORM_VAL_STATUS_RESTART) {
        restart = true;
    }

    // save the new config
    if (ret == FORM_VAL_STATUS_OK || ret == FORM_VAL_STATUS_RESTART) {
        memcpy(app_config, new_config, sizeof(config_t));
        ret = save_config();
    }

    // Set the timezone if the gmt offset has changed
    if (old_gmt_offset != app_config->gmt_offset) {
        set_timezone(app_config->gmt_offset);
    }

    // Set the response message
    char response_message[100] = "";
    switch (ret) {
        case FORM_VAL_STATUS_ERROR:
            strcat(response_message, "Unknown error.<br/>Config not updated.");
            break;
        case FORM_VAL_STATUS_AUTH_INVALID:
            strcat(response_message, "Invalid password.<br/>Config not updated.");
            break;
        case FORM_VAL_STATUS_FIELD_INVALID:
            strcat(response_message, "Invalid field.<br/>Config not updated.");
            break;
        case FORM_VAL_STATUS_RESTART:
            strcat(response_message, "Config updated.<br/>Restarting...");
            break;
        default:
            strcat(response_message, "Config updated.");
    }

    // copy the page to a new buffer in order to modify it
    const uint32_t page_size = response_template_end - response_template_start;
    //ESP_LOGI(TAG, "page_size = %lu", page_size);
    size_t size = page_size + strlen(response_message);
    char* page = (char*) malloc(size);
    assert(page && "Failed to allocate page");
    memset(page, 0, size);
    memcpy(page, response_template_start, page_size);

    // limit the chunk size to 64 bytes to reduce memory usage
    size_t max_chunk_size = 64;

    // Fill the template with the response message
    const char *match_strings[] = {"{{response_message}}"};
    const char *replacement_strings[] = {response_message};
    size_t num_match_strings = 1;
    replace_in_chunks(page, size, match_strings, replacement_strings, num_match_strings, max_chunk_size);

    // send the response page
    httpd_resp_set_type(req, "text/html");
    ret = httpd_resp_send(req, page, strlen(page));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send response");
    }
    // free heap memory
    free(page);
    
    // restart if needed
    if (restart) {
        esp_restart();
    }
    return ret;
}

/* HTTP CSS GET Handler */
static esp_err_t css_get_handler(httpd_req_t *req) {

    ESP_LOGI(TAG, "Process css get");
    
    // copy the page to a new buffer in order to modify it
    const uint32_t page_size = style_template_end - style_template_start;
    //ESP_LOGI(TAG, "page_size = %lu", page_size);
    char* page = (char*) malloc(page_size);
    assert(page && "Failed to allocate page");
    memset(page, 0, page_size);
    memcpy(page, style_template_start, page_size);

    // send the page
    httpd_resp_set_type(req, "text/css");
    int ret = httpd_resp_send(req, page, strlen(page));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send response");
        return ESP_FAIL;
    }
    info_lwm("httpd", "Served css");
    
    // free heap memory
    free(page);

    return ESP_OK;
}

/* HTTP Error (404) Handler */
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err) {
    // Set status
    httpd_resp_set_status(req, "404 Not Found");

    // Build the response message
    char response_message[100] = "Page not found";

    // copy the page to a new buffer in order to modify it
    const uint32_t page_size = response_template_end - response_template_start;
    //ESP_LOGI(TAG, "page_size = %lu", page_size);
    size_t size = page_size + strlen(response_message);
    char* page = (char*) malloc(size);
    assert(page && "Failed to allocate page");
    memset(page, 0, page_size);
    memcpy(page, response_template_start, page_size);

    // limit the chunk size to 64 bytes to reduce memory usage
    size_t max_chunk_size = 64;

    // replace substrings in the page
    const char *match_strings[] = {"{{response_message}}"};
    const char *replacement_strings[] = {response_message};
    size_t num_match_strings = 1;
    replace_in_chunks(page, size, match_strings, replacement_strings, num_match_strings, max_chunk_size);
    // send the page
    httpd_resp_set_type(req, "text/html");
    int ret = httpd_resp_send(req, page, strlen(page));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send response");
    }
    return ESP_OK;
}

/* HTTP Captive Portal (404) Handler - Redirects all requests to the root page */
esp_err_t http_404_captiveportal_handler(httpd_req_t *req, httpd_err_code_t err) {
    // Set status
    httpd_resp_set_status(req, "302 Temporary Redirect");
    // Redirect to the "/" root directory
    httpd_resp_set_hdr(req, "Location", "/");
    // iOS requires content in the response to detect a captive portal, simply redirecting is not sufficient.
    httpd_resp_send(req, "Redirect to the captive portal", HTTPD_RESP_USE_STRLEN);

    ESP_LOGI(TAG, "Redirecting to root");
    return ESP_OK;
}

/* Routes handled by the http server, enumerated with route_t types */
static const httpd_uri_t routes[] = { 
    {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler
    },
    {
        .uri = "/",
        .method = HTTP_GET,
        .handler = setup_root_get_handler
    },
    {
        .uri = "/",
        .method = HTTP_POST,
        .handler = config_post_handler
    },
    {
        .uri = "/",
        .method = HTTP_POST,
        .handler = wifi_post_handler
    },
    {
        .uri = "/css",
        .method = HTTP_GET,
        .handler = css_get_handler
    }
};

static httpd_handle_t start_webserver(bool captive_portal)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_open_sockets = 7;
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        if (captive_portal) {
            ESP_LOGI(TAG, "Registering URI handlers for captive portal");
            httpd_register_uri_handler(server, &routes[ROUTE_SETUP_ROOT]);
            httpd_register_uri_handler(server, &routes[ROUTE_WIFI]);
            httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_captiveportal_handler);
        } else {
            ESP_LOGI(TAG, "Registering URI handlers for clock");
            httpd_register_uri_handler(server, &routes[ROUTE_ROOT]);
            httpd_register_uri_handler(server, &routes[ROUTE_CONFIG]);
            httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);
        }
        httpd_register_uri_handler(server, &routes[ROUTE_CSS]);
    }
    return server;
}

static void start_captiveportal(void) {
    ESP_LOGI(TAG, "Starting captive portal");

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    /* Initialize AP */
    ESP_LOGI(TAG_AP, "ESP_WIFI_MODE_AP");
    esp_netif_t *esp_netif_ap = wifi_init_softap();

    /* Set sta as the default interface */
    esp_netif_set_default_netif(esp_netif_ap);
    
    // Configure DNS-based captive portal
    dhcp_set_captiveportal_url();

    // Start the server for the first time
    start_webserver(true);

    // Start the DNS server that will redirect all queries to the softAP IP
    dns_server_config_t config = DNS_SERVER_CONFIG_SINGLE("*" /* all A queries */, "WIFI_AP_DEF" /* softAP netif ID */);
    start_dns_server(&config);
}

led_strip_handle_t configure_led(void)
{
    // Set format for the led strip
    struct format_layout color_format = {
        .g_pos = 0, // green is the first byte in the color data
        .r_pos = 1, // red is the second byte in the color data
        .b_pos = 2, // blue is the third byte in the color data
        .num_components = 3, // total 3 color components
    };
    // Set change format for SK6812
    if (led_type == LED_MODEL_SK6812) {
        color_format.w_pos = 3;
        color_format.num_components = 4;
    }

    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN, // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_COUNT,      // The number of LEDs in the strip,
        .led_model = led_type,                // LED type: WS2812 or SK6812
        // set the color order of the strip
        .color_component_format = {
            .format = color_format,
        },
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    // LED strip backend configuration: SPI
    led_strip_spi_config_t spi_config = {
        .clk_src = SPI_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
        .spi_bus = SPI2_HOST,           // SPI bus ID
        .flags = {
            .with_dma = true, // Using DMA can improve performance and help drive more LEDs
        }
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with SPI backend");
    return led_strip;
}

void print_display_bits(uint16_t time_bits) {
    char bit_str[17];  // 16 bits + null terminator
    bit_str[16] = '\0';
    
    for (int i = 0; i < 16; i++) {
        bit_str[15 - i] = (time_bits & (1 << i)) ? '1' : '0';
    }
    
    // Format: [5 unused][1 AM/PM][4 Hours][6 Minutes]
    ESP_LOGI(TAG, "Display: |%c|%.*s|%.*s|", 
           bit_str[5],                   // AM/PM bit
           4, &bit_str[6],               // hour bits
           6, &bit_str[10]);             // minute bits
}

/* Display time (normal mode) or end of IP address (identify mode) */
void display_time_task(void *pvParameters) {

    // Get config from task parameters, setup led strip
    //config_t *config = (config_t *)pvParameters;
    
    time_t now = 0;
    struct tm timeinfo = { 0 };

    while (true) {

        // Check if we are in identify mode
        EventBits_t bits = xEventGroupWaitBits(s_app_event_group,
                                           APP_MODE_IDENTIFY,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(500));
        if (bits & APP_MODE_IDENTIFY) {

            // We are in identify mode, show the end of the IP address
            uint16_t ip_bits = 0;
            uint8_t last_quad = (uint8_t)((device_ip.addr >> 24) & 0xFF);
            //ESP_LOGI(TAG, "IP last quad: %d", last_quad);
            ip_bits |= (last_quad >> 4) << 6; // most significant nibble
            ip_bits |= (last_quad & 0x0F) << 1; // least significant nibble
            //print_display_bits(ip_bits);

            // Setup mac address bits array for led strip api
            bool ip_bits_array[11];
            size_t count = 0;
            for (int i = 10; i >= 0; i--) {
                ip_bits_array[count] = (ip_bits & (1 << i)) ? true : false;
                count++;
            }

            // Set pixel color for each led
            for (int i = 0; i < 11; i++) {
                uint8_t r = 0;
                uint8_t g = 0;
                uint8_t b = 0;
                uint8_t w = 0;
                // last quad of the mac address will be displayed
                // on the 8 pixels in the middle of the pyramid
                if (i != 0 && i != 5 && i != 10) {
                    if (led_type == LED_MODEL_SK6812) {
                        g = ip_bits_array[i] ? 50 : 0;
                        w = ip_bits_array[i] ? 0 : 3;
                    } else {
                        g = ip_bits_array[i] ? 50 : 1;
                    }
                }
                // If SK6812, set pixel with RGBW, otherwise set pixel with RGB
                if (led_type == LED_MODEL_SK6812) {
                    ESP_ERROR_CHECK(led_strip_set_pixel_rgbw(*led_strip, i, r, g, b, w));
                } else {
                    ESP_ERROR_CHECK(led_strip_set_pixel(*led_strip, i, r, g, b));
                }
            }
            /* Refresh the strip to send data */
            ESP_ERROR_CHECK(led_strip_refresh(*led_strip));
        } else {

            // We are in normal mode, show the time
            time(&now);
            localtime_r(&now, &timeinfo);
            //ESP_LOGI(TAG, "Time: %d:%d", timeinfo.tm_hour, timeinfo.tm_min);
            
            // Setup time bits array for led display
            uint16_t time_bits = 0;
            time_bits |= (timeinfo.tm_hour > 11) << 10; // AM/PM (1 bit)
            uint8_t hour = timeinfo.tm_hour==12 ? 12 : timeinfo.tm_hour % 12;
            time_bits |= (hour) << 6;        // hours (in 4 bits)
            time_bits |= (timeinfo.tm_min);  // minutes (in 6 bits)
            //print_display_bits(time_bits);

            // Setup time bits array for led strip api
            bool time_bits_array[11];
            size_t count = 0;
            for (int i = 10; i >= 0; i--) {
                time_bits_array[count] = (time_bits & (1 << i)) ? true : false;
                count++;
            }

            config_t *config = app_config;
            //ESP_LOGI(TAG, "Active preset: %d", config->active_preset);

            preset_t *preset = &config->preset_1;
            if (config->active_preset == 2) {
                preset = &config->preset_2;
            } else if (config->active_preset == 3) {
                preset = &config->preset_3;
            }

            // Set pixel color for each led
            for (int i = 0; i < 11; i++) {
                uint8_t r = 0;
                uint8_t g = 0;
                uint8_t b = 0;
                uint8_t w = 0;
                // AM/PM LED
                if (i == 0) {
                    r = time_bits_array[i] ? preset->pm_color.r : preset->am_color.r;
                    g = time_bits_array[i] ? preset->pm_color.g : preset->am_color.g;
                    b = time_bits_array[i] ? preset->pm_color.b : preset->am_color.b;
                    w = time_bits_array[i] ? preset->pm_color.w : preset->am_color.w;
                // Hour LEDs
                } else if (i > 0 && i < 5) {
                    r = time_bits_array[i] ? preset->hr1_color.r : preset->hr0_color.r;
                    g = time_bits_array[i] ? preset->hr1_color.g : preset->hr0_color.g;
                    b = time_bits_array[i] ? preset->hr1_color.b : preset->hr0_color.b;
                    w = time_bits_array[i] ? preset->hr1_color.w : preset->hr0_color.w;
                // Minute LEDs
                } else if (i > 4 && i < 11) {
                    r = time_bits_array[i] ? preset->min1_color.r : preset->min0_color.r;
                    g = time_bits_array[i] ? preset->min1_color.g : preset->min0_color.g;
                    b = time_bits_array[i] ? preset->min1_color.b : preset->min0_color.b;
                    w = time_bits_array[i] ? preset->min1_color.w : preset->min0_color.w;
                }
                // If SK6812, set pixel with RGBW, otherwise set pixel with RGB
                if (led_type == LED_MODEL_SK6812) {
                    ESP_ERROR_CHECK(led_strip_set_pixel_rgbw(*led_strip, i, r, g, b, w));
                } else {
                    ESP_ERROR_CHECK(led_strip_set_pixel(*led_strip, i, r, g, b));
                }
            }
            /* Refresh the strip to send data */
            ESP_ERROR_CHECK(led_strip_refresh(*led_strip));
        }
        //vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

static void start_clock(void) {

    ESP_LOGI(TAG, "Starting clock");
    
#if LWIP_DHCP_GET_NTP_SRV
    /**
     * NTP server address could be acquired via DHCP,
     * see following menuconfig options:
     * 'LWIP_DHCP_GET_NTP_SRV' - enable STNP over DHCP
     * 'LWIP_SNTP_DEBUG' - enable debugging messages
     *
     * NOTE: This call should be made BEFORE esp acquires IP address from DHCP,
     * otherwise NTP option would be rejected by default.
     */
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_SNTP_TIME_SERVER);
    config.start = false;                       // start SNTP service explicitly (after connecting)
    config.server_from_dhcp = true;             // accept NTP offers from DHCP server, if any (need to enable *before* connecting)
    config.renew_servers_after_new_IP = true;   // let esp-netif update configured SNTP server(s) after receiving DHCP lease
    config.index_of_first_server = 1;           // updates from server num 1, leaving server 0 (from DHCP) intact
    // configure the event on which we renew servers
    config.ip_event_to_renew = IP_EVENT_STA_GOT_IP;
#endif

// if we have more than one NTP server then set config to use multiple servers
#if CONFIG_LWIP_SNTP_MAX_SERVERS > 1
    esp_sntp_config_t ntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(2,
                               ESP_SNTP_SERVER_LIST(app_config->ntp_server_1, app_config->ntp_server_2) );
#else
    esp_sntp_config_t ntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG(app_config->ntp_server_1);
#endif
    ntp_config.sync_cb = time_sync_notification_cb;
    esp_netif_sntp_init(&ntp_config);

#if LWIP_DHCP_GET_NTP_SRV
    // This is needed in the case of getting NTP server address via DHCP
    esp_netif_sntp_start();
#endif

    // wait for time to be set
    int retry = 0;
    const int retry_count = 15;
    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    }

    // Copy config to a copy to avoid other tasks from modifying the config
    config_t *config_copy = malloc(sizeof(config_t));
    memset(config_copy, 0, sizeof(config_t));
    memcpy(config_copy, app_config, sizeof(config_t));

    app_mode = APP_MODE_NORMAL;

    xTaskCreate(&display_time_task, "display_time", 2048, config_copy, 5, NULL);
}

/* play the startup animation */
void startup_animation(void *pvParameters) {
    ESP_LOGI(TAG, "Playing startup animation");

    const uint8_t NUM_LEDS = 11;      /* The amount of pixels/leds you have */
    const uint8_t BRIGHTNESS = 20;   /* Control the brightness of your leds */
    const uint8_t SATURATION = 255;   /* Control the saturation of your leds */

    while(app_mode == APP_MODE_STARTUP) {
        for (int j = 0; j < 361; j++) {
            for (int i = 0; i < NUM_LEDS; i++) {
                uint16_t hue = j+i*32 < 360 ? j+i*32 : j+i*32-360;
                if (i == 5) {
                    //ESP_LOGI(TAG, "%d + %d * 32 = %d", j, i, hue);
                }
                //hue = 0;
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(*led_strip, i, hue, SATURATION, BRIGHTNESS));
            }
            ESP_ERROR_CHECK(led_strip_refresh(*led_strip));
            vTaskDelay(pdMS_TO_TICKS(5)); /* Change this to your hearts desire, the lower the value the faster your colors move (and vice versa) */
        }
    }
    vTaskDelete(NULL);
}

/* flash lights when in AP captive portal */

void flash_lights(void *pvParameters) {
    ESP_LOGI(TAG, "Flashing lights");
    bool led_on_off = true;
    while (1) {
        if (led_on_off) {
            // Set pixel color for each led
            for (int i = 0; i < 11; i++) {
                if (led_type == LED_MODEL_SK6812) {
                    ESP_ERROR_CHECK(led_strip_set_pixel_rgbw(*led_strip, i, 0, 0, 0, 50));
                } else {
                    ESP_ERROR_CHECK(led_strip_set_pixel(*led_strip, i, 50, 50, 50));
                }
            }
            ESP_ERROR_CHECK(led_strip_refresh(*led_strip));
        } else {
            ESP_ERROR_CHECK(led_strip_clear(*led_strip));
        }
        led_on_off = !led_on_off;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = EXAMPLE_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 620, // minimum for ESP32C3 is 611
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

void app_main(void)
{   
    // Setup led strip
    led_strip = calloc(1, sizeof(led_strip_handle_t));
    *led_strip = configure_led();

    // Startup animation
    xTaskCreate(&startup_animation, "startup_animation", 2048, NULL, 5, NULL);

    // Allocate memory for app config
    app_config = calloc(1, sizeof(config_t));

    /*
        Turn of warnings from HTTP server as redirecting traffic will yield
        lots of invalid requests
    */
    esp_log_level_set("httpd_uri", ESP_LOG_ERROR);
    esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);
    esp_log_level_set("httpd_parse", ESP_LOG_ERROR);

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Set config defaults
    memcpy(app_config->clock_password, CONFIG_ELEVEN_BIT_CLOCK_DEFAULT_PASSWORD, strlen(CONFIG_ELEVEN_BIT_CLOCK_DEFAULT_PASSWORD));
    memcpy(app_config->wifi_ssid, CONFIG_ESP_WIFI_REMOTE_AP_SSID, strlen(CONFIG_ESP_WIFI_REMOTE_AP_SSID));
    memcpy(app_config->wifi_password, CONFIG_ESP_WIFI_REMOTE_AP_PASSWORD, strlen(CONFIG_ESP_WIFI_REMOTE_AP_PASSWORD));
    memcpy(app_config->ntp_server_1, CONFIG_ELEVEN_BIT_CLOCK_DEFAULT_NTP_SERVER_1, strlen(CONFIG_ELEVEN_BIT_CLOCK_DEFAULT_NTP_SERVER_1));
    memcpy(app_config->ntp_server_2, CONFIG_ELEVEN_BIT_CLOCK_DEFAULT_NTP_SERVER_2, strlen(CONFIG_ELEVEN_BIT_CLOCK_DEFAULT_NTP_SERVER_2));
    app_config->gmt_offset = CONFIG_ELEVEN_BIT_CLOCK_DEFAULT_GMT_OFFSET;

    // load config
    ret = load_config();
    if (ret != ESP_OK) {
        printf("Failed to read config from NVS. Using defaults.\n");
    }

    // Set the timezone
    set_timezone(app_config->gmt_offset);

    ESP_LOGI(TAG_STA, "Config ssid: %s", app_config->wifi_ssid);

    /* Initialize event groups */
    s_wifi_event_group = xEventGroupCreate();
    s_app_event_group = xEventGroupCreate();

    /* Register Event handler */
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

    /* Initialize WiFi */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Initialize STA */
    esp_netif_t *esp_netif_sta = NULL;
    if (strcmp(app_config->wifi_ssid, "") != 0) { 
        // If we have WiFi credentials, try to connect to the AP
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_LOGI(TAG_STA, "ESP_WIFI_MODE_STA");
        esp_netif_sta = wifi_init_sta();
    } else {
        // If we don't have WiFi credentials, start the captive portal
        ESP_LOGI(TAG_STA, "No WiFi credentials found, starting captive portal");
        app_mode = APP_MODE_SETUP;
        xTaskCreate(&flash_lights, "flash_lights", 2048, NULL, 5, NULL);
        start_captiveportal();
    }

    /* Start WiFi */
    ESP_ERROR_CHECK(esp_wifi_start());

    /*
     * Wait until either the connection is established (WIFI_CONNECTED_BIT) or
     * connection failed for the maximum number of re-tries (WIFI_FAIL_BIT).
     * The bits are set by event_handler() (see above)
     */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned,
     * hence we can test which event actually happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_STA, "connected to ap SSID:%s password:%s",
                 app_config->wifi_ssid, app_config->wifi_password);
        
        start_clock();
        /* Set sta as the default interface */
        esp_netif_set_default_netif(esp_netif_sta);
        start_webserver(false); // config web interface
        ESP_LOGI(TAG, "Started all services");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG_STA, "Failed to connect to SSID:%s, password:%s",
                 app_config->wifi_ssid, app_config->wifi_password);
        app_mode = APP_MODE_SETUP;
        xTaskCreate(&flash_lights, "flash_lights", 2048, NULL, 5, NULL);
        start_captiveportal();
    } else {
        ESP_LOGE(TAG_STA, "UNEXPECTED EVENT, terminating.");
        return;
    }

    // If we are in setup mode, don't do anything more
    if (app_mode == APP_MODE_SETUP) {
        return;
    }

    // ADC setup and loop for touch detection

    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    int identify_count = 0;     // count of activations of identify mode
    int touch_count = 0;        // count of readings while touching
    int avg_data_prev = 0;      // previous average of the touch readings
    int avg_data_prev_comp = 0; // previous average of the comparator readings
    int delta_prev = 0;         // previous delta
    bool valley_prev = false;   // true when the previous reading dipped an amount >= 80% of the threshold
    bool first_read = true;     // true when the first reading is taken
    
    // ADC loop to try reading data again after timeout (no data available)
    while (1) {
        /**
         * This is to show you the way to use the ADC continuous mode driver event callback.
         * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
         * However in this example, the data processing (print) is slow, so you barely block here.
         *
         * Without using this event callback (to notify this task), you can still just call
         * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        char unit[] = EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT);

        // ADC loop to read data until timeout (no data available)
        while (1) {
            if (app_mode == APP_MODE_NORMAL) {
                //ESP_LOGI(TAG, "App mode: %d", app_mode);
                ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
                if (ret == ESP_ERR_TIMEOUT) {
                    //ESP_LOGI(TAG, "Timeout reading ADC");
                    //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                    break;
                }
                // The code below is not my proudest moment, but it works
                // Lots of shenanigans to improve the touch detection robustness,
                // but still gets occassional false positives due to wild swings in the ADC readings
                if (ret == ESP_OK) {
                    //ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
                    int avg_data = 0;        // average of the touch readings
                    int avg_data_comp = 0;   // average of the comparator readings
                    int data_count = 0;      // count of readings for the touch
                    int data_count_comp = 0; // count of readings for the comparator
                    int delta = 0;           // difference between current and previous reading
                    
                    // calculate the average of the readings for the touch and comparator
                    for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                        adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                        uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
                        uint32_t data = EXAMPLE_ADC_GET_DATA(p);
                        /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
                        if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
                            // handle cases here for each channel if you have more than one channel
                            if (chan_num == TOUCH_ADC_CHANNEL) {
                                //ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIu32, unit, chan_num, data);
                                avg_data += data;
                                data_count++;
                            }
                            if (chan_num == TOUCH_COMP_CHANNEL) {
                                //ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIu32, unit, chan_num, data);
                                avg_data_comp += data;
                                data_count_comp++;
                            }
                        } else {
                            ESP_LOGW(TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
                        }
                    }
                    avg_data /= data_count;
                    avg_data_comp /= data_count_comp;
                    avg_data = avg_data - avg_data_comp; // diff between touch and comparator to reduce noise
                    if (avg_data < 0) {
                        avg_data += -2 * avg_data; // make sure avg_data is positive
                    }
                    delta = avg_data - avg_data_prev;
                    if (delta > TOUCH_DELTA_THRESHOLD * 0.6) {
                        ESP_LOGI(TAG, "Delta: %d, Prev: %d", delta, delta_prev);
                    }
                    if (delta_prev < TOUCH_DELTA_THRESHOLD * -0.8) {
                        valley_prev = true;
                        ESP_LOGI(TAG, "Prev was valley: %d", delta_prev);
                    }
                    
                    // check if touch has initiated
                    if (touch_count == 0) { 
                        if (delta > TOUCH_DELTA_THRESHOLD) { // sufficient change on one reading
                            touch_count++;
                            if (first_read) { // if first reading, then ignore this reading
                                touch_count = 0;
                            } else if (valley_prev) { // if previous delta was a big drop, then ignore this reading
                                touch_count = 0;
                                valley_prev = false; // no need to check for valley again
                            }
                            ESP_LOGI(TAG, "Initiated touch, count: %d, first: %d, valley: %d, delta: %d, prev: %d", touch_count, first_read, valley_prev, delta, delta_prev);
                        } else if ((delta + delta_prev) > TOUCH_DELTA_THRESHOLD) { // sufficient change over two readings
                            touch_count++;
                            if (first_read && delta_prev != 0) { // if second reading, ignore as well
                                touch_count = 0;
                                first_read = false; // no need to check for first read again
                            } else if (valley_prev && delta_prev >= TOUCH_DELTA_THRESHOLD * -0.8) { // there was a valley before delta_prev
                                touch_count = 0;
                                valley_prev = false; // no need to check for valley again
                            }
                            ESP_LOGI(TAG, "Initiated touch, count: %d, first: %d, valley: %d, delta: %d, prev: %d", touch_count, first_read, valley_prev, delta, delta_prev);
                        } else if (valley_prev && delta < TOUCH_DELTA_THRESHOLD * 0.5) {
                            valley_prev = false; // no need to check for valley again
                        }
                    }
                    // check if touch is sustained
                    else if (touch_count > 0 && touch_count < TOUCH_COUNT_THRESHOLD) { 
                        // increase touch count if sustained within delta threshold range
                        if (delta > TOUCH_JITTER_THRESHOLD - TOUCH_DELTA_THRESHOLD) {
                            touch_count++;
                            ESP_LOGI(TAG, "Sustained touch, count: %d (delta: %d)", touch_count, delta);
                            //ESP_LOGI(TAG, "Touch jitter, count: %d", touch_count);
                        }
                        // reset touch count if not sustained within jitter threshold
                        else {
                            touch_count = 0;
                            ESP_LOGI(TAG, "Too much jitter, touch count reset (delta: %d)", delta);
                        }
                    }
                    // if the touch count is at or above the threshold, set the app mode to identify
                    else if (touch_count >= TOUCH_COUNT_THRESHOLD) {
                        app_mode = APP_MODE_IDENTIFY;
                        xEventGroupSetBits(s_app_event_group, APP_MODE_IDENTIFY);
                        identify_count++;
                        ESP_LOGI(TAG, "Identify mode activated, count: %d", identify_count);
                        touch_count = 0;
                    }
                    // update previous data
                    avg_data_prev = avg_data; // update previous data
                    delta_prev = delta;
                    /**
                     * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
                     * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
                     * usually you don't need this delay (as this task will block for a while).
                     */
                    vTaskDelay(pdMS_TO_TICKS(10)); // avoid watchdog timeout
                }
            } else if (app_mode == APP_MODE_IDENTIFY) {
                vTaskDelay(pdMS_TO_TICKS(IDENTIFY_TIMEOUT)); // keep the identify mode for 10 seconds
                app_mode = APP_MODE_NORMAL;
                xEventGroupClearBits(s_app_event_group, APP_MODE_IDENTIFY);
            }
            else {
                break;
            }
        }
    }
    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}
