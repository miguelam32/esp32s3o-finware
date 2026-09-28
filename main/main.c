/*
 * ESP32-S2 mini: mouse USB HID controlado por WiFi (UDP).
 *
 * Version con soporte de "drag" (boton sostenido) ademas de:
 *   M <dx> <dy>   -> mover el cursor en relativo (como un trackpad)
 *   CL            -> clic izquierdo (press + release)
 *   CR            -> clic derecho (press + release)
 *   S <pasos>     -> girar la rueda de scroll
 *   DN            -> mantener presionado el boton izquierdo (drag start)
 *   UP            -> soltar el boton izquierdo (drag end)
 *
 * Mientras el boton esta "abajo" (DN), los reportes de movimiento (M)
 * viajan con ese boton incluido en la mascara, tal como si mantuvieras
 * fisicamente presionado el click mientras mueves el mouse. Asi se
 * puede arrastrar iconos, hacer swipe para desbloquear pantalla, etc.
 *
 * IMPORTANTE - datos de tu red:
 *   SSID / password puestos abajo en WIFI_SSID / WIFI_PASS.
 *
 * En el CMakeLists.txt de este componente ("main"), REQUIRES debe
 * incluir al menos:
 *   REQUIRES esp_wifi esp_netif esp_event nvs_flash lwip tinyusb driver
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"

static const char *TAG = "usb_hid_wifi_mouse";

/* ----------------- datos de tu red (hotspot del celular) ------------- */
#define WIFI_SSID   "BMW"
#define WIFI_PASS   "789012345"
#define UDP_PORT    4242

/* ----------------- Descriptores TinyUSB (mouse HID) -------------------- */

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

static const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_MOUSE()
};

static const char *hid_string_descriptor[5] = {
    (char[]){0x09, 0x04},           // 0: idioma (ingles US)
    "ESP32",                         // 1: fabricante
    "ESP32-S2 Mouse WiFi",           // 2: producto
    "123456",                        // 3: serial
    "HID Interface",                 // 4: nombre del descriptor HID
};

static const uint8_t hid_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN,
                           TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void) instance;
    return hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                                uint8_t *buffer, uint16_t reqlen)
{
    (void) instance; (void) report_id; (void) report_type; (void) buffer; (void) reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                            uint8_t const *buffer, uint16_t bufsize)
{
    (void) instance; (void) report_id; (void) report_type; (void) buffer; (void) bufsize;
}

/* ----------------------------- WiFi STA ------------------------------- */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi desconectado, reintentando conexion...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "IP obtenida: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "(anota esta IP si el broadcast desde el celular no llega y necesitas fijarla en el script)");
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* ------------------------- Helpers mouse HID --------------------------- */

/* Estado persistente del boton. Se usa para drag: mientras esta
   "abajo" (DN), todos los reportes de movimiento salen con este
   boton incluido en la mascara, como si lo tuvieras fisicamente
   presionado mientras mueves el mouse. */
static uint8_t s_button_mask = 0x00;

static void hid_click(uint8_t button_mask)
{
    if (!tud_hid_ready()) {
        return;
    }
    // clic rapido: no toca el estado persistente de drag, manda su
    // propio press+release combinado con lo que ya estuviera sostenido
    tud_hid_mouse_report(0, s_button_mask | button_mask, 0, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    tud_hid_mouse_report(0, s_button_mask, 0, 0, 0, 0);
}

static void hid_button_down(uint8_t button_mask)
{
    s_button_mask |= button_mask;
    if (!tud_hid_ready()) {
        return;
    }
    tud_hid_mouse_report(0, s_button_mask, 0, 0, 0, 0);
}

static void hid_button_up(uint8_t button_mask)
{
    s_button_mask &= (uint8_t)~button_mask;
    if (!tud_hid_ready()) {
        return;
    }
    tud_hid_mouse_report(0, s_button_mask, 0, 0, 0, 0);
}

static void hid_scroll(int steps)
{
    if (!tud_hid_ready()) {
        return;
    }
    int8_t wheel = (int8_t)(steps > 127 ? 127 : (steps < -127 ? -127 : steps));
    tud_hid_mouse_report(0, s_button_mask, 0, 0, wheel, 0);
}

static void hid_move(int dx, int dy)
{
    if (!tud_hid_ready()) {
        return;
    }
    // Los reportes HID de mouse usan deltas de 1 byte (-127..127). Si
    // llega un delta mas grande (por ejemplo por un swipe rapido),
    // lo partimos en varios reportes en vez de recortarlo.
    while (dx != 0 || dy != 0) {
        int8_t step_x = (int8_t)(dx > 127 ? 127 : (dx < -127 ? -127 : dx));
        int8_t step_y = (int8_t)(dy > 127 ? 127 : (dy < -127 ? -127 : dy));
        // usamos s_button_mask en vez de 0x00: asi si hay un drag
        // activo (DN mandado antes), el boton se mantiene presionado
        // durante todo el movimiento
        tud_hid_mouse_report(0, s_button_mask, step_x, step_y, 0, 0);
        dx -= step_x;
        dy -= step_y;
        if (dx != 0 || dy != 0) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

/* --------------------- Tarea UDP: recibe comandos ----------------------- */

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];

    struct sockaddr_in bind_addr;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(UDP_PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "No se pudo crear el socket UDP: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "No se pudo bindear el socket UDP: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Socket UDP escuchando en puerto %d", UDP_PORT);

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                            (struct sockaddr *)&source_addr, &socklen);
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom fallo: errno %d", errno);
            continue;
        }
        rx_buffer[len] = '\0';

        int dx = 0, dy = 0, steps = 0;
        if (rx_buffer[0] == 'M') {
            if (sscanf(rx_buffer, "M %d %d", &dx, &dy) == 2) {
                hid_move(dx, dy);
            }
        } else if (rx_buffer[0] == 'S') {
            if (sscanf(rx_buffer, "S %d", &steps) == 1) {
                hid_scroll(steps);
            }
        } else if (strncmp(rx_buffer, "CL", 2) == 0) {
            hid_click(0x01); // boton izquierdo (clic rapido)
        } else if (strncmp(rx_buffer, "CR", 2) == 0) {
            hid_click(0x02); // boton derecho (clic rapido)
        } else if (strncmp(rx_buffer, "DN", 2) == 0) {
            hid_button_down(0x01); // mantener boton izquierdo (drag start)
        } else if (strncmp(rx_buffer, "UP", 2) == 0) {
            hid_button_up(0x01);   // soltar boton izquierdo (drag end)
        }
    }
}

/* --------------------------------- main --------------------------------- */

#include <math.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

#define LED_GPIO   48
#define LED_BRIGHT 50   // 0-255

static void rainbow_task(void *arg) {
    led_strip_handle_t strip;
    led_strip_config_t cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&cfg, &rmt, &strip));

    float hue = 0;
    while (1) {
        float x = hue / 60.0f, f = x - floorf(x), q = 1 - f;
        float r, g, b;
        switch ((int)x % 6) {
            case 0: r = 1; g = f; b = 0; break;
            case 1: r = q; g = 1; b = 0; break;
            case 2: r = 0; g = 1; b = f; break;
            case 3: r = 0; g = q; b = 1; break;
            case 4: r = f; g = 0; b = 1; break;
            default: r = 1; g = 0; b = q; break;
        }
        led_strip_set_pixel(strip, 0,
            (uint8_t)(powf(r, 2.2f) * LED_BRIGHT + 0.5f),
            (uint8_t)(powf(g, 2.2f) * LED_BRIGHT + 0.5f),
            (uint8_t)(powf(b, 2.2f) * LED_BRIGHT + 0.5f));
        led_strip_refresh(strip);
        hue += 0.5f;
        if (hue >= 360) hue -= 360;
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

void app_main(void)
{
    xTaskCreate(rainbow_task, "rainbow", 4096, NULL, 1, NULL);
    ESP_LOGI(TAG, "Inicializando NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Inicializando USB HID mouse...");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = hid_configuration_descriptor,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    ESP_LOGI(TAG, "Conectando WiFi a SSID '%s'...", WIFI_SSID);
    wifi_init_sta();

    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Listo. Conecta el S2 mini por USB a la tablet.");
    ESP_LOGI(TAG, "Esperando comandos por WiFi UDP en el puerto %d.", UDP_PORT);
    ESP_LOGI(TAG, "Comandos: M dx dy | CL | CR | S pasos | DN (drag start) | UP (drag end)");
}

