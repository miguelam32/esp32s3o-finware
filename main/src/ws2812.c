/*
 * ws2812.c - WS2812 sobre componente led_strip (RMT).
 *
 * Notas que evitan los clasicos:
 *   - El WS2812 es GRB, NO RGB. Esta fijado en el config.
 *   - El arcoiris se calcula en HSV: el brillo se mantiene constante.
 *     Interpolar RGB directo pasa por grises apagados y se ve feo.
 *   - Resolucion RMT de 10 MHz (la que pide el WS2812 a 800 kHz),
 *     no 40 MHz.
 *   - Tarea propia con prioridad baja: no compite con el jammer.
 */
#include "ws2812.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "ws2812";

#ifndef CONFIG_WS_COUNT
#define CONFIG_WS_COUNT 1
#endif
#ifndef CONFIG_WS_GPIO
#define CONFIG_WS_GPIO 48
#endif

#define WS_COUNT   (CONFIG_WS_COUNT > 0 ? CONFIG_WS_COUNT : 1)
#define WS_GPIO    ((gpio_num_t)CONFIG_WS_GPIO)
#define RMT_RES_HZ 10000000

static led_strip_handle_t s_strip;
static volatile ws_mode_t s_mode = WS_MODE_RAINBOW;
static volatile uint8_t   s_bright = 96;
static volatile int       s_notify;
static uint8_t            s_solid[3] = { 0, 0, 255 };
static uint32_t           s_tick;

/* ---------------- HSV -> RGB ---------------- */
static void hsv2rgb(uint16_t h, uint8_t s, uint8_t v,
                    uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (s == 0) { *r = *g = *b = v; return; }

    uint8_t region = (uint8_t)(h / 60);
    uint8_t rem    = (uint8_t)(h % 60);

    uint32_t base = ((uint32_t)v * (255 - s)) >> 8;
    uint32_t up   = ((uint32_t)v * (255 - ((uint32_t)s * rem) / 60)) >> 8;
    uint32_t down = ((uint32_t)v * (255 - ((uint32_t)s * (60 - rem)) / 60)) >> 8;

    switch (region) {
        case 0:  *r = v;     *g = down; *b = base; break;
        case 1:  *r = up;    *g = v;    *b = base; break;
        case 2:  *r = base;  *g = v;    *b = down; break;
        case 3:  *r = base;  *g = up;   *b = v;    break;
        case 4:  *r = down;  *g = base; *b = v;    break;
        default: *r = v;     *g = base; *b = up;   break;
    }
}

static void push(uint8_t r, uint8_t g, uint8_t b)
{
    r = (uint8_t)(((uint16_t)r * s_bright) >> 8);
    g = (uint8_t)(((uint16_t)g * s_bright) >> 8);
    b = (uint8_t)(((uint16_t)b * s_bright) >> 8);
    for (int i = 0; i < WS_COUNT; i++) led_strip_set_pixel(s_strip, i, r, g, b);
    led_strip_refresh(s_strip);
}

/* ---------------- efectos ---------------- */
static void fx_rainbow(void)
{
    uint16_t hue = (uint16_t)((s_tick * 2) % 360);
    uint8_t r, g, b;
    hsv2rgb(hue, 255, 255, &r, &g, &b);
    push(r, g, b);
}

static void fx_breathe(void)
{
    uint32_t phase = s_tick % 120;
    uint8_t v = (phase < 60) ? (uint8_t)(phase * 255 / 60)
                             : (uint8_t)((120 - phase) * 255 / 60);
    uint8_t r, g, b;
    hsv2rgb(200, 40, v, &r, &g, &b);
    push(r, g, b);
}

static void fx_pulse(void)
{
    /* ritmo ~0.8 s; el numero de notificaciones alarga el destello */
    uint32_t phase = s_tick % 48;
    int flashes = s_notify > 0 ? s_notify : 0;
    bool on = (phase < (uint32_t)(6 + flashes * 4));
    if (on) push(0, 255, 80);        /* verde: carrier en el aire */
    else    push(0, 0, 0);
}

static void fx_solid(void)
{
    push(s_solid[0], s_solid[1], s_solid[2]);
}

/* ---------------- tarea ---------------- */
static void ws_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(16);   /* ~60 fps */
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        s_tick++;
        switch (s_mode) {
            case WS_MODE_RAINBOW: fx_rainbow(); break;
            case WS_MODE_BREATHE: fx_breathe(); break;
            case WS_MODE_PULSE:   fx_pulse();   break;
            case WS_MODE_SOLID:   fx_solid();   break;
            case WS_MODE_OFF:
            default:              led_strip_clear(s_strip); break;
        }
        xTaskDelayUntil(&last, period);
    }
}

/* ---------------- API ---------------- */
bool ws2812_ready(void) { return s_strip != NULL; }

esp_err_t ws2812_init(void)
{
    if (s_strip) return ESP_OK;

    led_strip_config_t cfg = {
        .strip_gpio_num          = WS_GPIO,
        .max_leds                = WS_COUNT,
        .led_model               = LED_MODEL_WS2812,
        .color_component_format  = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out        = false,
    };
    led_strip_rmt_config_t rmt = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = RMT_RES_HZ,
        .mem_block_symbols = 64,
    };

    esp_err_t e = led_strip_new_rmt_device(&cfg, &rmt, &s_strip);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_new_rmt_device gpio%d: %s", CONFIG_WS_GPIO,
                 esp_err_to_name(e));
        return e;
    }
    led_strip_clear(s_strip);

    if (xTaskCreatePinnedToCore(ws_task, "ws2812", 3072, NULL, 3, NULL,
                                tskNO_AFFINITY) != pdPASS) {
        ESP_LOGE(TAG, "no se pudo crear la tarea");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "WS2812 listo gpio%d x%d (arcoiris)", CONFIG_WS_GPIO, WS_COUNT);
    return ESP_OK;
}

void ws2812_deinit(void)
{
    if (!s_strip) return;
    led_strip_clear(s_strip);
    led_strip_del(s_strip);
    s_strip = NULL;
}

void      ws2812_set_mode(ws_mode_t m) { s_mode = m; s_tick = 0; }
ws_mode_t ws2812_get_mode(void)        { return s_mode; }

void ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    s_solid[0] = r; s_solid[1] = g; s_solid[2] = b;
    s_mode = WS_MODE_SOLID;
}

void ws2812_set_brightness(uint8_t br) { s_bright = br; }

void ws2812_notify(int count) { s_notify += count; }
