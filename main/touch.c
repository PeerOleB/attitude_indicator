/*
 * Minimaler Treiber für den Touch-Controller AXS5106L (I2C 0x63, INT aktiv low).
 *
 * Hinweis: Das Registerlayout (Register 0x01, 14 Bytes: [1] = Anzahl Berührungen,
 * [2..5] = 12-Bit-X/Y des ersten Punkts) stammt aus meiner Kenntnis verwandter
 * Treiber und ist nicht gegen das Datenblatt geprüft. Mit TOUCH_DEBUG 1 werden die
 * Rohdaten ausgegeben, damit man es am echten Gerät verifizieren kann.
 */
#include "touch.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "config.h"

#ifndef TOUCH_ADDR
#define TOUCH_ADDR 0x63
#endif
#ifndef PIN_TP_INT
#define PIN_TP_INT 21
#endif
#ifndef PIN_TP_RST
#define PIN_TP_RST 20
#endif
#ifndef TOUCH_FLIP_X
#define TOUCH_FLIP_X 0      // 1 = Touch-X zusätzlich umdrehen
#endif
#ifndef TOUCH_FLIP_Y
#define TOUCH_FLIP_Y 0      // 1 = Touch-Y zusätzlich umdrehen
#endif
#ifndef TOUCH_DEBUG
#define TOUCH_DEBUG 0       // 1 = Rohdaten und Koordinaten im Monitor ausgeben
#endif

static const char *TAG = "touch";

static i2c_master_dev_handle_t dev;
static bool ok = false;
static volatile bool irq_flag = false;
static bool down = false;
static int cur_x = 0, cur_y = 0;
static int64_t last_poll_us = 0;
static int64_t init_us = 0;
static int stable = 0;          // aufeinanderfolgende gültige Messungen (Entprellung)
static int miss = 0;            // aufeinanderfolgende Messungen ohne Berührung

static void IRAM_ATTR touch_isr(void *arg)
{
    irq_flag = true;
}

static bool read_raw(uint8_t *d, size_t n);

bool touch_init(i2c_master_bus_handle_t bus)
{
    gpio_config_t rst = { .pin_bit_mask = 1ULL << PIN_TP_RST, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&rst);
    gpio_set_level(PIN_TP_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_TP_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(250));          // Controller braucht nach dem Reset etwas Zeit

    if (i2c_master_probe(bus, TOUCH_ADDR, 50) != ESP_OK) {
        ESP_LOGW(TAG, "Kein Touch-Controller @0x%02X", TOUCH_ADDR);
        return false;
    }
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TOUCH_ADDR,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(bus, &dc, &dev) != ESP_OK) return false;

    gpio_config_t in = {
        .pin_bit_mask = 1ULL << PIN_TP_INT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&in);
    gpio_install_isr_service(0);             // liefert Fehler, falls schon installiert: ignorieren
    gpio_isr_handler_add(PIN_TP_INT, touch_isr, NULL);

    ok = true;
    init_us = esp_timer_get_time();
    uint8_t d[14] = { 0 };
    bool rd = read_raw(d, sizeof(d));
    ESP_LOGI(TAG, "Touch @0x%02X bereit, Ruhe-Rohdaten (%s): %02x %02x %02x %02x %02x %02x",
             TOUCH_ADDR, rd ? "ok" : "Lesefehler", d[0], d[1], d[2], d[3], d[4], d[5]);
    return true;
}

// Liest einen Datensatz. false = Controller hat nicht geantwortet.
// Laut Referenzcode muss die Registeradresse in einer eigenen I2C-Übertragung
// (ohne Repeated Start) gesendet werden, mit mindestens 45 us Pause vor dem Lesen.
static bool read_raw(uint8_t *d, size_t n)
{
    uint8_t reg = 0x01;
    esp_log_level_set("i2c.master", ESP_LOG_NONE);      // NACKs im Ruhezustand nicht melden
    esp_err_t e = i2c_master_transmit(dev, &reg, 1, 20);
    if (e == ESP_OK) {
        esp_rom_delay_us(60);
        e = i2c_master_receive(dev, d, n, 20);
    }
    esp_log_level_set("i2c.master", ESP_LOG_ERROR);
    return e == ESP_OK;
}

// Liefert true bei gültiger Berührung (Anzahl 1..5 und Koordinaten im Panelbereich).
static bool read_touch(int *tx, int *ty)
{
    uint8_t d[14] = { 0 };
    if (!read_raw(d, sizeof(d))) return false;

    int cnt = d[1];
    int x = ((d[2] & 0x0F) << 8) | d[3];
    int y = ((d[4] & 0x0F) << 8) | d[5];
#if TOUCH_DEBUG
    ESP_LOGI(TAG, "raw: %02x %02x %02x %02x %02x %02x %02x %02x | cnt=%d x=%d y=%d",
             d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], cnt, x, y);
#endif
    if (cnt < 1 || cnt > 5) return false;
    if (x > 190 || y > 340) return false;                 // Panel ist 172 x 320: Unsinn verwerfen
    *tx = x;
    *ty = y;
    return true;
}

void touch_poll(touch_event_t *ev)
{
    ev->pressed_edge = false;
    ev->released_edge = false;
    if (!ok) { ev->down = false; ev->x = ev->y = 0; return; }

    int64_t now = esp_timer_get_time();
    if (now - init_us < 800000) {            // kurz nach dem Start alles ignorieren
        ev->down = false; ev->x = cur_x; ev->y = cur_y;
        return;
    }

    // Lesen: bei Interrupt, solange der Finger aufliegt oder eine Berührung gerade
    // bestätigt wird, sonst alle 50 ms.
    bool want = irq_flag || down || stable > 0 || (now - last_poll_us > 50000);
    if (want) {
        irq_flag = false;
        last_poll_us = now;
        int tx = 0, ty = 0;
        if (read_touch(&tx, &ty)) {
            // Native Hochformat-Koordinaten (172x320) -> Querformat-Bildschirm
            int sx = ty, sy = tx;
            if (LCD_MIRROR_X ^ TOUCH_FLIP_X) sx = W - 1 - sx;
            if (LCD_MIRROR_Y ^ TOUCH_FLIP_Y) sy = H - 1 - sy;
            if (sx < 0) sx = 0;
            if (sx > W - 1) sx = W - 1;
            if (sy < 0) sy = 0;
            if (sy > H - 1) sy = H - 1;
            cur_x = sx;
            cur_y = sy;
            miss = 0;
            if (!down && ++stable >= 2) {    // erst nach 2 gültigen Messungen als Berührung werten
                down = true;
                ev->pressed_edge = true;
#if TOUCH_DEBUG
                ESP_LOGI(TAG, "screen x=%d y=%d", sx, sy);
#endif
            }
        } else {
            stable = 0;
            if (down && ++miss >= 3) {       // erst nach 3 Fehlmessungen als losgelassen werten
                down = false;
                miss = 0;
                ev->released_edge = true;
            }
        }
    }
    ev->down = down;
    ev->x = cur_x;
    ev->y = cur_y;
}
