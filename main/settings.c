#include "settings.h"
#include <math.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "settings";
static nvs_handle_t h;
static bool ok = false;

void settings_init(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        e = nvs_flash_init();
    }
    if (e == ESP_OK && nvs_open("horizon", NVS_READWRITE, &h) == ESP_OK) {
        ok = true;
    } else {
        ESP_LOGW(TAG, "NVS nicht verfügbar, Einstellungen werden nicht gespeichert");
    }
}

static bool get_f(const char *key, float *v)
{
    if (!ok) return false;
    float t = 0;
    size_t n = sizeof(t);
    if (nvs_get_blob(h, key, &t, &n) == ESP_OK && n == sizeof(t)) {
        *v = t;
        return true;
    }
    return false;
}

static void set_f(const char *key, float v)
{
    if (ok) nvs_set_blob(h, key, &v, sizeof(v));
}

bool settings_load_qnh(float *qnh_hpa)
{
    float v;
    if (get_f("qnh", &v) && v >= 850.0f && v <= 1100.0f) {
        *qnh_hpa = v;
        return true;
    }
    return false;
}

bool settings_load_trim(float *roll_rad, float *pitch_rad)
{
    float r, p;
    if (get_f("roll_trim", &r) && get_f("pitch_trim", &p) && fabsf(r) < 1.6f && fabsf(p) < 1.6f) {
        *roll_rad = r;
        *pitch_rad = p;
        return true;
    }
    return false;
}

bool settings_load_bias(float b[3])
{
    float x, y, z;
    if (get_f("gyro_bx", &x) && get_f("gyro_by", &y) && get_f("gyro_bz", &z) &&
        fabsf(x) < 0.2f && fabsf(y) < 0.2f && fabsf(z) < 0.2f) {
        b[0] = x; b[1] = y; b[2] = z;
        return true;
    }
    return false;
}

void settings_save_bias(const float b[3])
{
    set_f("gyro_bx", b[0]);
    set_f("gyro_by", b[1]);
    set_f("gyro_bz", b[2]);
    if (ok) nvs_commit(h);
}

void settings_save_qnh(float qnh_hpa)
{
    set_f("qnh", qnh_hpa);
    if (ok) nvs_commit(h);
}

void settings_save_trim(float roll_rad, float pitch_rad)
{
    set_f("roll_trim", roll_rad);
    set_f("pitch_trim", pitch_rad);
    if (ok) nvs_commit(h);
}
