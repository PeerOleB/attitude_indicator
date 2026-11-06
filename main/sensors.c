#include "sensors.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "config.h"

static const char *TAG = "sensors";

sensor_data_t g_sens;

/* ------------------------------------------------------------------ */
/*  BMP280                                                             */
/* ------------------------------------------------------------------ */
static i2c_master_dev_handle_t bmp;
static uint16_t T1, P1;
static int16_t T2, T3, P2, P3, P4, P5, P6, P7, P8, P9;
static int32_t t_fine;
static volatile float qnh_pa = 101325.0f;
static volatile bool recal_req = false;
static volatile bool reinit_req = false;

static esp_err_t bmp_rd(uint8_t reg, uint8_t *d, size_t n)
{
    return i2c_master_transmit_receive(bmp, &reg, 1, d, n, 50);
}

static esp_err_t bmp_wr(uint8_t reg, uint8_t v)
{
    uint8_t b[2] = { reg, v };
    return i2c_master_transmit(bmp, b, 2, 50);
}

// Kompensationsformeln aus dem Bosch-Datenblatt (Ganzzahl-Variante)
static int32_t comp_T(int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)T1 << 1))) * ((int32_t)T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)T1)) * ((adc_T >> 4) - ((int32_t)T1))) >> 12) *
                    ((int32_t)T3)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;      // 0,01 °C
}

static uint32_t comp_P(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)P6;
    var2 = var2 + ((var1 * (int64_t)P5) << 17);
    var2 = var2 + (((int64_t)P4) << 35);
    var1 = ((var1 * var1 * (int64_t)P3) >> 8) + ((var1 * (int64_t)P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)P1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)P7) << 4);
    return (uint32_t)p;                  // Pa in Q24.8
}

static bool baro_init(i2c_master_bus_handle_t bus)
{
    uint8_t addr = 0;
    if (i2c_master_probe(bus, 0x76, 50) == ESP_OK) addr = 0x76;
    else if (i2c_master_probe(bus, 0x77, 50) == ESP_OK) addr = 0x77;
    if (!addr) { ESP_LOGW(TAG, "Kein BMP280 gefunden (0x76/0x77)"); return false; }

    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(bus, &dc, &bmp) != ESP_OK) return false;

    uint8_t id = 0;
    bmp_rd(0xD0, &id, 1);
    ESP_LOGI(TAG, "Baro @0x%02X ID=0x%02X (BMP280: 0x56-0x58, BME280: 0x60)", addr, id);
    if (id != 0x56 && id != 0x57 && id != 0x58 && id != 0x60) return false;

    bmp_wr(0xE0, 0xB6);                  // Reset
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t c[24];
    if (bmp_rd(0x88, c, 24) != ESP_OK) return false;
    T1 = c[0] | (c[1] << 8);   T2 = (int16_t)(c[2] | (c[3] << 8));   T3 = (int16_t)(c[4] | (c[5] << 8));
    P1 = c[6] | (c[7] << 8);   P2 = (int16_t)(c[8] | (c[9] << 8));   P3 = (int16_t)(c[10] | (c[11] << 8));
    P4 = (int16_t)(c[12] | (c[13] << 8)); P5 = (int16_t)(c[14] | (c[15] << 8));
    P6 = (int16_t)(c[16] | (c[17] << 8)); P7 = (int16_t)(c[18] | (c[19] << 8));
    P8 = (int16_t)(c[20] | (c[21] << 8)); P9 = (int16_t)(c[22] | (c[23] << 8));

    bmp_wr(0xF5, 0x04);                  // Standby 0,5 ms, IIR-Filter 2
    bmp_wr(0xF4, 0x53);                  // Temp x2, Druck x8, Normal-Modus
    return true;
}

static void baro_task(void *arg)
{
    int64_t last = esp_timer_get_time();
    bool init = false;
    float alt_est = 0, vs_est = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(BARO_PERIOD_MS));
        uint8_t d[6];
        if (bmp_rd(0xF7, d, 6) != ESP_OK) continue;
        int32_t adc_P = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
        int32_t adc_T = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
        int32_t t = comp_T(adc_T);
        uint32_t p = comp_P(adc_P);
        if (p == 0) continue;
        float pa = p / 256.0f;

        int64_t now = esp_timer_get_time();
        float dt = (now - last) * 1e-6f;
        last = now;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.5f) dt = 0.5f;

        if (recal_req) {
            float target = g_sens.gps_alt_valid ? g_sens.gps_alt_m : 0.0f;
            qnh_pa = pa / powf(1.0f - target / 44330.0f, 5.255f);
            init = false;
            recal_req = false;
            ESP_LOGI(TAG, "Hoehe kalibriert auf %.0f m, QNH %.1f hPa", target, qnh_pa / 100.0f);
        }

        if (reinit_req) { init = false; reinit_req = false; }   // QNH wurde geändert -> Filter neu starten

        float alt = 44330.0f * (1.0f - powf(pa / qnh_pa, 0.190295f));

        if (!init) {
            alt_est = alt;
            vs_est = 0;
            init = true;
        } else {
            // Alpha-Beta-Filter: glättet die Höhe und liefert die Steigrate
            float pred = alt_est + vs_est * dt;
            float r = alt - pred;
            alt_est = pred + VARIO_ALPHA * r;
            vs_est += (VARIO_BETA / dt) * r;
        }

        g_sens.pressure_pa = pa;
        g_sens.temp_c = t / 100.0f;
        g_sens.alt_m = alt_est;
        g_sens.vs_ms = vs_est;
        g_sens.baro_ok = true;
    }
}

void sensors_calibrate_alt(void)
{
    recal_req = true;
}

float sensors_get_qnh_hpa(void)
{
    return qnh_pa / 100.0f;
}

void sensors_set_qnh_hpa(float hpa)
{
    if (hpa < 850.0f) hpa = 850.0f;
    if (hpa > 1100.0f) hpa = 1100.0f;
    qnh_pa = hpa * 100.0f;
    reinit_req = true;
}

bool sensors_qnh_from_gps(void)
{
    if (!g_sens.baro_ok || !g_sens.gps_alt_valid || g_sens.pressure_pa < 30000.0f) return false;
    float target = g_sens.gps_alt_m;
    qnh_pa = g_sens.pressure_pa / powf(1.0f - target / 44330.0f, 5.255f);
    reinit_req = true;
    return true;
}

/* ------------------------------------------------------------------ */
/*  GPS (NMEA)                                                         */
/* ------------------------------------------------------------------ */
static int64_t gps_last_us = 0;

static void gps_parse(char *s)
{
    if (s[0] != '$') return;
    char *star = strchr(s, '*');
    if (star) {
        uint8_t cs = 0;
        for (char *p = s + 1; p < star; p++) cs ^= (uint8_t)*p;
        if (cs != (uint8_t)strtol(star + 1, NULL, 16)) return;
        *star = 0;
    }
    char *f[20];
    int nf = 0;
    f[nf++] = s;
    for (char *p = s; *p && nf < 20; p++) {
        if (*p == ',') { *p = 0; f[nf++] = p + 1; }
    }
    if (nf < 8 || strlen(f[0]) < 6) return;
    const char *type = f[0] + 3;         // nach "$GP", "$GN", "$GL" ...

    if (!strncmp(type, "RMC", 3)) {
        if (f[2][0] == 'A') {
            g_sens.gps_speed_kmh = strtof(f[7], NULL) * 1.852f;   // Knoten -> km/h
            if (nf > 8 && f[8][0]) g_sens.gps_track_deg = strtof(f[8], NULL);
            gps_last_us = esp_timer_get_time();
            g_sens.gps_valid = true;
        } else {
            g_sens.gps_valid = false;
        }
    } else if (!strncmp(type, "GGA", 3) && nf >= 10) {
        g_sens.gps_sats = atoi(f[7]);
        if (atoi(f[6]) > 0 && f[9][0]) {
            g_sens.gps_alt_m = strtof(f[9], NULL);
            g_sens.gps_alt_valid = true;
        } else {
            g_sens.gps_alt_valid = false;
        }
    }
}

static void gps_task(void *arg)
{
    uart_config_t cfg = {
        .baud_rate = GPS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART, UART_PIN_NO_CHANGE, GPS_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    char line[128];
    int n = 0;
    uint8_t buf[64];
    while (1) {
        int len = uart_read_bytes(GPS_UART, buf, sizeof(buf), pdMS_TO_TICKS(50));
        for (int i = 0; i < len; i++) {
            char ch = (char)buf[i];
            if (ch == '$') n = 0;
            if (ch == '\n' || ch == '\r') {
                if (n > 6) { line[n] = 0; gps_parse(line); }
                n = 0;
            } else if (n < (int)sizeof(line) - 1) {
                line[n++] = ch;
            }
        }
        if (g_sens.gps_valid && esp_timer_get_time() - gps_last_us > 3000000) {
            g_sens.gps_valid = false;    // länger als 3 s keine gültigen Daten
        }
    }
}

/* ------------------------------------------------------------------ */
void sensors_init(i2c_master_bus_handle_t bus)
{
    if (baro_init(bus)) xTaskCreate(baro_task, "baro", 4096, NULL, 5, NULL);
    xTaskCreate(gps_task, "gps", 4096, NULL, 4, NULL);
}
