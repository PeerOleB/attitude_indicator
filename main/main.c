/*
 * Künstlicher Horizont mit Speed-, Höhen- und Variometerband (Garmin-G3X-Stil)
 * Board: Spotpear / Waveshare ESP32-C6-Touch-LCD-1.47
 *
 * Bedienung (BOOT-Taste):
 *   kurz drücken (oder Touch-Tipp): aktuelle Lage als Null setzen
 *   lang drücken (> 1,5 s):         Höhe kalibrieren (auf GPS-Höhe, sonst 0 m)
 *
 * Alle Einstellungen stehen in config.h.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "config.h"
#include "sensors.h"
#include "touch.h"
#include "settings.h"

static const char *TAG = "horizon";

#ifndef COLOR_TEST
#define COLOR_TEST 0        // 1 = nur Farbbalken zeigen (zum Einstellen von LCD_INVERT / LCD_RGB_BGR)
#endif

#define PI_F      3.14159265f
#define DEG2RAD   (PI_F / 180.0f)
#define RAD2DEG   (180.0f / PI_F)

/* ------------------------------------------------------------------ */
/*  Farben (RGB565, byte-getauscht für SPI)                            */
/* ------------------------------------------------------------------ */
#define RGB(r, g, b) ((uint16_t)__builtin_bswap16( \
    (uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))))
#define COL_SKY     RGB(45, 130, 225)
#define COL_GROUND  RGB(160, 105, 55)
#define COL_WHITE   RGB(255, 255, 255)
#define COL_BLACK   RGB(0, 0, 0)
#define COL_YELLOW  RGB(245, 230, 65)
#define COL_GRAY    RGB(150, 150, 150)
#define COL_GREEN   RGB(60, 220, 60)
#define COL_RED     RGB(255, 70, 70)

/* ------------------------------------------------------------------ */
/*  Framebuffer + Zeichen-Primitive                                    */
/* ------------------------------------------------------------------ */
static uint16_t *fb[2];
static uint16_t *cur;
static SemaphoreHandle_t tx_done;

static inline void px(int x, int y, uint16_t c)
{
    if ((unsigned)x < W && (unsigned)y < H) cur[y * W + x] = c;
}

static void box(int x, int y, int w, int h, uint16_t c)
{
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++) px(x + i, y + j, c);
}

static void line(int x0, int y0, int x1, int y1, int t, uint16_t c)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        box(x0 - t / 2, y0 - t / 2, t, t, c);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// Bereich auf ca. 25 % Helligkeit abdunkeln (halbtransparentes Band)
static void shade(int x0, int x1, int y0, int y1)
{
    for (int y = y0; y < y1; y++) {
        for (int x = x0; x < x1; x++) {
            uint16_t v = __builtin_bswap16(cur[y * W + x]);
            v = ((v >> 1) & 0x7BEF) + ((v >> 2) & 0x39E7);
            cur[y * W + x] = __builtin_bswap16(v);
        }
    }
}

// 3x5-Ziffernfont, 15 Bit, MSB = oben links
static const uint16_t font3x5[10] = {
    0x7B6F, 0x2C97, 0x73E7, 0x73CF, 0x5BC9,
    0x79CF, 0x79EF, 0x7249, 0x7BEF, 0x7BCF
};

static void digit(int x, int y, int d, int s, uint16_t c)
{
    for (int r = 0; r < 5; r++)
        for (int col = 0; col < 3; col++)
            if (font3x5[d] & (1 << (14 - (r * 3 + col))))
                box(x + col * s, y + r * s, s, s, c);
}

static int ndigits(int v)
{
    int n = 1;
    v = abs(v);
    while (v >= 10) { v /= 10; n++; }
    return n;
}

static int num_w(int v, int s)
{
    int w = ndigits(v) * 4 * s - s;
    if (v < 0) w += 4 * s;
    return w;
}

// größte Skalierung (max. 3), bei der die Zahl in maxw Pixel passt
static int fit_scale(int v, int maxw, int smax)
{
    for (int s = smax; s > 1; s--)
        if (num_w(v, s) <= maxw) return s;
    return 1;
}

// Zahl rechtsbündig bis einschließlich Pixel xr zeichnen, y = Oberkante
static void draw_num(int xr, int y, int v, int s, uint16_t c)
{
    int x = xr - num_w(v, s) + 1;
    if (v < 0) { box(x, y + 2 * s, 3 * s, s, c); x += 4 * s; }
    int a = abs(v), n = ndigits(a), div = 1;
    for (int i = 0; i < n - 1; i++) div *= 10;
    for (int i = 0; i < n; i++) {
        digit(x, y, (a / div) % 10, s, c);
        x += 4 * s;
        div /= 10;
    }
}

// Positive Zahl um den Mittelpunkt (cx, cy) gedreht gezeichnet. Die Schrift läuft entlang der
// Horizontlinie: sn / cs = sin / cos des Rollwinkels. Rückwärts-Abbildung, daher ohne Lücken.
static void draw_num_rot(int cx, int cy, int v, int sc, float sn, float cs, uint16_t col)
{
    const int a = abs(v);
    const int n = ndigits(a);
    const int tw = n * 4 * sc - sc;                // Textbreite in Pixel
    const int th = 5 * sc;                         // Texthöhe in Pixel
    const int r = (int)ceilf(sqrtf((float)(tw * tw + th * th)) * 0.5f) + 1;
    for (int y = cy - r; y <= cy + r; y++) {
        for (int x = cx - r; x <= cx + r; x++) {
            float dx = (float)(x - cx), dy = (float)(y - cy);
            float u = dx * cs - dy * sn + tw * 0.5f;       // Position im ungedrehten Text
            float w = dx * sn + dy * cs + th * 0.5f;
            if (u < 0.0f || w < 0.0f || u >= tw || w >= th) continue;
            int di = (int)u / (4 * sc);
            int lx = (int)u - di * 4 * sc;
            if (lx >= 3 * sc) continue;                    // Lücke zwischen den Ziffern
            int div = 1;
            for (int k = 0; k < n - 1 - di; k++) div *= 10;
            int dig = (a / div) % 10;
            if (font3x5[dig] & (1 << (14 - (((int)w / sc) * 3 + lx / sc)))) px(x, y, col);
        }
    }
}

// Zahl mit einer Nachkommastelle (z. B. -2.5)
static void draw_vs_text(int xr, int y, float vs, uint16_t col)
{
    const int s = 2;
    int t = (int)lroundf(vs * 10.0f);
    bool neg = t < 0;
    int a = abs(t), ip = a / 10, fp = a % 10;
    int n = ndigits(ip);
    int w = (neg ? 4 * s : 0) + n * 4 * s + 2 * s + 3 * s;
    int x = xr - w + 1;
    if (neg) { box(x, y + 2 * s, 3 * s, s, col); x += 4 * s; }
    int div = 1;
    for (int i = 0; i < n - 1; i++) div *= 10;
    for (int i = 0; i < n; i++) {
        digit(x, y, (ip / div) % 10, s, col);
        x += 4 * s;
        div /= 10;
    }
    box(x, y + 4 * s, s, s, col);
    x += 2 * s;
    digit(x, y, fp, s, col);
}

// "---" bei fehlenden Daten
static void dashes(int xr, int cy)
{
    int x0 = xr - 29;
    for (int i = 0; i < 3; i++) box(x0 + i * 11, cy - 1, 8, 3, COL_YELLOW);
}

/* ------------------------------------------------------------------ */
/*  Text (3x5-Pixelfont, Großbuchstaben) und Touch-Menü                */
/* ------------------------------------------------------------------ */
// 5x7-Pixelfont: Ziffern 0-9, danach Buchstaben A-Z (5 Bit pro Zeile, Bit 4 = links)
static const uint8_t F7[36][7] = {
    {0b01110,0b10001,0b10011,0b10101,0b11001,0b10001,0b01110},  // 0
    {0b00100,0b01100,0b00100,0b00100,0b00100,0b00100,0b01110},  // 1
    {0b01110,0b10001,0b00001,0b00010,0b00100,0b01000,0b11111},  // 2
    {0b11110,0b00001,0b00001,0b01110,0b00001,0b00001,0b11110},  // 3
    {0b00010,0b00110,0b01010,0b10010,0b11111,0b00010,0b00010},  // 4
    {0b11111,0b10000,0b11110,0b00001,0b00001,0b10001,0b01110},  // 5
    {0b00110,0b01000,0b10000,0b11110,0b10001,0b10001,0b01110},  // 6
    {0b11111,0b00001,0b00010,0b00100,0b01000,0b01000,0b01000},  // 7
    {0b01110,0b10001,0b10001,0b01110,0b10001,0b10001,0b01110},  // 8
    {0b01110,0b10001,0b10001,0b01111,0b00001,0b00010,0b01100},  // 9
    {0b01110,0b10001,0b10001,0b11111,0b10001,0b10001,0b10001},  // A
    {0b11110,0b10001,0b10001,0b11110,0b10001,0b10001,0b11110},  // B
    {0b01110,0b10001,0b10000,0b10000,0b10000,0b10001,0b01110},  // C
    {0b11110,0b10001,0b10001,0b10001,0b10001,0b10001,0b11110},  // D
    {0b11111,0b10000,0b10000,0b11110,0b10000,0b10000,0b11111},  // E
    {0b11111,0b10000,0b10000,0b11110,0b10000,0b10000,0b10000},  // F
    {0b01110,0b10001,0b10000,0b10111,0b10001,0b10001,0b01111},  // G
    {0b10001,0b10001,0b10001,0b11111,0b10001,0b10001,0b10001},  // H
    {0b01110,0b00100,0b00100,0b00100,0b00100,0b00100,0b01110},  // I
    {0b00111,0b00010,0b00010,0b00010,0b00010,0b10010,0b01100},  // J
    {0b10001,0b10010,0b10100,0b11000,0b10100,0b10010,0b10001},  // K
    {0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b11111},  // L
    {0b10001,0b11011,0b10101,0b10101,0b10001,0b10001,0b10001},  // M
    {0b10001,0b11001,0b10101,0b10011,0b10001,0b10001,0b10001},  // N
    {0b01110,0b10001,0b10001,0b10001,0b10001,0b10001,0b01110},  // O
    {0b11110,0b10001,0b10001,0b11110,0b10000,0b10000,0b10000},  // P
    {0b01110,0b10001,0b10001,0b10001,0b10101,0b10010,0b01101},  // Q
    {0b11110,0b10001,0b10001,0b11110,0b10100,0b10010,0b10001},  // R
    {0b01111,0b10000,0b10000,0b01110,0b00001,0b00001,0b11110},  // S
    {0b11111,0b00100,0b00100,0b00100,0b00100,0b00100,0b00100},  // T
    {0b10001,0b10001,0b10001,0b10001,0b10001,0b10001,0b01110},  // U
    {0b10001,0b10001,0b10001,0b10001,0b10001,0b01010,0b00100},  // V
    {0b10001,0b10001,0b10001,0b10101,0b10101,0b11011,0b10001},  // W
    {0b10001,0b10001,0b01010,0b00100,0b01010,0b10001,0b10001},  // X
    {0b10001,0b10001,0b01010,0b00100,0b00100,0b00100,0b00100},  // Y
    {0b11111,0b00001,0b00010,0b00100,0b01000,0b10000,0b11111},  // Z
};

static const uint8_t *g7(char ch)
{
    if (ch >= '0' && ch <= '9') return F7[ch - '0'];
    if (ch >= 'A' && ch <= 'Z') return F7[10 + ch - 'A'];
    return NULL;
}

// Schrifthöhe = 7*s Pixel, Zeichenbreite = 5*s (+ 1*s Abstand)
static int text_w(const char *t, int s)
{
    int w = 0;
    for (; *t; t++) w += (*t == '.') ? 3 * s : 6 * s;
    return w > 0 ? w - s : 0;
}

static void draw_text(int x, int y, const char *t, int s, uint16_t c)
{
    for (; *t; t++) {
        char ch = *t;
        if (ch >= 'a' && ch <= 'z') ch -= 32;
        const uint8_t *g = g7(ch);
        if (g) {
            for (int r = 0; r < 7; r++)
                for (int col = 0; col < 5; col++)
                    if (g[r] & (1 << (4 - col))) box(x + col * s, y + r * s, s, s, c);
            x += 6 * s;
        } else if (ch == '-') { box(x + s, y + 3 * s, 3 * s, s, c); x += 6 * s; }
        else if (ch == '+') { box(x + s, y + 3 * s, 3 * s, s, c); box(x + 2 * s, y + 2 * s, s, 3 * s, c); x += 6 * s; }
        else if (ch == '.') { box(x, y + 5 * s, 2 * s, 2 * s, c); x += 3 * s; }
        else x += 6 * s;
    }
}

static void draw_text_c(int cx, int y, const char *t, int s, uint16_t c)
{
    draw_text(cx - text_w(t, s) / 2, y, t, s, c);
}

// Zahl mit einer Nachkommastelle, mittig um cx
static void draw_fixed1_c(int cx, int y, float v, int s, uint16_t c)
{
    char b[16];
    snprintf(b, sizeof(b), "%.1f", v);
    draw_text_c(cx, y, b, s, c);
}

#define COL_DARK    RGB(30, 30, 34)
#define COL_BTN     RGB(105, 112, 130)
#define COL_BTN_ON  RGB(255, 205, 0)

typedef struct { int16_t x, y, w, h; const char *label; } mbtn_t;
enum { B_M1, B_P1, B_M01, B_P01, B_GPS, B_NULL, B_OK, B_N };

#define PNL_X   40
#define PNL_Y   8
#define PNL_W   240
#define PNL_H   156

static const mbtn_t mbtn[B_N] = {
    [B_M1]   = { PNL_X + 6,   PNL_Y + 32, 44, 36, "-1"   },
    [B_P1]   = { PNL_X + 190, PNL_Y + 32, 44, 36, "+1"   },
    [B_M01]  = { PNL_X + 6,   PNL_Y + 72, 44, 28, "-.1"  },
    [B_P01]  = { PNL_X + 190, PNL_Y + 72, 44, 28, "+.1"  },
    [B_GPS]  = { PNL_X + 6,   PNL_Y + 108, 70, 40, "GPS" },
    [B_NULL] = { PNL_X + 85,  PNL_Y + 108, 70, 40, "NULL" },
    [B_OK]   = { PNL_X + 164, PNL_Y + 108, 70, 40, "OK"  },
};

static int menu_hit(int x, int y)
{
    const int m = 4;                     // etwas Toleranz um die Tasten
    for (int i = 0; i < B_N; i++) {
        const mbtn_t *b = &mbtn[i];
        if (x >= b->x - m && x < b->x + b->w + m && y >= b->y - m && y < b->y + b->h + m) return i;
    }
    return -1;
}

static void draw_menu(float qnh_hpa, int pressed_btn, const char *msg, bool finger, int fx, int fy)
{
    shade(0, W, 0, H);                                         // Hintergrund abdunkeln
    box(PNL_X - 2, PNL_Y - 2, PNL_W + 4, PNL_H + 4, COL_WHITE);
    box(PNL_X, PNL_Y, PNL_W, PNL_H, COL_DARK);

    draw_text(PNL_X + 8, PNL_Y + 6, "QNH", 2, COL_YELLOW);
    draw_text(PNL_X + PNL_W - 8 - text_w("HPA", 2), PNL_Y + 6, "HPA", 2, COL_GRAY);
    if (msg) draw_text_c(PNL_X + PNL_W / 2, PNL_Y + 6, msg, 2, COL_RED);

    draw_fixed1_c(PNL_X + PNL_W / 2, PNL_Y + 40, qnh_hpa, 3, COL_WHITE);

    for (int i = 0; i < B_N; i++) {
        const mbtn_t *b = &mbtn[i];
        bool on = (i == pressed_btn);
        box(b->x, b->y, b->w, b->h, on ? COL_BTN_ON : COL_BTN);
        int s = (b->h >= 36) ? 3 : 2;
        if (i >= B_GPS) s = 2;
        uint16_t tc = on ? COL_BLACK : COL_WHITE;
        draw_text_c(b->x + b->w / 2, b->y + (b->h - 7 * s) / 2, b->label, s, tc);
    }
    if (finger) {                                              // Rückmeldung zur Touch-Position
        box(fx - 3, fy - 3, 7, 7, COL_RED);
    }
}

/* ------------------------------------------------------------------ */
/*  Horizont-Rendering                                                 */
/* ------------------------------------------------------------------ */
#define ARC_N   25
#define TICK_N  10
static float arc_sa[ARC_N], arc_ca[ARC_N];
static const int tick_deg[TICK_N] = { -60, -45, -30, -20, -10, 10, 20, 30, 45, 60 };
static float tick_sa[TICK_N], tick_ca[TICK_N];

typedef struct {
    float spd_kmh; bool spd_ok;
    float alt_m;   bool alt_ok;
    float vs;      bool vs_ok;
    bool  gps;                 // GPS empfängt gültige Daten
    int   sats;                // Anzahl Satelliten
    float qnh;                 // eingestelltes QNH in hPa
    float slip;                // Kugel: -1 (links) .. +1 (rechts)
} tapes_t;

static void scale_init(void)
{
    for (int i = 0; i < ARC_N; i++) {
        float a = (-60 + i * 5) * DEG2RAD;
        arc_sa[i] = sinf(a); arc_ca[i] = cosf(a);
    }
    for (int i = 0; i < TICK_N; i++) {
        float a = tick_deg[i] * DEG2RAD;
        tick_sa[i] = sinf(a); tick_ca[i] = cosf(a);
    }
}

static inline void ac2scr(float u, float v, float s, float c, int *x, int *y)
{
    *x = (int)lroundf(CX + u * c + v * s);
    *y = (int)lroundf(CY - u * s + v * c);
}

static inline void scale_pt(float sa, float ca, float r, float s, float c, int *x, int *y)
{
    float sb = sa * c - ca * s;
    float cb = ca * c + sa * s;
    *x = (int)lroundf(CX + r * sb);
    *y = (int)lroundf(CY - r * cb);
}

static void draw_aircraft(int t, int pad, uint16_t col)
{
    line(CX - 72, CY, CX - 28, CY, t, col);
    line(CX - 28, CY, CX - 28, CY + 12, t, col);
    line(CX + 28, CY, CX + 72, CY, t, col);
    line(CX + 28, CY, CX + 28, CY + 12, t, col);
    box(CX - 3 - pad, CY - 3 - pad, 7 + 2 * pad, 7 + 2 * pad, col);
}

/* ---- Speed-Band (links) ---- */
static void draw_speed_tape(float spd_kmh, bool ok)
{
    shade(0, SPD_W, 0, H);
    box(SPD_W - 1, 0, 1, H, COL_GRAY);
    float spd = SPEED_KT ? spd_kmh / 1.852f : spd_kmh;

    if (ok) {
        const float ppu = 16.0f / SPD_MINOR;
        const float span = 90.0f / ppu;
        int k0 = (int)floorf((spd - span) / SPD_MINOR);
        int k1 = (int)ceilf((spd + span) / SPD_MINOR);
        for (int k = k0; k <= k1; k++) {
            int val = k * SPD_MINOR;
            if (val < 0) continue;
            int y = (int)lroundf(CY - (val - spd) * ppu);
            if (y < 0 || y >= H) continue;
            bool major = (k % 2 == 0);
            int len = major ? 9 : 5;
            box(SPD_W - 1 - len, y - 1, len, 2, COL_WHITE);
            if (major && abs(y - CY) > 14) draw_num(SPD_W - 14, y - 5, val, 2, COL_WHITE);
        }
    }

    box(0, CY - 11, SPD_W - 6, 23, COL_WHITE);
    box(1, CY - 10, SPD_W - 8, 21, COL_BLACK);
    for (int i = 0; i <= 5; i++) box(SPD_W - 1 - i, CY - i, 1, 2 * i + 1, COL_WHITE);
    if (ok) {
        int v = (int)lroundf(spd);
        int s = fit_scale(v, 34, 3);
        draw_num(SPD_W - 9, CY - (5 * s) / 2, v, s, COL_WHITE);
    } else {
        dashes(SPD_W - 9, CY);
    }
}

/* ---- Höhenband (rechts, neben dem Variometer) ---- */
static void draw_alt_tape(float alt_m, bool ok)
{
    shade(ALT_X, ALT_X + ALT_W, 0, H);
    box(ALT_X, 0, 1, H, COL_GRAY);
    float alt = ALT_FT ? alt_m * 3.28084f : alt_m;

    if (ok) {
        const float ppu = 16.0f / ALT_MINOR;
        const float span = 90.0f / ppu;
        int k0 = (int)floorf((alt - span) / ALT_MINOR);
        int k1 = (int)ceilf((alt + span) / ALT_MINOR);
        for (int k = k0; k <= k1; k++) {
            int val = k * ALT_MINOR;
            int y = (int)lroundf(CY - (val - alt) * ppu);
            if (y < 0 || y >= H) continue;
            bool major = (k % 2 == 0);
            int len = major ? 7 : 4;
            box(ALT_X + 1, y - 1, len, 2, COL_WHITE);
            if (major && abs(y - CY) > 14) {
                int s = fit_scale(val, 31, 2);
                draw_num(ALT_X + ALT_W - 2, y - (5 * s) / 2, val, s, COL_WHITE);
            }
        }
    }

    box(ALT_X + 6, CY - 11, ALT_W - 6, 23, COL_WHITE);
    box(ALT_X + 7, CY - 10, ALT_W - 8, 21, COL_BLACK);
    for (int i = 0; i <= 5; i++) box(ALT_X + i, CY - i, 1, 2 * i + 1, COL_WHITE);
    if (ok) {
        int v = (int)lroundf(alt);
        int s = fit_scale(v, 34, 3);
        draw_num(ALT_X + ALT_W - 3, CY - (5 * s) / 2, v, s, COL_WHITE);
    } else {
        dashes(ALT_X + ALT_W - 3, CY);
    }
}

/* ---- Variometer (ganz rechts) ---- */
static void draw_vario(float vs, bool ok)
{
    shade(VS_X, W, 0, H);
    box(VS_X, 0, 1, H, COL_GRAY);
    if (ok) {
        float c = fminf(fmaxf(vs, -5.3f), 5.3f);
        int h = (int)lroundf(c * 16.0f);
        if (h > 0) box(VS_X + 6, CY - h, 6, h, COL_GREEN);
        else if (h < 0) box(VS_X + 6, CY, 6, -h, COL_RED);
    }
    for (int k = -5; k <= 5; k++) {
        int y = CY - k * 16;
        box(VS_X + 1, y, k == 0 ? 12 : 4, 1, COL_WHITE);
    }
    if (ok) {
        // Zahlenwert unten im Höhenband
        box(ALT_X + 8, H - 17, ALT_W - 8, 16, COL_WHITE);
        box(ALT_X + 9, H - 16, ALT_W - 10, 14, COL_BLACK);
        draw_vs_text(ALT_X + ALT_W - 4, H - 14, vs, vs >= 0 ? COL_GREEN : COL_RED);
    }
}

/* ---- Statuszeile (unten, zwischen den Bändern) ---- */
static void draw_status(const tapes_t *tp)
{
    const int x0 = SPD_W, x1 = ALT_X, y0 = H - 18;
    box(x0, y0, x1 - x0, 18, COL_BLACK);
    box(x0, y0, x1 - x0, 1, COL_GRAY);
    // GPS-Piktogramm (Standort-Pin): rot ohne Fix, grün mit Fix + Satellitenzahl
    const uint16_t gc = tp->gps ? COL_GREEN : COL_RED;
    const int px = x0 + 14, py = y0 + 2;               // Mitte des Pins, Oberkante
    for (int r = 0; r < 12; r++) {
        int hw;
        if (r < 9) { float d = (float)(r - 4); hw = (int)(sqrtf(25.0f - d * d) + 0.5f); }
        else hw = 11 - r;                               // Spitze nach unten
        box(px - hw, py + r, 2 * hw + 1, 1, gc);
    }
    box(px - 1, py + 3, 3, 3, COL_BLACK);               // Loch im Pin
    if (tp->gps) {
        char sb[16];
        int ns = tp->sats < 0 ? 0 : (tp->sats > 99 ? 99 : tp->sats);
        snprintf(sb, sizeof(sb), "%02d", ns);
        draw_text(px + 10, y0 + 3, sb, 2, COL_GREEN);
    }
    char b[24];
    snprintf(b, sizeof(b), "QNH %.1f", tp->qnh);
    draw_text(x1 - 8 - text_w(b, 2) + 1, y0 + 3, b, 2, COL_GREEN);
}

/* ---- Slip/Skid-Anzeige (Kugel) unter dem Rollzeiger ---- */
static void draw_slip(float slip)
{
    const int bx = CX - 15, by = CY - ROLL_R + 14, bw = 31, bh = 11;
    // shade(bx, bx + bw, by, by + bh);                   // halbtransparent wie die Bänder
    box(bx, by, bw, 1, COL_WHITE);
    box(bx, by + bh - 1, bw, 1, COL_WHITE);
    box(bx, by, 1, bh, COL_WHITE);
    box(bx + bw - 1, by, 1, bh, COL_WHITE);
    box(CX - 5, by, 1, bh, COL_WHITE);                 // Mittenmarken
    box(CX + 5, by, 1, bh, COL_WHITE);
    if (slip > 1.0f) slip = 1.0f;
    if (slip < -1.0f) slip = -1.0f;
    const int cx = CX + (int)lroundf(slip * 10.0f), cy = by + bh / 2;
    for (int dy = -3; dy <= 3; dy++)
        for (int dx = -3; dx <= 3; dx++)
            if (dx * dx + dy * dy <= 11) box(cx + dx, cy + dy, 1, 1, COL_WHITE);
}

static void render(float roll, float pitch, const tapes_t *tp)
{
    const float s = sinf(roll), c = cosf(roll);
    const float p = pitch * RAD2DEG * PPD;

    // --- Himmel / Boden ---
    for (int y = 0; y < H; y++) {
        const float dy = y + 0.5f - CY;
        uint16_t *row = cur + y * W;
        int g0 = 0, g1 = 0;
        if (fabsf(s) < 1e-4f) {
            if (dy * c > p) { g0 = 0; g1 = W; }
        } else {
            float xt = CX + (p - dy * c) / s - 0.5f;
            int xi = (int)floorf(xt) + 1;
            if (xi < 0) xi = 0;
            if (xi > W) xi = W;
            if (s > 0) { g0 = xi; g1 = W; } else { g0 = 0; g1 = xi; }
        }
        for (int x = 0; x < W; x++) row[x] = COL_SKY;
        for (int x = g0; x < g1; x++) row[x] = COL_GROUND;
    }

    int xa, ya, xb, yb;

    // --- Horizontlinie ---
    ac2scr(-230, p, s, c, &xa, &ya);
    ac2scr(230, p, s, c, &xb, &yb);
    line(xa, ya, xb, yb, 2, COL_WHITE);

    // --- Pitch-Leiter ---
    for (int m = -30; m <= 30; m += 5) {
        if (m == 0) continue;
        float v = p - m * PPD;
        if (fabsf(v) > 66.0f) continue;
        bool major = (m % 10 == 0);
        float half = major ? 30.0f : 14.0f;
        ac2scr(-half, v, s, c, &xa, &ya);
        ac2scr(half, v, s, c, &xb, &yb);
        line(xa, ya, xb, yb, 2, COL_WHITE);
        if (major) {
            int dir = (m > 0) ? 1 : -1;
            int ex, ey;
            ac2scr(-half, v + dir * 5, s, c, &ex, &ey); line(xa, ya, ex, ey, 2, COL_WHITE);
            ac2scr(half, v + dir * 5, s, c, &ex, &ey);  line(xb, yb, ex, ey, 2, COL_WHITE);
            int n = abs(m);
            const float lw = num_w(n, 2) * 0.5f;       // halbe Textbreite
            int lx, ly;
            ac2scr(-half - 5.0f - lw, v, s, c, &lx, &ly);
            draw_num_rot(lx, ly, n, 2, s, c, COL_WHITE);   // links, mitgedreht
            ac2scr(half + 5.0f + lw, v, s, c, &lx, &ly);
            draw_num_rot(lx, ly, n, 2, s, c, COL_WHITE);   // rechts, mitgedreht
        }
    }

    // --- Roll-Skala ---
    int px0 = 0, py0 = 0;
    for (int i = 0; i < ARC_N; i++) {
        int x, y;
        scale_pt(arc_sa[i], arc_ca[i], ROLL_R, s, c, &x, &y);
        if (i > 0) line(px0, py0, x, y, 1, COL_WHITE);
        px0 = x; py0 = y;
    }
    for (int i = 0; i < TICK_N; i++) {
        int len = (abs(tick_deg[i]) == 30 || abs(tick_deg[i]) == 60) ? 11 : 6;
        scale_pt(tick_sa[i], tick_ca[i], ROLL_R, s, c, &xa, &ya);
        scale_pt(tick_sa[i], tick_ca[i], ROLL_R - len, s, c, &xb, &yb);
        line(xa, ya, xb, yb, 2, COL_WHITE);
    }
    scale_pt(0.0f, 1.0f, ROLL_R, s, c, &xa, &ya);
    scale_pt(0.0f, 1.0f, ROLL_R - 14, s, c, &xb, &yb);
    line(xa, ya, xb, yb, 3, COL_WHITE);

    for (int i = 0; i < 10; i++) {
        int hw = (i * 5) / 9;
        int y = CY - ROLL_R + 4 + i;
        box(CX - hw - 1, y, 2 * hw + 3, 1, COL_BLACK);
        box(CX - hw, y, 2 * hw + 1, 1, COL_YELLOW);
    }

    // --- Bänder ---
    draw_speed_tape(tp->spd_kmh, tp->spd_ok);
    draw_alt_tape(tp->alt_m, tp->alt_ok);
    draw_vario(tp->vs, tp->vs_ok);
    draw_status(tp);
    draw_slip(tp->slip);

    // --- Flugzeugsymbol ---
    draw_aircraft(5, 2, COL_BLACK);
    draw_aircraft(3, 0, COL_YELLOW);
}

/* ------------------------------------------------------------------ */
/*  LCD                                                                */
/* ------------------------------------------------------------------ */
static esp_lcd_panel_handle_t panel;

static bool on_color_done(esp_lcd_panel_io_handle_t io,
                          esp_lcd_panel_io_event_data_t *ev, void *ctx)
{
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(tx_done, &hp);
    return hp == pdTRUE;
}

static void lcd_init(void)
{
    gpio_config_t bl = { .pin_bit_mask = 1ULL << PIN_LCD_BL, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&bl);
    gpio_set_level(PIN_LCD_BL, 0);

    spi_bus_config_t bus = {
        .sclk_io_num = PIN_LCD_SCLK,
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = W * H * 2 + 16,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = PIN_LCD_DC,
        .cs_gpio_num = PIN_LCD_CS,
        .pclk_hz = LCD_PCLK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 4,
        .on_color_trans_done = on_color_done,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io));

    esp_lcd_panel_dev_config_t pcfg = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_BGR ? LCD_RGB_ELEMENT_ORDER_BGR : LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &pcfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, LCD_INVERT));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, LCD_MIRROR_X, LCD_MIRROR_Y));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel, LCD_GAP_X, LCD_GAP_Y));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    tx_done = xSemaphoreCreateBinary();
    xSemaphoreGive(tx_done);
}

/* ------------------------------------------------------------------ */
/*  I2C, IMU (QMI8658), Touch (AXS5106L)                               */
/* ------------------------------------------------------------------ */
static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t imu_dev;
static bool imu_ok = false;

static esp_err_t imu_write(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(imu_dev, b, 2, 50);
}

static esp_err_t imu_read(uint8_t reg, uint8_t *d, size_t n)
{
    return i2c_master_transmit_receive(imu_dev, &reg, 1, d, n, 50);
}

static i2c_master_dev_handle_t add_dev(uint8_t addr)
{
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t h = NULL;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dc, &h));
    return h;
}

static void i2c_devices_init(void)
{
    i2c_master_bus_config_t bc = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bc, &i2c_bus));

    uint8_t addr = 0;
    if (i2c_master_probe(i2c_bus, IMU_ADDR_A, 50) == ESP_OK) addr = IMU_ADDR_A;
    else if (i2c_master_probe(i2c_bus, IMU_ADDR_B, 50) == ESP_OK) addr = IMU_ADDR_B;
    if (addr) {
        imu_dev = add_dev(addr);
        uint8_t id = 0;
        imu_read(0x00, &id, 1);
        ESP_LOGI(TAG, "IMU @0x%02X WHO_AM_I=0x%02X (erwartet 0x05)", addr, id);
        if (id == 0x05) {
            imu_write(0x60, 0xB0);
            vTaskDelay(pdMS_TO_TICKS(30));
            imu_write(0x02, 0x40);
            imu_write(0x03, 0x15);
            imu_write(0x04, 0x55);
            imu_write(0x08, 0x03);
            vTaskDelay(pdMS_TO_TICKS(50));
            imu_ok = true;
        }
    } else {
        ESP_LOGW(TAG, "Kein IMU gefunden -> Demo-Modus");
    }
}

#ifndef IMU_COMP_GAIN
#define IMU_COMP_GAIN 1.0f      // 0 = Korrektur aus, -1 = Vorzeichen umkehren
#endif
static float g_roll = 0, g_pitch = 0;
static float g_slip = 0;                    // gefilterte Kugelposition (-1..1)
// GPS-Geschwindigkeit (m/s) und deren zeitliche Änderung, für die Beschleunigungskorrektur
static float g_imu_v = 0, g_imu_vdot = 0;
static bool  g_imu_vok = false;

#ifndef IMU_ACC_TAU
#define IMU_ACC_TAU 4.0f        // Zeitkonstante der Lagekorrektur durch den Beschleunigungssensor (s)
#endif
#ifndef IMU_CAL_TIME
#define IMU_CAL_TIME 1.5f       // Dauer der Gyro-Kalibrierung nach dem Start (s), Gerät ruhig halten
#endif

/* ---- Lageschätzung: Mahony-Filter (Quaternion) mit Bias-Schätzung ---- */
static float qw = 1, qx = 0, qy = 0, qz = 0;     // Lage Körper -> Erde (NED)
static float g_bias[3] = {0, 0, 0};              // Gyro-Bias (rad/s), Achsen vorn/rechts/unten
static bool  g_bias_new = false;                 // frisch kalibriert -> speichern
static float g_trust = 0, g_roll_a = 0, g_pitch_a = 0;   // nur zur Diagnose im Log
static float cal_t = 0, cal_sum[3], cal_min[3], cal_max[3];
static int   cal_n = 0;
static bool  cal_done = false;
static float fast_t = 0;                         // Schnellausrichtung nach dem Start (s)

/* ATT_BEGIN */
static void att_from_euler(float r, float p)
{
    const float cr = cosf(r / 2), sr = sinf(r / 2), cp = cosf(p / 2), sp = sinf(p / 2);
    qw = cr * cp; qx = sr * cp; qy = cr * sp; qz = -sr * sp;
}

static void att_fuse(float fx, float fy, float fz, float p, float q, float rr, float dt)
{
    if (dt > 0.1f) dt = 0.1f;
    float gvx = -fx, gvy = -fy, gvz = -fz;           // gemessene "Lotrechte" (in g)

    // Slip/Skid: seitliche Beschleunigung relativ zur Gesamtbeschleunigung (unkorrigiert, wie eine
    // echte Libelle). Voller Ausschlag bei etwa 0,15 g.
    {
        float gn = sqrtf(gvx * gvx + gvy * gvy + gvz * gvz);
        if (gn < 0.5f) gn = 0.5f;
        float tgt = (gvy / gn) / 0.15f;
        g_slip += (tgt - g_slip) * (dt / (0.25f + dt));
    }

    // --- Start: Gyro-Bias messen, solange das Gerät ruhig gehalten wird ---
    if (!cal_done) {
        const float gyro[3] = {p, q, rr};
        if (cal_n == 0) {
            for (int i = 0; i < 3; i++) { cal_sum[i] = 0; cal_min[i] = cal_max[i] = gyro[i]; }
        }
        for (int i = 0; i < 3; i++) {
            cal_sum[i] += gyro[i];
            if (gyro[i] < cal_min[i]) cal_min[i] = gyro[i];
            if (gyro[i] > cal_max[i]) cal_max[i] = gyro[i];
        }
        cal_n++;
        cal_t += dt;
        // Lage währenddessen direkt aus dem Beschleunigungssensor
        g_roll_a = atan2f(gvy, gvz);
        g_pitch_a = atan2f(-gvx, sqrtf(gvy * gvy + gvz * gvz));
        att_from_euler(g_roll_a, g_pitch_a);
        if (cal_t >= IMU_CAL_TIME && cal_n >= 10) {
            bool stable = true;
            float mean[3];
            for (int i = 0; i < 3; i++) {
                mean[i] = cal_sum[i] / cal_n;
                if (cal_max[i] - cal_min[i] > 0.12f || fabsf(mean[i]) > 0.1f) stable = false;
            }
            if (stable) {
                for (int i = 0; i < 3; i++) g_bias[i] = mean[i];
                g_bias_new = true;
            }
            cal_done = true;
            fast_t = 3.0f;                           // 3 s lang schnell einrasten
        }
        g_roll = g_roll_a; g_pitch = g_pitch_a;
        return;
    }

    const float wx = p - g_bias[0], wy = q - g_bias[1], wz = rr - g_bias[2];

    // --- Fahrzeugbeschleunigung herausrechnen: a = w x V + dV/dt mit V = (v, 0, 0) ---
    // Der Beschleunigungssensor misst Schwerkraft MINUS Fahrzeugbeschleunigung.
    if (g_imu_vok && g_imu_v > 2.0f) {
        gvx += IMU_COMP_GAIN * g_imu_vdot / 9.81f;           // Längsbeschleunigung
        gvy += IMU_COMP_GAIN * wz * g_imu_v / 9.81f;         // Zentripetal (Kurve)
        gvz -= IMU_COMP_GAIN * wy * g_imu_v / 9.81f;         // Zentripetal (Nicken)
    }
    const float gm = sqrtf(gvx * gvx + gvy * gvy + gvz * gvz);
    g_roll_a = atan2f(gvy, gvz);
    g_pitch_a = atan2f(-gvx, sqrtf(gvy * gvy + gvz * gvz));

    // --- Vertrauen in den Beschleunigungssensor: nur bei ruhigem Geradeausflug hoch ---
    float wgt = 1.0f - fabsf(gm - 1.0f) / 0.10f;             // Betrag nahe 1 g
    if (wgt < 0.0f) wgt = 0.0f;
    const float rate = sqrtf(wx * wx + wy * wy + wz * wz);   // Drehrate (rad/s)
    const float wr = 1.0f / (1.0f + (rate / 0.15f) * (rate / 0.15f));
    float wa = 1.0f;                                         // starkes Bremsen/Beschleunigen
    if (g_imu_vok) { float k = g_imu_vdot / 1.5f; wa = 1.0f / (1.0f + k * k); }
    const float trust = wgt * wr * wa;
    g_trust = trust;

    // geschätzte Lotrechte im Körpersystem
    const float vx = 2 * (qx * qz - qw * qy);
    const float vy = 2 * (qy * qz + qw * qx);
    const float vz = qw * qw - qx * qx - qy * qy + qz * qz;
    float cx = 0, cy = 0, cz = 0;                            // Korrekturdrehrate
    if (gm > 0.3f) {
        const float mx = gvx / gm, my = gvy / gm, mz = gvz / gm;
        const float ex = my * vz - mz * vy;
        const float ey = mz * vx - mx * vz;
        const float ez = mx * vy - my * vx;
        const float kp0 = 1.0f / IMU_ACC_TAU;
        float kp = kp0 * trust;
        if (fast_t > 0) {
            fast_t -= dt;
            if (kp < 2.0f * trust) kp = 2.0f * trust;        // Schnellausrichtung nach dem Start (nur bei ruhigem Zustand)
        } else {
            const float ki = 0.25f * kp0 * kp0 * trust;      // Bias nachlernen, nur bei hohem Vertrauen
            g_bias[0] -= ki * ex * dt; g_bias[1] -= ki * ey * dt; g_bias[2] -= ki * ez * dt;
            for (int i = 0; i < 3; i++) {
                if (g_bias[i] > 0.1f) g_bias[i] = 0.1f;
                if (g_bias[i] < -0.1f) g_bias[i] = -0.1f;
            }
        }
        cx = kp * ex; cy = kp * ey; cz = kp * ez;
    }

    // --- Quaternion integrieren ---
    const float ox = wx + cx, oy = wy + cy, oz = wz + cz;
    const float dqw = 0.5f * (-qx * ox - qy * oy - qz * oz);
    const float dqx = 0.5f * ( qw * ox + qy * oz - qz * oy);
    const float dqy = 0.5f * ( qw * oy - qx * oz + qz * ox);
    const float dqz = 0.5f * ( qw * oz + qx * oy - qy * ox);
    qw += dqw * dt; qx += dqx * dt; qy += dqy * dt; qz += dqz * dt;
    const float n = 1.0f / sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    qw *= n; qx *= n; qy *= n; qz *= n;

    g_roll = atan2f(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
    float sp = 2 * (qw * qy - qz * qx);
    if (sp > 1.0f) sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;
    g_pitch = asinf(sp);
}
/* ATT_END */

static void imu_update(float dt)
{
    uint8_t d[12];
    if (imu_read(0x35, d, sizeof(d)) != ESP_OK) return;
    int16_t r[6];
    for (int i = 0; i < 6; i++) r[i] = (int16_t)(d[2 * i] | (d[2 * i + 1] << 8));

    const float ka = 1.0f / 8192.0f;
    const float kg = DEG2RAD / 64.0f;
    float ax = r[0] * ka, ay = r[1] * ka, az = r[2] * ka;
    float gx = r[3] * kg, gy = r[4] * kg, gz = r[5] * kg;

    att_fuse(MAP_FWD(ax, ay, az), MAP_RIGHT(ax, ay, az), MAP_DOWN(ax, ay, az),
             MAP_FWD(gx, gy, gz), MAP_RIGHT(gx, gy, gz), MAP_DOWN(gx, gy, gz), dt);
}

/* ------------------------------------------------------------------ */
/*  main                                                               */
/* ------------------------------------------------------------------ */
void app_main(void)
{
    for (int i = 0; i < 2; i++) {
        fb[i] = heap_caps_malloc(W * H * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!fb[i]) { ESP_LOGE(TAG, "Kein Speicher für Framebuffer"); return; }
    }
    scale_init();
    lcd_init();
    i2c_devices_init();
    sensors_init(i2c_bus);
    touch_init(i2c_bus);

    // --- gespeicherte Einstellungen laden ---
    settings_init();
    float qnh_saved = sensors_get_qnh_hpa();
    {
        float q;
        if (settings_load_qnh(&q)) {
            sensors_set_qnh_hpa(q);
            qnh_saved = q;
            ESP_LOGI(TAG, "QNH geladen: %.1f hPa", q);
        }
    }

    gpio_config_t btn_cfg = { .pin_bit_mask = 1ULL << PIN_BOOT, .mode = GPIO_MODE_INPUT,
                              .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&btn_cfg);

    tapes_t tp = { 0 };
    cur = fb[0];
    render(0, 0, &tp);
    xSemaphoreTake(tx_done, portMAX_DELAY);
    esp_lcd_panel_draw_bitmap(panel, 0, 0, W, H, cur);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_LCD_BL, 1);

#if COLOR_TEST
    {
        const uint16_t cols[8] = { RGB(255, 0, 0), RGB(0, 255, 0), RGB(0, 0, 255), RGB(255, 255, 255),
                                   RGB(0, 0, 0), RGB(255, 255, 0), RGB(0, 255, 255), RGB(255, 0, 255) };
        const char *lab[8] = { "R", "G", "B", "W", "K", "Y", "C", "M" };
        cur = fb[0];
        for (int i = 0; i < 8; i++) {
            box(i * 40, 0, 40, H, cols[i]);
            draw_text(i * 40 + 12, CY - 10, lab[i], 3, COL_GRAY);
        }
        xSemaphoreTake(tx_done, portMAX_DELAY);
        esp_lcd_panel_draw_bitmap(panel, 0, 0, W, H, cur);
        ESP_LOGI(TAG, "Farbtest: INVERT=%d BGR=%d", LCD_INVERT, LCD_RGB_BGR);
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif

    const bool demo = FORCE_DEMO || !imu_ok;
    float roll_trim = 0, pitch_trim = 0;
    float bias_saved[3] = {0, 0, 0};
    if (settings_load_bias(bias_saved)) {
        for (int i = 0; i < 3; i++) g_bias[i] = bias_saved[i];
        ESP_LOGI(TAG, "Gyro-Bias geladen: %.2f %.2f %.2f Grad/s", bias_saved[0] * RAD2DEG, bias_saved[1] * RAD2DEG, bias_saved[2] * RAD2DEG);
    }
    {
        float r, p;
        if (settings_load_trim(&r, &p)) {
            roll_trim = r;
            pitch_trim = p;
            ESP_LOGI(TAG, "Trimmung geladen: roll %.1f pitch %.1f Grad", r * RAD2DEG, p * RAD2DEG);
        }
    }
    bool last_btn = false, long_done = false, spd_was_ok = false;
    float spd_disp = 0;
    bool menu_open = false, trim_req = false;
    int menu_btn = -1;                       // aktuell gedrückte Menütaste
    int64_t menu_t_last = 0, rep_t0 = 0, rep_t_last = 0, msg_until = 0;
    const char *menu_msg = NULL;
    int64_t press_t0 = 0;
    int64_t last = esp_timer_get_time();
    int64_t t0 = last, fps_t = last;
    int ib = 1, frames = 0;

    while (1) {
        int64_t now = esp_timer_get_time();
        float dt = (now - last) * 1e-6f;
        last = now;
        if (dt > 0.1f) dt = 0.1f;
        float t = (now - t0) * 1e-6f;

        // --- Bedienung ---
        bool btn = (gpio_get_level(PIN_BOOT) == 0);
        if (btn && !last_btn) { press_t0 = now; long_done = false; }
        if (btn && !long_done && (now - press_t0) > 1500000) {
            sensors_calibrate_alt();
            long_done = true;
            ESP_LOGI(TAG, "Hoehenkalibrierung angefordert");
        }
        bool short_release = (!btn && last_btn && !long_done);
        last_btn = btn;
        trim_req = short_release;

        // --- Touch / Menü ---
        touch_event_t te;
        touch_poll(&te);
        if (!menu_open) {
            if (te.pressed_edge) {           // Antippen öffnet das Menü
                menu_open = true;
                menu_btn = -1;
                menu_t_last = now;
                menu_msg = NULL;
            }
        } else {
            if (te.down || te.pressed_edge) menu_t_last = now;
            if (te.pressed_edge) {
                menu_btn = menu_hit(te.x, te.y);
                rep_t0 = rep_t_last = now;
                float q = sensors_get_qnh_hpa();
                switch (menu_btn) {
                case B_M1:  sensors_set_qnh_hpa(roundf((q - 1.0f) * 10.0f) / 10.0f); break;
                case B_P1:  sensors_set_qnh_hpa(roundf((q + 1.0f) * 10.0f) / 10.0f); break;
                case B_M01: sensors_set_qnh_hpa(roundf((q - 0.1f) * 10.0f) / 10.0f); break;
                case B_P01: sensors_set_qnh_hpa(roundf((q + 0.1f) * 10.0f) / 10.0f); break;
                case B_GPS:
                    if (!sensors_qnh_from_gps()) { menu_msg = "KEIN GPS"; msg_until = now + 2000000; }
                    break;
                case B_NULL: trim_req = true; break;
                case B_OK:   menu_open = false; break;
                default: break;
                }
            } else if (te.down && menu_btn >= B_M1 && menu_btn <= B_P01 &&
                       menu_hit(te.x, te.y) == menu_btn &&
                       now - rep_t0 > 500000 && now - rep_t_last > 100000) {
                // Taste gehalten: automatisch wiederholen
                rep_t_last = now;
                float q = sensors_get_qnh_hpa();
                float step = (menu_btn == B_M1) ? -1.0f : (menu_btn == B_P1) ? 1.0f :
                             (menu_btn == B_M01) ? -0.1f : 0.1f;
                sensors_set_qnh_hpa(roundf((q + step) * 10.0f) / 10.0f);
            }
            if (!te.down) menu_btn = -1;
            if (msg_until && now > msg_until) { menu_msg = NULL; msg_until = 0; }
            if (now - menu_t_last > 15000000) menu_open = false;   // 15 s ohne Eingabe: schließen
        }

        // --- QNH speichern, sobald es sich geändert hat und das Menü zu ist ---
        if (!menu_open && fabsf(sensors_get_qnh_hpa() - qnh_saved) > 0.05f) {
            qnh_saved = sensors_get_qnh_hpa();
            settings_save_qnh(qnh_saved);
            ESP_LOGI(TAG, "QNH gespeichert: %.1f hPa", qnh_saved);
        }

        // --- Fluglage ---
        float roll, pitch;
        if (demo) {
            roll = 35.0f * DEG2RAD * sinf(0.7f * t);
            pitch = 15.0f * DEG2RAD * sinf(0.45f * t);
        } else {
            {
                // GPS-Geschwindigkeit und -Beschleunigung für die Korrektur bereitstellen
                static uint32_t last_cnt = 0;
                static int64_t last_t = 0;
                static float last_v = 0;
                g_imu_vok = g_sens.gps_valid;
                g_imu_v = g_sens.gps_speed_kmh / 3.6f;
                if (g_sens.gps_nmea != last_cnt) {
                    last_cnt = g_sens.gps_nmea;
                    float gdt = (now - last_t) * 1e-6f;
                    if (gdt >= 0.5f && gdt < 3.0f) {
                        float a = (g_imu_v - last_v) / gdt;
                        g_imu_vdot += 0.5f * (a - g_imu_vdot);      // glätten
                        last_t = now; last_v = g_imu_v;
                    } else if (gdt >= 3.0f) {
                        g_imu_vdot = 0; last_t = now; last_v = g_imu_v;
                    }
                }
                if (!g_imu_vok) g_imu_vdot = 0;
            }
            imu_update(dt);
            if (g_bias_new) {                       // Start-Kalibrierung erfolgreich
                g_bias_new = false;
                ESP_LOGI(TAG, "Gyro-Bias gemessen: %.2f %.2f %.2f Grad/s", g_bias[0] * RAD2DEG, g_bias[1] * RAD2DEG, g_bias[2] * RAD2DEG);
                if (fabsf(g_bias[0] - bias_saved[0]) > 0.003f || fabsf(g_bias[1] - bias_saved[1]) > 0.003f ||
                    fabsf(g_bias[2] - bias_saved[2]) > 0.003f) {
                    for (int i = 0; i < 3; i++) bias_saved[i] = g_bias[i];
                    settings_save_bias(bias_saved);
                }
            }
            if (trim_req) {
                roll_trim = g_roll;
                pitch_trim = g_pitch;
                settings_save_trim(roll_trim, pitch_trim);
                ESP_LOGI(TAG, "Trimmung gesetzt und gespeichert");
            }
            roll = g_roll - roll_trim;
            pitch = g_pitch - pitch_trim;
        }

        // --- Bänder ---
#if DEMO_SENSORS
        tp.spd_kmh = 110.0f + 30.0f * sinf(0.3f * t);   tp.spd_ok = true;
        tp.alt_m = 1250.0f + 60.0f * sinf(0.2f * t);    tp.alt_ok = true;
        tp.vs = 2.5f * sinf(0.5f * t);                  tp.vs_ok = true;
        tp.gps = true;
        tp.sats = 9;
#else
        tp.spd_ok = g_sens.gps_valid;
        if (tp.spd_ok) {
            float tgt = g_sens.gps_speed_kmh;
            if (!spd_was_ok) spd_disp = tgt;
            else spd_disp += (tgt - spd_disp) * fminf(1.0f, dt * 3.0f);   // GPS kommt nur mit 1 Hz
        }
        spd_was_ok = tp.spd_ok;
        tp.spd_kmh = spd_disp;
        tp.alt_ok = g_sens.baro_ok;
        tp.alt_m = g_sens.alt_m;
        tp.vs_ok = g_sens.baro_ok;
        tp.vs = g_sens.vs_ms;
        tp.gps = g_sens.gps_valid;
        tp.sats = g_sens.gps_sats;
#endif
        tp.slip = demo ? 0.6f * sinf(0.4f * t) : g_slip;
        tp.qnh = sensors_get_qnh_hpa();

        cur = fb[ib];
        render(roll, pitch, &tp);
        if (menu_open) {
            draw_menu(sensors_get_qnh_hpa(), menu_btn, menu_msg, te.down, te.x, te.y);
        }
        xSemaphoreTake(tx_done, portMAX_DELAY);
        esp_lcd_panel_draw_bitmap(panel, 0, 0, W, H, cur);
        ib ^= 1;

        if (++frames >= 60) {
            ESP_LOGI(TAG, "%.1f fps | roll %.1f pitch %.1f (acc %.1f %.1f, trust %.2f) | spd %.0f km/h alt %.1f m vs %.2f m/s | gps %d sats %d | rx %u B, %u msg @%d",
                     frames * 1e6f / (now - fps_t), roll * RAD2DEG, pitch * RAD2DEG,
                     g_roll_a * RAD2DEG, g_pitch_a * RAD2DEG, g_trust,
                     tp.spd_kmh, tp.alt_m, tp.vs, (int)g_sens.gps_valid, (int)g_sens.gps_sats,
                     (unsigned)g_sens.gps_bytes, (unsigned)g_sens.gps_nmea, (int)g_sens.gps_baud);
            frames = 0;
            fps_t = now;
        }
        vTaskDelay(1);
    }
}
