#pragma once
/*
 * Zentrale Konfiguration. Hier stehen alle Einstellungen, die von Ihrer
 * Hardware und Ihrem Aufbau abhängen.
 *
 * >>> Tragen Sie in den Abschnitten "Display-Ausrichtung" und "IMU-Achsen"
 * >>> die Werte ein, die bei Ihnen schon funktionieren.
 */

/* ===== Display (SPI) ===== */
#define PIN_LCD_MOSI   2
#define PIN_LCD_SCLK   1
#define PIN_LCD_CS     14
#define PIN_LCD_DC     15
#define PIN_LCD_RST    22
#define PIN_LCD_BL     23
#define LCD_HOST       SPI2_HOST
#define LCD_PCLK_HZ    (40 * 1000 * 1000)

/* ===== Display-Ausrichtung (hier Ihre funktionierenden Werte eintragen) ===== */
#define LCD_MIRROR_X   1
#define LCD_MIRROR_Y   1
#define LCD_GAP_X      0
#define LCD_GAP_Y      34
#define LCD_INVERT     0
#define LCD_RGB_BGR    1

/* ===== I2C (IMU, Touch, BMP280) ===== */
#define PIN_I2C_SDA    18
#define PIN_I2C_SCL    19
#define PIN_TP_RST     20
#define PIN_BOOT       9
#define IMU_ADDR_A     0x6B
#define IMU_ADDR_B     0x6A
#define TOUCH_ADDR     0x63
#define ENABLE_TOUCH_TRIM 1

/* ===== IMU-Achsen (hier Ihre funktionierenden Werte eintragen) ===== */
#define MAP_FWD(x, y, z)    ( (z))
#define MAP_RIGHT(x, y, z)  (-(y))
#define MAP_DOWN(x, y, z)   ( (x))

/* ===== GPS (NMEA über UART, nur Empfang) ===== */
// GPS-TX-Leitung wird an diesen ESP-Pin angeschlossen. Bitte prüfen, ob der
// Pin an Ihrer Stiftleiste herausgeführt und frei ist.
#define GPS_RX_PIN     7
#define GPS_BAUD       115200
#define GPS_UART       UART_NUM_1

/* ===== BMP280 / Variometer ===== */
#define BARO_PERIOD_MS 30
#define VARIO_ALPHA    0.15f     // Höhenfilter: größer = schneller, aber unruhiger
#define VARIO_BETA     0.0118f   // Steigratenfilter

/* ===== Einheiten ===== */
#define SPEED_KT       0         // 0 = km/h, 1 = Knoten
#define ALT_FT         0         // 0 = Meter, 1 = Fuß (Variometer bleibt in m/s)

#if SPEED_KT
#define SPD_MINOR      5         // kleine Teilstriche alle 5 kt, Zahlen alle 10 kt
#else
#define SPD_MINOR      10        // alle 10 km/h, Zahlen alle 20 km/h
#endif
#if ALT_FT
#define ALT_MINOR      50
#else
#define ALT_MINOR      10
#endif

/* ===== Betriebsarten ===== */
#define FORCE_DEMO     0         // 1 = simulierte Fluglage (kein IMU nötig)
#define DEMO_SENSORS   0         // 1 = simulierte Speed/Höhe/Vario für die Bänder

/* ===== Layout ===== */
#define W              320
#define H              172
#define CX             (W / 2)
#define CY             (H / 2)
#define SPD_W          44
#define ALT_W          44
#define VS_W           14
#define ALT_X          (W - VS_W - ALT_W)
#define VS_X           (W - VS_W)
#define PPD            2.8f
#define ROLL_R         78
#define GPS_DEBUG 1
#define GPS_AUTOBAUD 0
