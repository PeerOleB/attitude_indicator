#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"

typedef struct {
    // Barometer (BMP280)
    volatile bool  baro_ok;
    volatile float pressure_pa;
    volatile float temp_c;
    volatile float alt_m;        // Höhe gegenüber der QNH-Referenz, gefiltert
    volatile float vs_ms;        // Steigen (+) / Sinken (-) in m/s, gefiltert
    // GPS (NMEA)
    volatile bool  gps_valid;    // gültige Position und Geschwindigkeit
    volatile bool  gps_alt_valid;
    volatile float gps_speed_kmh;  // Geschwindigkeit über Grund
    volatile float gps_track_deg;
    volatile float gps_alt_m;      // Höhe über NN laut GPS
    volatile int   gps_sats;
    // GPS-Diagnose
    volatile uint32_t gps_bytes;   // empfangene Bytes am UART
    volatile uint32_t gps_nmea;    // gültige Nachrichten (NMEA oder UBX, Prüfsumme stimmt)
    volatile int      gps_baud;    // aktuelle Baudrate
} sensor_data_t;

extern sensor_data_t g_sens;

// Startet BMP280-Auslesung (am gemeinsamen I2C-Bus) und GPS-Empfang (UART).
void sensors_init(i2c_master_bus_handle_t bus);

// Höhe neu kalibrieren: mit GPS-Fix auf GPS-Höhe, sonst auf 0 m.
void sensors_calibrate_alt(void);

// QNH (Luftdruck auf Meereshöhe) in hPa lesen / setzen.
float sensors_get_qnh_hpa(void);
void  sensors_set_qnh_hpa(float hpa);
// QNH so setzen, dass die Höhe der GPS-Höhe entspricht. false = kein Baro/GPS-Fix.
bool  sensors_qnh_from_gps(void);
