#pragma once
#include <stdbool.h>

// Dauerhafte Einstellungen im NVS (Flash). Schreibt nur bei Änderungen.
void settings_init(void);

bool settings_load_qnh(float *qnh_hpa);                       // false = nichts gespeichert
bool settings_load_trim(float *roll_rad, float *pitch_rad);   // false = nichts gespeichert

void settings_save_qnh(float qnh_hpa);
void settings_save_trim(float roll_rad, float pitch_rad);
