#pragma once
#include <stdbool.h>
#include "driver/i2c_master.h"

typedef struct {
    bool down;              // Finger liegt gerade auf
    int  x, y;              // Position in Bildschirmkoordinaten (320x172, Querformat)
    bool pressed_edge;      // genau ein Aufruf lang true: Finger gerade aufgesetzt
    bool released_edge;     // genau ein Aufruf lang true: Finger gerade abgehoben
} touch_event_t;

// Reset, Erkennung und Interrupt des AXS5106L. false = nicht gefunden.
bool touch_init(i2c_master_bus_handle_t bus);

// Einmal pro Bild aufrufen.
void touch_poll(touch_event_t *ev);
