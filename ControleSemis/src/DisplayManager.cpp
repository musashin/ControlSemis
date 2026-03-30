#include "DisplayManager.h"

void DisplayManager::begin() {
    _matrix.begin();
}

void DisplayManager::show(ControllerState state) {
    switch (state) {
        case ControllerState::HEALTHY:
            showStatic("OK");
            break;
        case ControllerState::SENSOR_ERROR:
            showScrolling("SENS");
            break;
        case ControllerState::COMM_FAULT:
            showScrolling("COMMS ISSUE");
            break;
    }
}

// Static display — non-blocking. "OK" fits in the 12×8 matrix with Font_5x7.
void DisplayManager::showStatic(const char* text) {
    _matrix.beginDraw();
    _matrix.stroke(0xFFFFFFFF);
    _matrix.textFont(Font_5x7);
    _matrix.beginText(0, 1, 0xFFFFFFFF);
    _matrix.print(text);
    _matrix.endText();
    _matrix.endDraw();
}

// Scrolling display — blocks until the full message has scrolled off screen.
// Acceptable for infrequent (~30 s) refresh ticks.
void DisplayManager::showScrolling(const char* text) {
    _matrix.beginDraw();
    _matrix.stroke(0xFFFFFFFF);
    _matrix.textScrollSpeed(80);
    _matrix.textFont(Font_5x7);
    _matrix.beginText(0, 1, 0xFFFFFFFF);
    _matrix.print(text);
    _matrix.endText(SCROLL_LEFT);
    _matrix.endDraw();
}
