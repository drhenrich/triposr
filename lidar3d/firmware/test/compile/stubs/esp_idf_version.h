// Attrappe von esp_idf_version.h.
//
// Die Fassung laesst sich beim Uebersetzungstest vorgeben (-DSTUB_IDF_MAJOR),
// damit beide Zweige der Versionsweichen gebaut werden. Genau hier ist einmal
// etwas durchgerutscht: UART_SCLK_DEFAULT gibt es erst ab IDF 5, und der
// Fehler fiel erst beim echten Build auf einem Core mit IDF 4 auf - beim
// Benutzer, mit der Hardware auf dem Tisch.
#pragma once

#ifndef STUB_IDF_MAJOR
#define STUB_IDF_MAJOR 4
#endif

#define ESP_IDF_VERSION_VAL(major, minor, patch) \
  ((major) * 10000 + (minor) * 100 + (patch))
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(STUB_IDF_MAJOR, 0, 0)
