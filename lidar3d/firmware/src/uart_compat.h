// Was sich am UART-Treiber zwischen den ESP-IDF-Fassungen geaendert hat.
//
// Die Taktquelle heisst nicht ueberall gleich:
//
//   IDF 4.x (Arduino-Core 2.x)   uart_sclk_t { UART_SCLK_APB, UART_SCLK_RTC,
//                                              UART_SCLK_XTAL, ... }
//   IDF 5.x (Arduino-Core 3.x)   zusaetzlich UART_SCLK_DEFAULT
//
// UART_SCLK_DEFAULT gibt es also erst ab IDF 5. Wer mit einem aelteren Core
// baut, bekommt sonst
//
//     error: 'UART_SCLK_DEFAULT' was not declared in this scope
//
// und zwar an jeder Stelle, die eine UART aufmacht. Deshalb steht die
// Entscheidung hier an einer Stelle und nicht verstreut im Code.
//
// APB ist auf IDF 4 die richtige Wahl: die UART haengt dort ohnehin am
// APB-Takt, UART_SCLK_DEFAULT waehlt auf IDF 5 genau denselben.
#pragma once

#include <driver/uart.h>
#include <esp_idf_version.h>

namespace nwl {

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static const uart_sclk_t kUartSourceClk = UART_SCLK_DEFAULT;
#else
static const uart_sclk_t kUartSourceClk = UART_SCLK_APB;
#endif

}  // namespace nwl
