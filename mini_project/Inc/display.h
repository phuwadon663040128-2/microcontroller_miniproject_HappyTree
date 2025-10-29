#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DISPLAY_BACKEND_UART_STREAM = 0, /**< Legacy UART console streaming. */
    DISPLAY_BACKEND_OLED_TEXT  = 1   /**< Reserved for upcoming OLED driver (currently UART fallback). */
} display_backend_t;

typedef struct {
    display_backend_t backend;
    uint32_t uart_baud;
} display_config_t;

void display_init(const display_config_t *config);

/* Attempt to switch the active backend at runtime. Returns false if the request fell back to UART. */
bool display_select_backend(display_backend_t backend);
display_backend_t display_current_backend(void);
const char *display_backend_name(display_backend_t backend);

/* Stream-style helpers retained for UART compatibility; OLED driver can map these to buffered text. */
void display_write(const char *text);
void display_printf(const char *fmt, ...);
void display_vprintf(const char *fmt, va_list args);
void display_uart_write(const char *text);
void display_uart_printf(const char *fmt, ...);
void display_flush(void);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_H */
