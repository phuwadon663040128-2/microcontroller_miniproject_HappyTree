#ifndef TELEMETRY_UART_H
#define TELEMETRY_UART_H

#include <stdbool.h>
#include <stdint.h>

#ifndef TELEMETRY_UART_ENABLE
#define TELEMETRY_UART_ENABLE 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *state;
    float soil_moisture_norm;
    float temp_norm;
    float ldr_norm;
    float threshold_low;
    float threshold_high;
    uint32_t below_low_ticks;
    bool alert_active;
    uint32_t uptime_ms;
} telemetry_sample_t;

void telemetry_uart_init(uint32_t baud);
bool telemetry_uart_is_ready(void);
bool telemetry_uart_send_frame(const char *frame);
bool telemetry_uart_publish(const telemetry_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_UART_H */
