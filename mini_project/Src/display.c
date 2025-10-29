/* Abstraction layer that lets the app print text either to UART or an OLED */
#include "display.h"

#ifndef STM32F411xE
#define STM32F411xE
#endif

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

#ifndef DISPLAY_UART_DEFAULT_BAUD
#define DISPLAY_UART_DEFAULT_BAUD 115200u
#endif

#ifndef HSI_VALUE
#define HSI_VALUE 16000000u
#endif

#ifndef HSE_VALUE
#define HSE_VALUE 8000000u
#endif

#define DISPLAY_UART_INSTANCE USART2
#define DISPLAY_UART_GPIO GPIOA
#define DISPLAY_UART_TX_PIN 2u
#define DISPLAY_UART_RX_PIN 3u
#define DISPLAY_UART_AF     7u

#define DISPLAY_UART_ENABLE_GPIO() (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN)
#define DISPLAY_UART_ENABLE_CLK()  (RCC->APB1ENR |= RCC_APB1ENR_USART2EN)

extern uint32_t SystemCoreClock;

/* Minimal driver interface so we can swap between UART and OLED text backends. */
typedef struct {
    void (*init)(const display_config_t *cfg);
    void (*write)(const char *text);
    void (*vprintf)(const char *fmt, va_list args);
    void (*flush)(void);
} display_driver_vtable_t;

static display_backend_t select_and_init_driver(display_backend_t backend);

static void uart_driver_init(const display_config_t *cfg);
static void uart_driver_write(const char *text);
static void uart_driver_vprintf(const char *fmt, va_list args);
static void uart_driver_flush(void);

static void oled_stub_init(const display_config_t *cfg);
static void oled_stub_write(const char *text);
static void oled_stub_vprintf(const char *fmt, va_list args);
static void oled_stub_flush(void);

static const display_driver_vtable_t uart_driver = {
    .init = uart_driver_init,
    .write = uart_driver_write,
    .vprintf = uart_driver_vprintf,
    .flush = uart_driver_flush
};

static const display_driver_vtable_t oled_stub_driver = {
    .init = oled_stub_init,
    .write = oled_stub_write,
    .vprintf = oled_stub_vprintf,
    .flush = oled_stub_flush
};

static display_config_t active_config = {
    .backend = DISPLAY_BACKEND_UART_STREAM,
    .uart_baud = DISPLAY_UART_DEFAULT_BAUD
};

static const display_driver_vtable_t *active_driver = &uart_driver;
static bool oled_stub_warned = false;
static bool uart_console_ready = false;

/* Convert the desired baud rate into the USART BRR register format. */
static uint16_t uart_brr_from_pclk(uint32_t pclk, uint32_t baud) {
    uint32_t usartdiv_x16 = (pclk + (baud / 2u)) / baud;
    return (uint16_t)usartdiv_x16;
}

static uint32_t rcc_apb_prescaler_decode(uint32_t bits) {
    switch (bits & 0x7u) {
        case 0u:
        case 1u:
        case 2u:
        case 3u: return 1u;
        case 4u: return 2u;
        case 5u: return 4u;
        case 6u: return 8u;
        default: return 16u;
    }
}

static inline uint32_t ahb_prescaler_decode(uint32_t bits) {
    static const uint16_t table[16] = {1u,1u,1u,1u,1u,1u,1u,1u,2u,4u,8u,16u,64u,128u,256u,512u};
    return table[bits & 0xFu];
}

/* Figure out what the current APB1 clock is (it feeds USART2). */
static uint32_t display_rcc_apb1_freq_hz(void) {
    uint32_t pre_bits = (RCC->CFGR >> 10u) & 0x7u;
    uint32_t prescaler = rcc_apb_prescaler_decode(pre_bits);
    if (prescaler == 0u) { prescaler = 1u; }
    return SystemCoreClock / prescaler;
}

/* Busy-wait until the transmitter is ready, then push one byte. */
static inline void uart_putc(char c) {
    while (!(DISPLAY_UART_INSTANCE->SR & USART_SR_TXE)) {
    }
    DISPLAY_UART_INSTANCE->DR = (uint16_t)c;
}

/* Send a complete C-string out of the UART, expanding newlines for terminals. */
static void uart_write_raw(const char *text) {
    if (!text) { return; }
    while (*text) {
        if (*text == '\n') {
            uart_putc('\r');
        }
        uart_putc(*text++);
    }
}

/* Shared printf implementation that formats into a stack buffer. */
static void uart_vprintf_common(const char *fmt, va_list args) {
    char buf[160];
    vsnprintf(buf, sizeof(buf), fmt, args);
    uart_write_raw(buf);
}

/* Configure PA2/PA3 for USART2 alternate function mode. */
static void uart_gpio_config(void) {
    DISPLAY_UART_ENABLE_GPIO();
    
    DISPLAY_UART_GPIO->MODER &= ~((3u << (DISPLAY_UART_TX_PIN * 2u)) | (3u << (DISPLAY_UART_RX_PIN * 2u)));
    DISPLAY_UART_GPIO->MODER |= (2u << (DISPLAY_UART_TX_PIN * 2u)) | (2u << (DISPLAY_UART_RX_PIN * 2u));
    DISPLAY_UART_GPIO->OSPEEDR |= (3u << (DISPLAY_UART_TX_PIN * 2u)) | (3u << (DISPLAY_UART_RX_PIN * 2u));
    DISPLAY_UART_GPIO->PUPDR &= ~((3u << (DISPLAY_UART_TX_PIN * 2u)) | (3u << (DISPLAY_UART_RX_PIN * 2u)));
    DISPLAY_UART_GPIO->PUPDR |= (0u << (DISPLAY_UART_TX_PIN * 2u)) | (1u << (DISPLAY_UART_RX_PIN * 2u));

    if (DISPLAY_UART_TX_PIN < 8u) {
        uint32_t shift = DISPLAY_UART_TX_PIN * 4u;
        DISPLAY_UART_GPIO->AFR[0] = (DISPLAY_UART_GPIO->AFR[0] & ~(0xFu << shift)) | (DISPLAY_UART_AF << shift);
    } else {
        uint32_t shift = (DISPLAY_UART_TX_PIN - 8u) * 4u;
        DISPLAY_UART_GPIO->AFR[1] = (DISPLAY_UART_GPIO->AFR[1] & ~(0xFu << shift)) | (DISPLAY_UART_AF << shift);
    }

    if (DISPLAY_UART_RX_PIN < 8u) {
        uint32_t shift = DISPLAY_UART_RX_PIN * 4u;
        DISPLAY_UART_GPIO->AFR[0] = (DISPLAY_UART_GPIO->AFR[0] & ~(0xFu << shift)) | (DISPLAY_UART_AF << shift);
    } else {
        uint32_t shift = (DISPLAY_UART_RX_PIN - 8u) * 4u;
        DISPLAY_UART_GPIO->AFR[1] = (DISPLAY_UART_GPIO->AFR[1] & ~(0xFu << shift)) | (DISPLAY_UART_AF << shift);
    }
}

/* Reset the USART and set it up for 8N1 transfers. */
static void uart_peripheral_config(uint32_t baud) {
    DISPLAY_UART_ENABLE_CLK();

    DISPLAY_UART_INSTANCE->CR1 = 0;
    DISPLAY_UART_INSTANCE->CR2 = 0;
    DISPLAY_UART_INSTANCE->CR3 = 0;
    DISPLAY_UART_INSTANCE->BRR = uart_brr_from_pclk(display_rcc_apb1_freq_hz(), baud);
    DISPLAY_UART_INSTANCE->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/* Start the UART backend using either the requested baud or a default. */
static void uart_driver_init(const display_config_t *cfg) {
    uint32_t baud = (cfg && cfg->uart_baud) ? cfg->uart_baud : DISPLAY_UART_DEFAULT_BAUD;
    uart_gpio_config();
    uart_peripheral_config(baud);
    uart_console_ready = true;
}

static void uart_driver_write(const char *text) {
    uart_write_raw(text);
}

static void uart_driver_vprintf(const char *fmt, va_list args) {
    uart_vprintf_common(fmt, args);
}

static void uart_driver_flush(void) {
}

/* Until the real OLED text driver lands, we piggy-back on the UART path. */
static void oled_stub_init(const display_config_t *cfg) {
    (void)cfg;
    /* Until the OLED panel driver is implemented, reuse the UART path so logs stay visible. */
    uart_driver_init(&active_config);
}

/* Let the developer know why nothing shows up on the OLED if the driver is missing. */
static void oled_warn_once(void) {
    if (!oled_stub_warned) {
        oled_stub_warned = true;
        uart_write_raw("[display] OLED backend not implemented; using UART fallback.\n");
    }
}

static void oled_stub_write(const char *text) {
    oled_warn_once();
    uart_driver_write(text);
}

static void oled_stub_vprintf(const char *fmt, va_list args) {
    oled_warn_once();
    uart_driver_vprintf(fmt, args);
}

static void oled_stub_flush(void) {
    uart_driver_flush();
}

/* Switch the active backend and run its init routine. */
static display_backend_t select_and_init_driver(display_backend_t backend) {
    display_backend_t resolved = backend;
    switch (backend) {
        case DISPLAY_BACKEND_UART_STREAM:
            active_driver = &uart_driver;
            break;
        case DISPLAY_BACKEND_OLED_TEXT:
            active_driver = &oled_stub_driver;
            break;
        default:
            active_driver = &uart_driver;
            resolved = DISPLAY_BACKEND_UART_STREAM;
            break;
    }

    active_config.backend = resolved;

    if (active_driver && active_driver->init) {
        active_driver->init(&active_config);
    }

    return resolved;
}

/* Call this once during boot to choose a backend and prime the driver. */
void display_init(const display_config_t *config) {
    if (config) {
        active_config = *config;
    } else {
        active_config.backend = DISPLAY_BACKEND_UART_STREAM;
        active_config.uart_baud = DISPLAY_UART_DEFAULT_BAUD;
    }

    (void)select_and_init_driver(active_config.backend);
}

/* Allow runtime backend switching (mainly for experiments on the bench). */
bool display_select_backend(display_backend_t backend) {
    if (backend == active_config.backend) {
        return true;
    }

    return select_and_init_driver(backend) == backend;
}

display_backend_t display_current_backend(void) {
    return active_config.backend;
}

/* Convert the enum into a friendly string for logs. */
const char *display_backend_name(display_backend_t backend) {
    switch (backend) {
        case DISPLAY_BACKEND_UART_STREAM: return "UART";
        case DISPLAY_BACKEND_OLED_TEXT:   return "OLED";
        default:                          return "Unknown";
    }
}

/* Raw write helper: no formatting, just pass text to the active driver. */
void display_write(const char *text) {
    if (active_driver && active_driver->write) {
        active_driver->write(text);
    }
}

/* Same as printf but takes a va_list so display_printf can wrap it. */
void display_vprintf(const char *fmt, va_list args) {
    if (active_driver && active_driver->vprintf) {
        active_driver->vprintf(fmt, args);
    }
}

/* Drop-in replacement for printf that routes through whichever backend is live. */
void display_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    display_vprintf(fmt, args);
    va_end(args);
}

static void ensure_uart_console_ready(void) {
    if (!uart_console_ready) {
        uart_driver_init(&active_config);
    }
}

void display_uart_write(const char *text) {
    ensure_uart_console_ready();
    uart_write_raw(text);
}

void display_uart_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    ensure_uart_console_ready();
    uart_vprintf_common(fmt, args);
    va_end(args);
}

void display_flush(void) {
    if (active_driver && active_driver->flush) {
        active_driver->flush();
    }
}
