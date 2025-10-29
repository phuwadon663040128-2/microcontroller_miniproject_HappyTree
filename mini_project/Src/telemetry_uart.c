#include "telemetry_uart.h"

#if TELEMETRY_UART_ENABLE

#ifndef STM32F411xE
#define STM32F411xE
#define STM32F411xE_LOCAL_DEFINE
#endif
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#ifdef STM32F411xE_LOCAL_DEFINE
#undef STM32F411xE
#undef STM32F411xE_LOCAL_DEFINE
#endif

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef TELEMETRY_UART_DEFAULT_BAUD
#define TELEMETRY_UART_DEFAULT_BAUD 115200u
#endif

#ifndef TELEMETRY_UART_ACK_TIMEOUT_MS
#define TELEMETRY_UART_ACK_TIMEOUT_MS 2000u
#endif

#ifndef TELEMETRY_UART_MAX_RETRIES
#define TELEMETRY_UART_MAX_RETRIES 3u
#endif

#ifndef TELEMETRY_UART_RETRY_BACKOFF_MS
#define TELEMETRY_UART_RETRY_BACKOFF_MS 10u
#endif

#ifndef HSI_VALUE
#define HSI_VALUE 16000000u
#endif

#ifndef HSE_VALUE
#define HSE_VALUE 8000000u
#endif

#define TELEMETRY_UART_INSTANCE USART6
#define TELEMETRY_UART_GPIO GPIOA
#define TELEMETRY_UART_TX_PIN 11u
#define TELEMETRY_UART_RX_PIN 12u
#define TELEMETRY_UART_AF     8u

#define TELEMETRY_UART_ENABLE_GPIO() (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN)
#define TELEMETRY_UART_ENABLE_CLK()  (RCC->APB2ENR |= RCC_APB2ENR_USART6EN)

extern volatile uint32_t g_ms;
extern uint32_t SystemCoreClock;

static bool s_ready = false;

typedef enum
{
    UART_ACK_TIMEOUT = 0,
    UART_ACK_OK,
    UART_ACK_ERR
} uart_ack_result_t;

static uint16_t uart_brr_from_pclk(uint32_t pclk, uint32_t baud)
{
    uint32_t usartdiv_x16 = (pclk + (baud / 2u)) / baud;
    return (uint16_t)usartdiv_x16;
}

static uint32_t rcc_apb_prescaler_decode(uint32_t bits)
{
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

static uint32_t telemetry_rcc_apb2_freq_hz(void)
{
    uint32_t pre_bits = (RCC->CFGR >> 13u) & 0x7u;
    uint32_t prescaler = rcc_apb_prescaler_decode(pre_bits);
    if (prescaler == 0u) {
        prescaler = 1u;
    }
    return SystemCoreClock / prescaler;
}

static inline void uart_flush_rx(void)
{
    while (TELEMETRY_UART_INSTANCE->SR & USART_SR_RXNE) {
        (void)TELEMETRY_UART_INSTANCE->DR;
    }
}

static void uart_gpio_config(void)
{
    TELEMETRY_UART_ENABLE_GPIO();

    TELEMETRY_UART_GPIO->MODER &= ~((3u << (TELEMETRY_UART_TX_PIN * 2u)) |
                                    (3u << (TELEMETRY_UART_RX_PIN * 2u)));
    TELEMETRY_UART_GPIO->MODER |=  (2u << (TELEMETRY_UART_TX_PIN * 2u)) |
                                   (2u << (TELEMETRY_UART_RX_PIN * 2u));

    TELEMETRY_UART_GPIO->OSPEEDR |= (3u << (TELEMETRY_UART_TX_PIN * 2u)) |
                                    (3u << (TELEMETRY_UART_RX_PIN * 2u));

    TELEMETRY_UART_GPIO->PUPDR &= ~((3u << (TELEMETRY_UART_TX_PIN * 2u)) |
                                    (3u << (TELEMETRY_UART_RX_PIN * 2u)));
    TELEMETRY_UART_GPIO->PUPDR |=  (0u << (TELEMETRY_UART_TX_PIN * 2u)) |
                                   (1u << (TELEMETRY_UART_RX_PIN * 2u));

    if (TELEMETRY_UART_TX_PIN < 8u) {
        uint32_t shift = TELEMETRY_UART_TX_PIN * 4u;
        TELEMETRY_UART_GPIO->AFR[0] = (TELEMETRY_UART_GPIO->AFR[0] & ~(0xFu << shift)) |
                                      (TELEMETRY_UART_AF << shift);
    } else {
        uint32_t shift = (TELEMETRY_UART_TX_PIN - 8u) * 4u;
        TELEMETRY_UART_GPIO->AFR[1] = (TELEMETRY_UART_GPIO->AFR[1] & ~(0xFu << shift)) |
                                      (TELEMETRY_UART_AF << shift);
    }

    if (TELEMETRY_UART_RX_PIN < 8u) {
        uint32_t shift = TELEMETRY_UART_RX_PIN * 4u;
        TELEMETRY_UART_GPIO->AFR[0] = (TELEMETRY_UART_GPIO->AFR[0] & ~(0xFu << shift)) |
                                      (TELEMETRY_UART_AF << shift);
    } else {
        uint32_t shift = (TELEMETRY_UART_RX_PIN - 8u) * 4u;
        TELEMETRY_UART_GPIO->AFR[1] = (TELEMETRY_UART_GPIO->AFR[1] & ~(0xFu << shift)) |
                                      (TELEMETRY_UART_AF << shift);
    }
}

static void uart_peripheral_config(uint32_t baud)
{
    TELEMETRY_UART_ENABLE_CLK();

    TELEMETRY_UART_INSTANCE->CR1 = 0u;
    TELEMETRY_UART_INSTANCE->CR2 = 0u;
    TELEMETRY_UART_INSTANCE->CR3 = 0u;
    TELEMETRY_UART_INSTANCE->BRR = uart_brr_from_pclk(telemetry_rcc_apb2_freq_hz(), baud);
    TELEMETRY_UART_INSTANCE->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static inline void uart_putc(char c)
{
    while (!(TELEMETRY_UART_INSTANCE->SR & USART_SR_TXE)) {
    }
    TELEMETRY_UART_INSTANCE->DR = (uint16_t)c;
}

static void telemetry_delay_ms(uint32_t delay_ms)
{
    if (delay_ms == 0u)
    {
        return;
    }
    uint32_t start = g_ms;
    while ((g_ms - start) < delay_ms)
    {
        __NOP();
    }
}

static uart_ack_result_t uart_wait_ack(uint32_t timeout_ms)
{
    const uint32_t start = g_ms;
    char buf[4];
    size_t idx = 0u;

    memset(buf, 0, sizeof(buf));

    while ((g_ms - start) < timeout_ms) {
        if (TELEMETRY_UART_INSTANCE->SR & USART_SR_RXNE) {
            char c = (char)(TELEMETRY_UART_INSTANCE->DR & 0xFFu);
            if (c == '\r' || c == '\n') {
                if (idx == 2u && buf[0] == 'O' && buf[1] == 'K') {
                    return UART_ACK_OK;
                }
                if (idx == 3u && buf[0] == 'E' && buf[1] == 'R' && buf[2] == 'R') {
                    return UART_ACK_ERR;
                }
                idx = 0u;
            } else if (idx < (sizeof buf) - 1u) {
                buf[idx++] = c;
                buf[idx] = '\0';
            }
        }
    }

    return UART_ACK_TIMEOUT;
}

void telemetry_uart_init(uint32_t baud)
{
    uint32_t effective_baud = baud ? baud : TELEMETRY_UART_DEFAULT_BAUD;
    uart_gpio_config();
    uart_peripheral_config(effective_baud);
    uart_flush_rx();
    s_ready = true;
}

bool telemetry_uart_is_ready(void)
{
    return s_ready;
}

bool telemetry_uart_send_frame(const char *frame)
{
    if (!s_ready || frame == NULL) {
        return false;
    }

    for (uint32_t attempt = 0u; attempt < TELEMETRY_UART_MAX_RETRIES; ++attempt)
    {
        uart_flush_rx();

        const char *ptr = frame;
        while (*ptr)
        {
            if (*ptr == '\n')
            {
                uart_putc('\r');
            }
            uart_putc(*ptr++);
        }
        uart_putc('\n');

        uart_ack_result_t ack = uart_wait_ack(TELEMETRY_UART_ACK_TIMEOUT_MS);
        if (ack == UART_ACK_OK)
        {
            return true;
        }

        if (ack == UART_ACK_ERR)
        {
            break;
        }

        telemetry_delay_ms(TELEMETRY_UART_RETRY_BACKOFF_MS * (attempt + 1u));
    }

    return false;
}

bool telemetry_uart_publish(const telemetry_sample_t *sample)
{
    if (!s_ready || sample == NULL) {
        return false;
    }

    char frame[256];

    /* scheme:
    {
        "device_id":"<id>",
        "state":"<state>",
        "uptime_ms":<ms>,
    "soil_moisture_norm":<float>,
        "temp_norm":<float>,
        "ldr_norm":<float>,
        "threshold_low":<float>,
        "threshold_high":<float>,
        "below_low_ticks":<ticks>,
        "alert_active":<true|false>
    }
    */
    int written = snprintf(
        frame,
        sizeof(frame),
        "{\"device_id\":\"stm32-node-1\",\"state\":\"%s\","
            "\"uptime_ms\":%lu,\"soil_moisture_norm\":%.3f,"
        "\"temp_norm\":%.3f,\"ldr_norm\":%.3f,\"threshold_low\":%.3f,"
        "\"threshold_high\":%.3f,\"below_low_ticks\":%lu,\"alert_active\":%s}",
        sample->state ? sample->state : "unknown",
        (unsigned long)sample->uptime_ms,
            (double)sample->soil_moisture_norm,
        (double)sample->temp_norm,
        (double)sample->ldr_norm,
        (double)sample->threshold_low,
        (double)sample->threshold_high,
        (unsigned long)sample->below_low_ticks,
        sample->alert_active ? "true" : "false");

    if (written <= 0 || written >= (int)sizeof(frame)) {
        return false;
    }

    return telemetry_uart_send_frame(frame);
}

#else /* TELEMETRY_UART_ENABLE == 0 */

void telemetry_uart_init(uint32_t baud)
{
    (void)baud;
}

bool telemetry_uart_is_ready(void)
{
    return false;
}

bool telemetry_uart_send_frame(const char *frame)
{
    (void)frame;
    return false;
}

bool telemetry_uart_publish(const telemetry_sample_t *sample)
{
    (void)sample;
    return false;
}

#endif /* TELEMETRY_UART_ENABLE */
