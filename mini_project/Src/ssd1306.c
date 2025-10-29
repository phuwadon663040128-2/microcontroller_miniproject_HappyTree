/* Thin glue layer between the STM32 and the u8g2 SSD1306 graphics library. */
#include "ssd1306.h"

#ifndef STM32F411xE
#define STM32F411xE
#endif
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#include "../../Library/csrc/u8g2.h"
#include "../../Library/csrc/u8x8.h"

#include "display.h"

#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifndef SSD1306_DEBUG_FLUSH
#define SSD1306_DEBUG_FLUSH 0
#endif

#define SSD1306_I2C_ADDR            0x3Cu
#define SSD1306_GPIO                GPIOB
#define SSD1306_SCL_PIN             8u
#define SSD1306_SDA_PIN             9u

#define SSD1306_GPIO_RESET_OFFSET   16u
#define SSD1306_GPIO_MODE_BITS      2u
#define SSD1306_GPIO_MODE_MASK      0x3u
#define SSD1306_GPIO_MODE_OUTPUT    0x1u
#define SSD1306_GPIO_SPEED_HIGH     0x3u
#define SSD1306_GPIO_PULL_NONE      0x0u
#define SSD1306_GPIO_PULL_UP        0x1u

#define SSD1306_MICROS_PER_SECOND        1000000u
#define SSD1306_MICROS_PER_MILLISECOND   1000u
#define SSD1306_DELAY_DIVISOR            8u
#define SSD1306_DELAY_10US_FACTOR        5u
#define SSD1306_DELAY_I2C_FACTOR         2u

#define SSD1306_U8X8_SUCCESS        1u
#define SSD1306_DEFAULT_FONT        u8g2_font_6x10_tr
#define SSD1306_DEFAULT_DRAW_COLOR  SSD1306_COLOR_ON

#define SSD1306_GPIO_FIELD_SHIFT(pin)    ((pin) * SSD1306_GPIO_MODE_BITS)
#define SSD1306_GPIO_FIELD_MASK(pin)     (SSD1306_GPIO_MODE_MASK << SSD1306_GPIO_FIELD_SHIFT(pin))
#define SSD1306_GPIO_OUTPUT_MODE(pin)    (SSD1306_GPIO_MODE_OUTPUT << SSD1306_GPIO_FIELD_SHIFT(pin))
#define SSD1306_GPIO_SPEED_HIGH_BITS(pin) (SSD1306_GPIO_SPEED_HIGH << SSD1306_GPIO_FIELD_SHIFT(pin))
#define SSD1306_GPIO_PULLUP_BITS(pin)    (SSD1306_GPIO_PULL_UP << SSD1306_GPIO_FIELD_SHIFT(pin))

#define SSD1306_GPIO_SET_BIT(pin)        (1u << (pin))
#define SSD1306_GPIO_RESET_BIT(pin)      (1u << ((pin) + SSD1306_GPIO_RESET_OFFSET))

#define SSD1306_ENABLE_GPIO_CLOCK()      do { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; (void)RCC->AHB1ENR; } while (0)

static u8g2_t g_displayContext;
static bool g_isDisplayReady = false;
static uint16_t g_i2c_bytes_in_transfer = 0u;

extern const uint8_t u8g2_font_4x6_tr[];
extern const uint8_t u8g2_font_6x10_tr[];
extern uint32_t SystemCoreClock;
extern void iwdg_kick(void);

static void delay_cycles(uint32_t cycles) {
    while (cycles--) {
        __NOP();
    }
}

/* Coarse microsecond delay based on CPU cycles (good enough for the OLED). */
static void delay_microseconds(uint32_t microseconds) {
    if (microseconds == 0u) {
        return;
    }
    uint32_t cycles_per_us = SystemCoreClock / SSD1306_MICROS_PER_SECOND;
    if (cycles_per_us == 0u) {
        cycles_per_us = 1u;
    }
    cycles_per_us /= SSD1306_DELAY_DIVISOR;
    if (cycles_per_us == 0u) {
        cycles_per_us = 1u;
    }
    while (microseconds--) {
        delay_cycles(cycles_per_us);
    }
}

static void delay_milliseconds(uint32_t milliseconds) {
    while (milliseconds--) {
        delay_microseconds(SSD1306_MICROS_PER_MILLISECOND);
    }
}

static const uint8_t s_apb_presc_table[8] = {1u, 1u, 1u, 1u, 2u, 4u, 8u, 16u};

static uint32_t get_apb1_clock_hz(void) {
    uint32_t hclk_hz = SystemCoreClock;
    if (hclk_hz == 0u) {
        hclk_hz = 1000000u;
    }
    uint32_t presc_index = (RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x7u;
    uint32_t prescaler = s_apb_presc_table[presc_index];
    if (prescaler == 0u) {
        prescaler = 1u;
    }
    return hclk_hz / prescaler;
}

static void configure_i2c1_pins(void) {
    SSD1306_ENABLE_GPIO_CLOCK();

    GPIOB->MODER &= ~((uint32_t)(0x3u << (SSD1306_SCL_PIN * 2u)) |
                       (uint32_t)(0x3u << (SSD1306_SDA_PIN * 2u)));
    GPIOB->MODER |= (uint32_t)(0x2u << (SSD1306_SCL_PIN * 2u)) |
                    (uint32_t)(0x2u << (SSD1306_SDA_PIN * 2u));

    GPIOB->OTYPER |= (uint32_t)(1u << SSD1306_SCL_PIN) |
                     (uint32_t)(1u << SSD1306_SDA_PIN);

    GPIOB->OSPEEDR |= (uint32_t)(0x3u << (SSD1306_SCL_PIN * 2u)) |
                      (uint32_t)(0x3u << (SSD1306_SDA_PIN * 2u));

    GPIOB->PUPDR &= ~((uint32_t)(0x3u << (SSD1306_SCL_PIN * 2u)) |
                      (uint32_t)(0x3u << (SSD1306_SDA_PIN * 2u)));
    GPIOB->PUPDR |= (uint32_t)(0x1u << (SSD1306_SCL_PIN * 2u)) |
                    (uint32_t)(0x1u << (SSD1306_SDA_PIN * 2u));

    uint32_t scl_shift = (uint32_t)((SSD1306_SCL_PIN % 8u) * 4u);
    uint32_t sda_shift = (uint32_t)((SSD1306_SDA_PIN % 8u) * 4u);
    uint32_t afr_index = SSD1306_SCL_PIN / 8u;
    GPIOB->AFR[afr_index] &= ~(((uint32_t)0xFu << scl_shift) | ((uint32_t)0xFu << sda_shift));
    GPIOB->AFR[afr_index] |= ((uint32_t)0x4u << scl_shift) | ((uint32_t)0x4u << sda_shift);
}

static void i2c1_disable(void) {
    I2C1->CR1 &= (uint16_t)(~I2C_CR1_PE);
}

static void i2c1_init_400k(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    (void)RCC->APB1ENR;

    configure_i2c1_pins();

    i2c1_disable();
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0u;

    uint32_t pclk1_hz = get_apb1_clock_hz();
    if (pclk1_hz == 0u) {
        pclk1_hz = 42000000u;
    }
    uint32_t freq_mhz = pclk1_hz / 1000000u;
    if (freq_mhz < 2u) {
        freq_mhz = 2u;
    }
    if (freq_mhz > 42u) {
        freq_mhz = 42u;
    }
    I2C1->CR2 = (uint16_t)freq_mhz;

    uint32_t denom = 3u * 400000u;
    uint32_t ccr = (uint32_t)((pclk1_hz + (denom / 2u)) / denom);
    if (ccr < 1u) {
        ccr = 1u;
    }
    if (ccr > 0xFFFu) {
        ccr = 0xFFFu;
    }
    I2C1->CCR = (uint16_t)(I2C_CCR_FS | ccr);

    uint32_t trise = ((freq_mhz * 300u) + 999u) / 1000u + 1u;
    if (trise < 1u) {
        trise = 1u;
    }
    if (trise > 63u) {
        trise = 63u;
    }
    I2C1->TRISE = (uint16_t)trise;

    I2C1->CR1 = I2C_CR1_PE;
}

static bool i2c1_wait_flag(uint32_t flag) {
    uint32_t guard = 100000u;
    while ((I2C1->SR1 & flag) == 0u) {
        if ((I2C1->SR1 & I2C_SR1_AF) != 0u) {
            I2C1->SR1 &= (uint16_t)(~I2C_SR1_AF);
            return false;
        }
        if ((guard & 0x1Fu) == 0u) {
            iwdg_kick();
        }
        if (--guard == 0u) {
            return false;
        }
    }
    return true;
}

static bool i2c1_wait_not_busy(void) {
    uint32_t guard = 100000u;
    while ((I2C1->SR2 & I2C_SR2_BUSY) != 0u) {
        if ((guard & 0x1Fu) == 0u) {
            iwdg_kick();
        }
        if (--guard == 0u) {
            return false;
        }
    }
    return true;
}

static uint8_t u8x8_byte_i2c1_hw(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    (void)u8x8;
    switch (msg) {
        case U8X8_MSG_BYTE_INIT:
            i2c1_init_400k();
            return SSD1306_U8X8_SUCCESS;
        case U8X8_MSG_BYTE_START_TRANSFER: {
            if (!i2c1_wait_not_busy()) {
                return 0u;
            }
            uint8_t addr7 = (uint8_t)(u8x8_GetI2CAddress(u8x8) >> 1);
            I2C1->CR1 |= I2C_CR1_START;
            if (!i2c1_wait_flag(I2C_SR1_SB)) {
                return 0u;
            }
            (void)I2C1->SR1;
            I2C1->DR = (uint8_t)((addr7 << 1) | 0u);
            if (!i2c1_wait_flag(I2C_SR1_ADDR)) {
                I2C1->CR1 |= I2C_CR1_STOP;
                return 0u;
            }
            (void)I2C1->SR1;
            (void)I2C1->SR2;
            g_i2c_bytes_in_transfer = 0u;
            iwdg_kick();
            return SSD1306_U8X8_SUCCESS;
        }
        case U8X8_MSG_BYTE_SEND: {
            uint8_t *data = (uint8_t *)arg_ptr;
            while (arg_int-- > 0) {
                I2C1->DR = *data++;
                if (!i2c1_wait_flag(I2C_SR1_TXE)) {
                    I2C1->CR1 |= I2C_CR1_STOP;
                    return 0u;
                }
                g_i2c_bytes_in_transfer++;
                if (g_i2c_bytes_in_transfer >= 64u) {
                    iwdg_kick();
                    g_i2c_bytes_in_transfer = 0u;
                }
            }
            return SSD1306_U8X8_SUCCESS;
        }
        case U8X8_MSG_BYTE_END_TRANSFER:
            if (!i2c1_wait_flag(I2C_SR1_TXE)) {
                I2C1->CR1 |= I2C_CR1_STOP;
                return 0u;
            }
            if (g_i2c_bytes_in_transfer > 0u) {
                (void)i2c1_wait_flag(I2C_SR1_BTF);
            }
            I2C1->CR1 |= I2C_CR1_STOP;
            g_i2c_bytes_in_transfer = 0u;
            iwdg_kick();
            return SSD1306_U8X8_SUCCESS;
        case U8X8_MSG_BYTE_SET_DC:
            return SSD1306_U8X8_SUCCESS;
        default:
            return 0u;
    }
}

/* Callback that u8g2 calls when it needs GPIO or timing services from us. */
static uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    (void)u8x8;
    (void)arg_ptr;

    switch (msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            return SSD1306_U8X8_SUCCESS;
        case U8X8_MSG_DELAY_MILLI:
            delay_milliseconds(arg_int);
            return SSD1306_U8X8_SUCCESS;
        case U8X8_MSG_DELAY_10MICRO:
            delay_microseconds((uint32_t)arg_int * SSD1306_DELAY_10US_FACTOR);
            return SSD1306_U8X8_SUCCESS;
        case U8X8_MSG_DELAY_100NANO:
        case U8X8_MSG_DELAY_NANO:
            return SSD1306_U8X8_SUCCESS;
        case U8X8_MSG_DELAY_I2C:
            delay_microseconds((uint32_t)arg_int * SSD1306_DELAY_I2C_FACTOR);
            return SSD1306_U8X8_SUCCESS;
        case U8X8_MSG_GPIO_I2C_CLOCK:
            return SSD1306_U8X8_SUCCESS;
        case U8X8_MSG_GPIO_I2C_DATA:
            return SSD1306_U8X8_SUCCESS;
        case U8X8_MSG_GPIO_RESET:
        case U8X8_MSG_GPIO_DC:
        case U8X8_MSG_GPIO_CS:
            return SSD1306_U8X8_SUCCESS;
        case U8X8_MSG_GPIO_MENU_SELECT:
        case U8X8_MSG_GPIO_MENU_NEXT:
        case U8X8_MSG_GPIO_MENU_PREV:
        case U8X8_MSG_GPIO_MENU_HOME:
        case U8X8_MSG_GPIO_MENU_UP:
        case U8X8_MSG_GPIO_MENU_DOWN:
            u8x8_SetGPIOResult(u8x8, SSD1306_COLOR_OFF);
            return SSD1306_U8X8_SUCCESS;
        default:
            return SSD1306_U8X8_SUCCESS;
    }
}

/* Bring the display online and clear the frame buffer. */
bool ssd1306_initialize(void) {
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&g_displayContext, U8G2_R0, u8x8_byte_i2c1_hw, u8x8_gpio_and_delay_stm32);
    u8g2_InitDisplay(&g_displayContext);

    /* Some panels briefly report zero geometry while coming out of reset.  */
    uint8_t detected_height = u8g2_GetDisplayHeight(&g_displayContext);
    uint8_t detected_width = u8g2_GetDisplayWidth(&g_displayContext);
    if (detected_height == 0u || detected_width == 0u) {
        detected_height = SSD1306_HEIGHT;
        detected_width = SSD1306_WIDTH;
    }
    if (detected_height < SSD1306_HEIGHT || detected_width < SSD1306_WIDTH) {
        g_isDisplayReady = false;
        return false;
    }
    display_printf("[Debug] Detected %ux%u panel\n", (unsigned)detected_width, (unsigned)detected_height);
    u8g2_SetPowerSave(&g_displayContext, SSD1306_COLOR_OFF);
    u8g2_SetFont(&g_displayContext, SSD1306_DEFAULT_FONT);
    display_printf("[Debug] Font set\n");
    u8g2_SetFontPosTop(&g_displayContext);
    display_printf("[Debug] Font pos set\n");
    u8g2_ClearBuffer(&g_displayContext);
    display_printf("[Debug] Buffer cleared\n");

    g_isDisplayReady = true;
    ssd1306_clear();
    display_printf("[Debug] OLED Clear done\n");
    ssd1306_flush();
    display_printf("[Debug] OLED Initialization complete\n");
    return true;
}

/* Fill the whole buffer with either black or white. */
void ssd1306_fill(uint8_t color) {
    if (!g_isDisplayReady) {
        return;
    }
    if (color == SSD1306_COLOR_OFF) {
        u8g2_ClearBuffer(&g_displayContext);
    } else {
        u8g2_SetDrawColor(&g_displayContext, SSD1306_COLOR_ON);
        u8g2_DrawBox(&g_displayContext, SSD1306_ORIGIN_X, SSD1306_ORIGIN_Y, SSD1306_WIDTH, SSD1306_HEIGHT);
    }
    u8g2_SetDrawColor(&g_displayContext, SSD1306_DEFAULT_DRAW_COLOR);
}

void ssd1306_clear(void) {
    ssd1306_fill(SSD1306_COLOR_OFF);
}

/* Draw a solid rectangle while clipping to the display area. */
void ssd1306_fill_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, bool color) {
    if (!g_isDisplayReady || x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT || width == 0u || height == 0u) {
        return;
    }
    uint8_t w = width;
    uint8_t h = height;
    if ((uint16_t)x + width > SSD1306_WIDTH) {
        w = (uint8_t)(SSD1306_WIDTH - x);
    }
    if ((uint16_t)y + height > SSD1306_HEIGHT) {
        h = (uint8_t)(SSD1306_HEIGHT - y);
    }
    u8g2_SetDrawColor(&g_displayContext, color ? SSD1306_COLOR_ON : SSD1306_COLOR_OFF);
    u8g2_DrawBox(&g_displayContext, x, y, w, h);
    u8g2_SetDrawColor(&g_displayContext, SSD1306_DEFAULT_DRAW_COLOR);
}

/* Render a single character using the font selected during init. */
void ssd1306_draw_char(uint8_t x, uint8_t y, char c, bool color) {
    if (!g_isDisplayReady || x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }
    unsigned char glyph = (unsigned char)toupper((unsigned char)c);
    u8g2_SetDrawColor(&g_displayContext, color ? SSD1306_COLOR_ON : SSD1306_COLOR_OFF);
    u8g2_DrawGlyph(&g_displayContext, x, y, glyph);
    u8g2_SetDrawColor(&g_displayContext, SSD1306_DEFAULT_DRAW_COLOR);
}

/* Render an ASCII string after uppercasing it so the small font stays readable. */
void ssd1306_draw_text(uint8_t x, uint8_t y, const char *text, bool color) {
    if (!g_isDisplayReady || text == NULL || x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }

    char buffer[SSD1306_TEXT_BUFFER_LENGTH];
    size_t len = strnlen(text, SSD1306_TEXT_BUFFER_LENGTH - SSD1306_BUFFER_PADDING);
    for (size_t i = 0u; i < len; ++i) {
        buffer[i] = (char)toupper((unsigned char)text[i]);
    }
    buffer[len] = '\0';

    u8g2_SetDrawColor(&g_displayContext, color ? SSD1306_COLOR_ON : SSD1306_COLOR_OFF);
    u8g2_DrawStr(&g_displayContext, x, y, buffer);
    u8g2_SetDrawColor(&g_displayContext, SSD1306_DEFAULT_DRAW_COLOR);
}

/* Copy the RAM buffer out to the panel over I2C. */
void ssd1306_flush(void) {
#if SSD1306_DEBUG_FLUSH
    display_printf("[Debug] OLED Flush called\n");
#endif
    if (!g_isDisplayReady) {
        return;
    }
    u8g2_SendBuffer(&g_displayContext);
    iwdg_kick();
#if SSD1306_DEBUG_FLUSH
    display_printf("[Debug] OLED Flush done\n");
#endif
}

u8g2_t *ssd1306_get_display_context(void) {
    if (!g_isDisplayReady) {
        return NULL;
    }
    return &g_displayContext;
}
