#ifndef SSD1306_H
#define SSD1306_H

#include <stdbool.h>
#include <stdint.h>

#include "../../Library/csrc/u8g2.h"

#define SSD1306_WIDTH 128u
#define SSD1306_HEIGHT 64u
#define SSD1306_ORIGIN_X 0u
#define SSD1306_ORIGIN_Y 0u
#define SSD1306_COLOR_OFF 0u
#define SSD1306_COLOR_ON  1u
#define SSD1306_BUFFER_PADDING 1u
#define SSD1306_TEXT_BUFFER_LENGTH (SSD1306_WIDTH + SSD1306_BUFFER_PADDING)

bool ssd1306_initialize(void);
void ssd1306_fill(uint8_t color);
void ssd1306_clear(void);
void ssd1306_flush(void);
void ssd1306_draw_char(uint8_t x, uint8_t y, char c, bool color);
void ssd1306_draw_text(uint8_t x, uint8_t y, const char *text, bool color);
void ssd1306_fill_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, bool color);
u8g2_t *ssd1306_get_display_context(void);

#endif /* SSD1306_H */
