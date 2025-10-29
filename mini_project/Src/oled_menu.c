/* Simple text compositor for the 128x64 OLED panel (4 lines x 16 chars). */
#include "oled_menu.h"

#include "display.h"
#include "ssd1306.h"

#include "../../Library/csrc/u8g2.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define OLED_MENU_LINES 5u
#define OLED_MENU_COLS  16u
#define OLED_MENU_LINE_HEIGHT 11u
#define OLED_MENU_TOP_MARGIN 2u
#define OLED_MENU_TEXT_HEIGHT 9u

static char g_lineBuffer[OLED_MENU_LINES][OLED_MENU_COLS + 1u];
static bool g_lineNeedsRefresh[OLED_MENU_LINES];
static bool g_isMenuInitialized = false;
static bool g_isFlushPending = false;

extern void iwdg_kick(void);

static inline void line_to_tiles(uint8_t line, uint8_t *tile_y, uint8_t *tile_height) {
    uint8_t y = (uint8_t)(OLED_MENU_TOP_MARGIN + line * OLED_MENU_LINE_HEIGHT);
    *tile_y = (uint8_t)(y / 8u);
    *tile_height = 2u;
}

static void push_line_area(uint8_t line) {
    u8g2_t *ctx = ssd1306_get_display_context();
    if (ctx == NULL) {
        return;
    }
    uint8_t tile_y = 0u;
    uint8_t tile_height = 0u;
    line_to_tiles(line, &tile_y, &tile_height);
    if (tile_y >= 8u) {
        return;
    }
    if ((uint16_t)tile_y + tile_height > 8u) {
        tile_height = (uint8_t)(8u - tile_y);
    }
    u8g2_UpdateDisplayArea(ctx, 0u, tile_y, 16u, tile_height);
    iwdg_kick();
}

/* Clamp the incoming text to the display size and replace unsupported characters. */
static void sanitize_to_display_line(const char *src, char *dst) {
    size_t i = 0u;
    if (src) {
        while (src[i] != '\0' && i < OLED_MENU_COLS) {
            char c = src[i];
            if (c >= 'a' && c <= 'z') {
                c = (char)(c - 'a' + 'A');
            }
            if ((unsigned char)c < 32u || (unsigned char)c > 126u) {
                c = ' ';
            }
            dst[i] = c;
            ++i;
        }
    }
    while (i < OLED_MENU_COLS) {
        dst[i++] = ' ';
    }
    dst[OLED_MENU_COLS] = '\0';
}

/* Update one line and mark it as dirty so the next flush will redraw it. */
static bool update_line_buffer(uint8_t index, const char *text) {
    if (index >= OLED_MENU_LINES) {
        return false;
    }
    char sanitized[OLED_MENU_COLS + 1u];
    sanitize_to_display_line(text, sanitized);
    if (memcmp(g_lineBuffer[index], sanitized, OLED_MENU_COLS + 1u) != 0) {
        memcpy(g_lineBuffer[index], sanitized, OLED_MENU_COLS + 1u);
        g_lineNeedsRefresh[index] = true;
        g_isFlushPending = true;
        return true;
    }
    return false;
}

/* Erase the current line and draw the uppercase text using the SSD1306 helper. */
static void render_line_to_display(uint8_t index) {
    uint8_t y = (uint8_t)(OLED_MENU_TOP_MARGIN + index * OLED_MENU_LINE_HEIGHT);
    ssd1306_fill_rect(SSD1306_ORIGIN_X, y, SSD1306_WIDTH, OLED_MENU_TEXT_HEIGHT, SSD1306_COLOR_OFF);
    ssd1306_draw_text(SSD1306_ORIGIN_X, y, g_lineBuffer[index], SSD1306_COLOR_ON);
    g_lineNeedsRefresh[index] = false;
}

/* Initialize the backing buffers and probe the panel; returns false when absent. */
bool oled_menu_init(void) {
    memset(g_lineBuffer, ' ', sizeof(g_lineBuffer));
    for (uint8_t i = 0u; i < OLED_MENU_LINES; ++i) {
        g_lineBuffer[i][OLED_MENU_COLS] = '\0';
        g_lineNeedsRefresh[i] = true;
    }
    g_isFlushPending = true;
    display_printf("[OLED] Initializing...\n");
    g_isMenuInitialized = ssd1306_initialize();
    display_printf("[OLED] Initialization %s\n", g_isMenuInitialized ? "succeeded" : "failed");
    if (!g_isMenuInitialized) {
        return false;
    }

    oled_menu_set_lines("MOISTURE CTRL", "STATE:BOOT", "WAIT SENSORS", "BTN4 CAL MENU", "FOO BAR");
    oled_menu_process();
    return true;
}

/* Copy all four lines at once; individual lines are still filtered through set_line. */
void oled_menu_set_lines(const char *line0, const char *line1, const char *line2, const char *line3, const char *line4) {
    if (!g_isMenuInitialized) {
        return;
    }
    bool changed = false;
    changed |= update_line_buffer(0u, line0);
    changed |= update_line_buffer(1u, line1);
    changed |= update_line_buffer(2u, line2);
    changed |= update_line_buffer(3u, line3);
    changed |= update_line_buffer(4u, line4);

    if (changed) {
        display_uart_printf("[OLED] L0:%s\n[OLED] L1:%s\n[OLED] L2:%s\n[OLED] L3:%s\n[OLED] L4:%s\n",
                            g_lineBuffer[0], g_lineBuffer[1], g_lineBuffer[2], g_lineBuffer[3], g_lineBuffer[4]);
    }
}

/* Push any dirty lines to the hardware; call this regularly from the main loop. */
void oled_menu_process(void) {
    if (!g_isMenuInitialized || !g_isFlushPending) {
        return;
    }
    bool any_pushed = false;
    for (uint8_t i = 0u; i < OLED_MENU_LINES; ++i) {
        if (g_lineNeedsRefresh[i]) {
            render_line_to_display(i);
            push_line_area(i);
            any_pushed = true;
        }
    }
    if (any_pushed) {
        g_isFlushPending = false;
    }
}
