#ifndef OLED_MENU_H
#define OLED_MENU_H

#include <stdbool.h>
#include <stdint.h>

bool oled_menu_init(void);
void oled_menu_set_lines(const char *line0, const char *line1, const char *line2, const char *line3, const char *line4);
void oled_menu_process(void);

#endif /* OLED_MENU_H */
