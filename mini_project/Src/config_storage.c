#include "config_storage.h"

#include "display.h"

#include <stdbool.h>

static config_storage_payload_t s_shadow_payload;
static bool s_shadow_valid = false;

void config_storage_init(void) {
    s_shadow_valid = false;
    display_printf("[Storage] Persistent storage disabled (RAM shadow only)\n");
}

bool config_storage_load(config_storage_payload_t *out_payload) {
    if (!out_payload) {
        return false;
    }

    if (!s_shadow_valid) {
        return false;
    }

    *out_payload = s_shadow_payload;
    return true;
}

bool config_storage_save(const config_storage_payload_t *payload) {
    if (!payload) {
        return false;
    }

    s_shadow_payload = *payload;
    s_shadow_valid = true;
    display_printf("[Storage] Updated in-memory configuration\n");
    return true;
}
