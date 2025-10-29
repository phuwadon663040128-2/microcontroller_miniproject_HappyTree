#ifndef CONFIG_STORAGE_H
#define CONFIG_STORAGE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float cal_dry;
    float cal_wet;
    float t_low;
    float t_high;
} config_storage_payload_t;

void config_storage_init(void);
bool config_storage_load(config_storage_payload_t *out_payload);
bool config_storage_save(const config_storage_payload_t *payload);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_STORAGE_H */
