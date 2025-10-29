#include "app_logic.h"

#include "board_pins.h"
#include "display.h"
#include "oled_menu.h"
#include "config_storage.h"
#include "telemetry_uart.h"

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>

#ifndef LOG_LEVEL
#define LOG_LEVEL 2
#endif

/* Timebase exported by main */
extern volatile uint32_t g_ms;
extern uint32_t SystemCoreClock;

#define LED_PWM_STEPS 64u
#define LED_PWM_FREQ_HZ 200u

static volatile uint16_t s_led_pwm_levels[4] = {0u, 0u, 0u, 0u};
static volatile uint16_t s_led_pwm_phase = 0u;

/* ADC DMA buffers provided by main */
extern volatile uint16_t g_adc_dma_raw[];
extern volatile uint32_t g_adc_dma_seq;

/* -------------------- Filters / thresholds -------------------- */
#define EMA_NUM 1u
#define EMA_DEN 8u
#define DEFAULT_T_LOW 0.35f
#define DEFAULT_T_HIGH 0.55f
#define PERIODIC_TICK_MS 1000u
#define BTN_DEBOUNCE_MS 30u
#define BTN_LONG_MS 800u
#define BTN_BOOT_GUARD_MS 50u
#define ALERT_ACCUM_TICKS ((uint32_t)((EMA_DEN * 3u + 1u) / 2u))
#define ALERT_BLINK_MS 250u
#define SNOOZE_MS 60000u
#define ALERT_LATCH_DELAY_MS 3000u
#define ALERT_POST_ACK_GRACE_MS 8000u
#define OLED_KEEPALIVE_MS 5000u

#define CAL_FINE_STEP 0.005f
#define CAL_MIN_SPAN 0.10f
#define CAL_THRESH_MIN_WINDOW 0.08f

/* 7-seg codes to show super-states (BCD 0..9) */
#define DISP_IDLE 0u
#define DISP_MON 1u
#define DISP_ALERT 2u
#define DISP_CAL 3u
#define DISP_FAULT 4u

/* -------------------- Buttons -------------------- */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    const char *name;
    volatile uint32_t last_irq_ms;
    volatile uint32_t press_start_ms;
    volatile uint8_t was_down;
    volatile uint8_t short_flag;
    volatile uint8_t long_flag;
    volatile uint8_t long_reported;
} btn_t;

static uint8_t btn_down(const btn_t *b);
static void btn_handle_edge(btn_t *b);
static void btn_update_hold(btn_t *b);
static uint8_t btn_take_short(btn_t *b);
static uint8_t btn_take_long(btn_t *b);

/* -------------------- Application FSM -------------------- */
typedef enum
{
    ST_BOOT_SELFTEST = 0,
    ST_BOOT_LOADCFG,
    ST_BOOT_CALCHECK,
    ST_IDLE_STANDBY,
    ST_MONITOR,
    ST_ALERT_VISUAL,
    ST_ALERT_LATCH,
    ST_ALERT_SNOOZE,
    ST_CAL_MENU,
    ST_CAL_SET_DRY,
    ST_CAL_SET_WET,
    ST_CAL_SET_THRESH,
    ST_FAULT
} app_state_t;
// each state meaning

static app_state_t g_state = ST_BOOT_SELFTEST;
static btn_t b1 = {.port = BTN1_GPIO, .pin = BTN1_PIN, .name = "BTN1"};
static btn_t b2 = {.port = BTN2_GPIO, .pin = BTN2_PIN, .name = "BTN2"};
static btn_t b3 = {.port = BTN3_GPIO, .pin = BTN3_PIN, .name = "BTN3"};
static btn_t b4 = {.port = BTN4_GPIO, .pin = BTN4_PIN, .name = "BTN4"};
static bool s_oled_ready = false;

/* Runtime state */
static uint32_t last_periodic = 0u, last_blink = 0u, alert_visual_enter = 0u, snooze_until = 0u;
static uint32_t alert_grace_until = 0u;
static float v_temp = 0.0f, v_ldr = 0.0f, v_soil_moisture = 0.0f, v_pot = 0.0f;
static float cal_dry = 0.20f, cal_wet = 0.80f;
static float T_low = DEFAULT_T_LOW, T_high = DEFAULT_T_HIGH;
static uint32_t below_low_accum = 0u;
static uint32_t s_last_oled_sync = 0u;
static float s_last_oled_soil = -1.0f;
static float s_last_oled_low = -1.0f;
static float s_last_oled_high = -1.0f;
static uint8_t s_cal_session_active = 0u;
static app_state_t s_cal_session_state = ST_BOOT_SELFTEST;
static float s_cal_entry_primary = 0.0f;
static float s_cal_entry_secondary = 0.0f;
static float s_cal_span = CAL_THRESH_MIN_WINDOW;
static float s_cal_fine_primary = 0.0f;

/* -------------------- Utility helpers -------------------- */
//clamp01 is for normalizing to 0..1 range
//clampf is for clamping to specific range
static inline float clamp01(float x)
{
    if (x < 0.0f)
    {
        return 0.0f;
    }
    if (x > 1.0f)
    {
        return 1.0f;
    }
    return x;
}
static inline float clampf(float x, float min_v, float max_v)
{
    if (x < min_v)
    {
        return min_v;
    }
    if (x > max_v)
    {
        return max_v;
    }
    return x;
}
static float f_abs(float x) { return (x >= 0.0f) ? x : -x; }
static inline bool time_is_before(uint32_t now, uint32_t ref)
{
    return (int32_t)(now - ref) < 0;
}
static inline bool time_is_after_or_equal(uint32_t now, uint32_t ref)
{
    return (int32_t)(now - ref) >= 0;
}
static inline uint32_t time_diff(uint32_t future, uint32_t now)
{
    return future - now;
}

static uint32_t decode_apb_prescaler_bits(uint32_t bits)
{
    switch (bits)
    {
    case 0u:
    case 1u:
    case 2u:
    case 3u:
        return 1u;
    case 4u:
        return 2u;
    case 5u:
        return 4u;
    case 6u:
        return 8u;
    default:
        return 16u;
    }
}

static uint32_t apb1_timer_clock_hz(void)
{
    uint32_t pre_bits = (RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x7u;
    uint32_t prescaler = decode_apb_prescaler_bits(pre_bits);
    if (prescaler == 0u)
    {
        prescaler = 1u;
    }
    uint32_t pclk1 = SystemCoreClock / prescaler;
    if (prescaler > 1u)
    {
        pclk1 *= 2u;
    }
    return pclk1;
}

static void led_pwm_apply(uint32_t index, float intensity)
{
    if (index >= 4u)
    {
        return;
    }
    if (intensity < 0.0f)
    {
        intensity = 0.0f;
    }
    if (intensity > 1.0f)
    {
        intensity = 1.0f;
    }
    uint16_t ticks = (uint16_t)((intensity * (float)LED_PWM_STEPS) + 0.5f);
    if (ticks > LED_PWM_STEPS)
    {
        ticks = LED_PWM_STEPS;
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_led_pwm_levels[index] = ticks;
    __set_PRIMASK(primask);
}

static void led_bar_pwm_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->APB1ENR;

    TIM3->CR1 = 0u;
    TIM3->CR2 = 0u;
    TIM3->SMCR = 0u;
    TIM3->DIER = 0u;
    TIM3->SR = 0u;

    const uint32_t target_hz = (uint32_t)LED_PWM_STEPS * (uint32_t)LED_PWM_FREQ_HZ;
    uint32_t tim_clk = apb1_timer_clock_hz();
    if (tim_clk == 0u)
    {
        tim_clk = 1u;
    }
    uint32_t ticks = (tim_clk + (target_hz / 2u)) / target_hz;
    if (ticks == 0u)
    {
        ticks = 1u;
    }

    uint32_t prescaler = (ticks + 0xFFFFu) / 0x10000u;
    if (prescaler == 0u)
    {
        prescaler = 1u;
    }
    uint32_t arr = (ticks + prescaler - 1u) / prescaler;
    if (arr == 0u)
    {
        arr = 1u;
    }
    while (arr > 0xFFFFu && prescaler < 0xFFFFu)
    {
        prescaler++;
        arr = (ticks + prescaler - 1u) / prescaler;
        if (arr == 0u)
        {
            arr = 1u;
        }
    }
    if (arr > 0xFFFFu)
    {
        arr = 0xFFFFu;
    }

    TIM3->PSC = (uint16_t)(prescaler - 1u);
    TIM3->ARR = (uint16_t)(arr - 1u);
    TIM3->CNT = 0u;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->DIER = TIM_DIER_UIE;

    NVIC_SetPriority(TIM3_IRQn, 2u);
    NVIC_EnableIRQ(TIM3_IRQn);

    TIM3->CR1 = TIM_CR1_CEN;
}

static void bar_show(float level);
static void debug_log_leds(float l1, float l2, float l3, float l4);
static void debug_log_seg(uint8_t code);
static void debug_log_button_event(const btn_t *btn, const char *event_key);
static void debug_log_sensor(float temp_norm, float ldr_norm, float soil_norm, float pot_norm);
static void debug_log_threshold_eval(float Tl, float Th, float soil_moisture, uint32_t below_ticks);
static void seg_write_bcd(uint8_t v);
static void sample_filter(void);
static float read_avg(uint32_t ch, int sample_count);
static float normalize_moisture(float x);
static void calibration_sanitize(void);
static void show_big_state(app_state_t s);
static const char *state_name(app_state_t s);
static const char *state_short_label(app_state_t s);
static void format_fixed_2(char *dst, size_t len, float value);
static void oled_sync_status(void);
static void set_state(app_state_t next, const char *reason);
static void monitor_tick(void);
static void self_test_or_fault(void);
static void load_config_ram(void);
static void calibration_check(void);
static void buttons_exti_init(void);
static void persist_calibration(void);

/* -------------------- Debug helpers -------------------- */
#if LOG_LEVEL >= 2
static void debug_log_leds(float l1, float l2, float l3, float l4)
{
    uint8_t pct1 = (uint8_t)(clampf(l1, 0.0f, 1.0f) * 100.0f + 0.5f);
    uint8_t pct2 = (uint8_t)(clampf(l2, 0.0f, 1.0f) * 100.0f + 0.5f);
    uint8_t pct3 = (uint8_t)(clampf(l3, 0.0f, 1.0f) * 100.0f + 0.5f);
    uint8_t pct4 = (uint8_t)(clampf(l4, 0.0f, 1.0f) * 100.0f + 0.5f);
    if (pct1 > 100u)
        pct1 = 100u;
    if (pct2 > 100u)
        pct2 = 100u;
    if (pct3 > 100u)
        pct3 = 100u;
    if (pct4 > 100u)
        pct4 = 100u;

    static uint8_t last_pct1 = 0xFFu, last_pct2 = 0xFFu, last_pct3 = 0xFFu, last_pct4 = 0xFFu;
    if (pct1 == last_pct1 && pct2 == last_pct2 && pct3 == last_pct3 && pct4 == last_pct4)
    {
        return;
    }

    display_printf("[LEDs] L1=%3u%% L2=%3u%% L3=%3u%% L4=%3u%% @%lums\n",
                   (unsigned int)pct1,
                   (unsigned int)pct2,
                   (unsigned int)pct3,
                   (unsigned int)pct4,
                   (unsigned long)g_ms);
    last_pct1 = pct1;
    last_pct2 = pct2;
    last_pct3 = pct3;
    last_pct4 = pct4;
}

static void debug_log_seg(uint8_t code)
{
    static uint8_t last_code = 0xFFu;
    if (code == last_code)
    {
        return;
    }
    display_printf("[7SEG] BCD=%u (@%lums)\n",
                   (unsigned int)(code & 0x0Fu),
                   (unsigned long)g_ms);
    last_code = code & 0x0Fu;
}

static void debug_log_button_event(const btn_t *btn, const char *event_key)
{
    if (!btn || !event_key)
    {
        return;
    }
    display_printf("[Buttons] %s %s @%lums\n", btn->name ? btn->name : "BTN", event_key, (unsigned long)g_ms);
}

static void debug_log_sensor(float temp_norm, float ldr_norm, float soil_norm, float pot_norm)
{
    static float prev_temp = -1.0f, prev_ldr = -1.0f, prev_soil = -1.0f, prev_pot = -1.0f;
    static uint32_t last_log = 0u;
    const float delta_thresh = 0.02f;
    bool first = (last_log == 0u);
    bool drift = (f_abs(temp_norm - prev_temp) > delta_thresh) ||
                 (f_abs(ldr_norm - prev_ldr) > delta_thresh) ||
                 (f_abs(soil_norm - prev_soil) > delta_thresh) ||
                 (f_abs(pot_norm - prev_pot) > delta_thresh);
    bool elapsed = (g_ms - last_log) >= 1000u;

    if (first || drift || elapsed)
    {
        display_printf("[Sensors] T=%.3f L=%.3f SM=%.3f POT=%.3f @%lums\n",
                       (double)temp_norm,
                       (double)ldr_norm,
                       (double)soil_norm,
                       (double)pot_norm,
                       (unsigned long)g_ms);
        prev_temp = temp_norm;
        prev_ldr = ldr_norm;
        prev_soil = soil_norm;
        prev_pot = pot_norm;
        last_log = g_ms;
    }
}

static void debug_log_threshold_eval(float Tl, float Th, float soil_moisture, uint32_t below_ticks)
{
    static float prev_Tl = -1.0f, prev_Th = -1.0f, prev_m = -1.0f;
    static uint32_t prev_ticks = UINT32_MAX;
    const float delta_thresh = 0.01f;
    bool first = (prev_ticks == UINT32_MAX);
    bool changed = (f_abs(Tl - prev_Tl) > delta_thresh) ||
                   (f_abs(Th - prev_Th) > delta_thresh) ||
                   (f_abs(soil_moisture - prev_m) > delta_thresh) ||
                   (below_ticks != prev_ticks);
    if (first || changed)
    {
        display_printf("[Eval] Tl=%.3f Th=%.3f SM=%.3f acc=%lu @%lums\n",
                       (double)Tl,
                       (double)Th,
                       (double)soil_moisture,
                       (unsigned long)below_ticks,
                       (unsigned long)g_ms);
        prev_Tl = Tl;
        prev_Th = Th;
        prev_m = soil_moisture;
        prev_ticks = below_ticks;
    }
}
#else
static void debug_log_leds(float l1, float l2, float l3, float l4)
{
    (void)l1;
    (void)l2;
    (void)l3;
    (void)l4;
}
static void debug_log_seg(uint8_t code) { (void)code; }
static void debug_log_button_event(const btn_t *btn, const char *event_key)
{
    (void)btn;
    (void)event_key;
}
static void debug_log_sensor(float temp_norm, float ldr_norm, float soil_norm, float pot_norm)
{
    (void)temp_norm;
    (void)ldr_norm;
    (void)soil_norm;
    (void)pot_norm;
}
static void debug_log_threshold_eval(float Tl, float Th, float soil_moisture, uint32_t below_ticks)
{
    (void)Tl;
    (void)Th;
    (void)soil_moisture;
    (void)below_ticks;
}
#endif

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF)
    {
        TIM3->SR &= ~TIM_SR_UIF;

        uint16_t phase = s_led_pwm_phase;
        uint16_t cmp = phase;
        phase++;
        if (phase >= LED_PWM_STEPS)
        {
            phase = 0u;
        }
        s_led_pwm_phase = phase;

        uint32_t set_a = 0u;
        uint32_t reset_a = 0u;
        uint32_t set_b = 0u;
        uint32_t reset_b = 0u;

        uint16_t lvl0 = s_led_pwm_levels[0];
        if (lvl0 > cmp)
        {
            set_a |= (1u << LED1_PIN);
        }
        else
        {
            reset_a |= (1u << LED1_PIN);
        }

        uint16_t lvl1 = s_led_pwm_levels[1];
        if (lvl1 > cmp)
        {
            set_a |= (1u << LED2_PIN);
        }
        else
        {
            reset_a |= (1u << LED2_PIN);
        }

        uint16_t lvl2 = s_led_pwm_levels[2];
        if (lvl2 > cmp)
        {
            set_a |= (1u << LED3_PIN);
        }
        else
        {
            reset_a |= (1u << LED3_PIN);
        }

        uint16_t lvl3 = s_led_pwm_levels[3];
        if (lvl3 > cmp)
        {
            set_b |= (1u << LED4_PIN);
        }
        else
        {
            reset_b |= (1u << LED4_PIN);
        }

        GPIOA->BSRR = set_a | (reset_a << 16u);
        GPIOB->BSRR = set_b | (reset_b << 16u);
    }
}

/* -------------------- Indicators -------------------- */
static void bar_show(float level)
{
    if (level < 0.0f)
    {
        level = 0.0f;
    }
    if (level > 4.0f)
    {
        level = 4.0f;
    }

    float fill0 = clampf(level, 0.0f, 1.0f);
    float fill1 = clampf(level - 1.0f, 0.0f, 1.0f);
    float fill2 = clampf(level - 2.0f, 0.0f, 1.0f);
    float fill3 = clampf(level - 3.0f, 0.0f, 1.0f);

    led_pwm_apply(0u, fill0);
    led_pwm_apply(1u, fill1);
    led_pwm_apply(2u, fill2);
    led_pwm_apply(3u, fill3);

    debug_log_leds(fill0, fill1, fill2, fill3);
}

static void seg_write_bcd(uint8_t v)
{
    static uint8_t current_code = 0xFFu;
    uint8_t code = v & 0x0Fu;
    if (code == current_code)
    {
        return;
    }

    SEG_A_GPIO->BSRR = (code & 0x1u) ? (1u << SEG_A_PIN) : (1u << (SEG_A_PIN + 16u));
    SEG_B_GPIO->BSRR = (code & 0x2u) ? (1u << SEG_B_PIN) : (1u << (SEG_B_PIN + 16u));
    SEG_C_GPIO->BSRR = (code & 0x4u) ? (1u << SEG_C_PIN) : (1u << (SEG_C_PIN + 16u));
    SEG_D_GPIO->BSRR = (code & 0x8u) ? (1u << SEG_D_PIN) : (1u << (SEG_D_PIN + 16u));
    debug_log_seg(code);
    current_code = code;
}

/* -------------------- Button handling -------------------- */
static uint8_t btn_down(const btn_t *b)
{
    return (b->port->IDR & (1u << b->pin)) ? 0u : 1u;
}

static void btn_handle_edge(btn_t *b)
{
    uint32_t now = g_ms;

    uint8_t pressed = btn_down(b);
    if (pressed)
    {
        if (!b->was_down)
        {
            b->was_down = 1u;
            b->press_start_ms = now;
            b->long_reported = 0u;
        }
    }
    else
    {
        if (!b->was_down)
        {
            return;
        }
        uint32_t held_ms = now - b->press_start_ms;
        if (!b->long_reported && held_ms >= BTN_DEBOUNCE_MS && b->press_start_ms > BTN_BOOT_GUARD_MS)
        {
            b->short_flag = 1u;
        }
        b->was_down = 0u;
        b->long_reported = 0u;
    }
}

static void btn_update_hold(btn_t *b)
{
    if (b->was_down && !b->long_reported)
    {
        if ((g_ms - b->press_start_ms) >= BTN_LONG_MS)
        {
            b->long_flag = 1u;
            b->long_reported = 1u;
        }
    }
}

static uint8_t btn_take_short(btn_t *b)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t f = b->short_flag;
    b->short_flag = 0u;
    __set_PRIMASK(primask);
    if (f)
    {
        debug_log_button_event(b, "short");
    }
    return f;
}

static uint8_t btn_take_long(btn_t *b)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t f = b->long_flag;
    b->long_flag = 0u;
    __set_PRIMASK(primask);
    if (f)
    {
        debug_log_button_event(b, "long");
    }
    return f;
}

/* -------------------- Sensor & calibration -------------------- */
static void sample_filter(void)
{
    const float inv4095 = 1.0f / 4095.0f;
    const float a = ((float)EMA_NUM) / ((float)EMA_DEN);
    const float one_a = 1.0f - a;
    float r0 = (float)g_adc_dma_raw[ADC_IDX_TEMP] * inv4095;
    float r1 = (float)g_adc_dma_raw[ADC_IDX_LDR] * inv4095;
    float r_soil = (float)g_adc_dma_raw[ADC_IDX_SOIL] * inv4095;
    float r_pot = (float)g_adc_dma_raw[ADC_IDX_POT] * inv4095;
    static uint8_t seeded = 0u;
    if (!seeded)
    {
        v_temp = clamp01(r0);
        v_ldr = clamp01(r1);
        v_soil_moisture = clamp01(r_soil);
        v_pot = clamp01(r_pot);
        seeded = 1u;
    }
    else
    {
        v_temp = clamp01(one_a * v_temp + a * r0);
        v_ldr = clamp01(one_a * v_ldr + a * r1);
        v_soil_moisture = clamp01(one_a * v_soil_moisture + a * r_soil);
        v_pot = clamp01(one_a * v_pot + a * r_pot);
    }

    debug_log_sensor(v_temp, v_ldr, v_soil_moisture, v_pot);
}

static float read_avg(uint32_t ch, int sample_count)
{
    if (sample_count <= 0)
    {
        return 0.0f;
    }

    uint32_t idx;
    switch (ch)
    {
    case CH_TEMP:
        idx = ADC_IDX_TEMP;
        break;
    case CH_LDR:
        idx = ADC_IDX_LDR;
        break;
    case CH_SOIL:
        idx = ADC_IDX_SOIL;
        break;
    case CH_POT:
        idx = ADC_IDX_POT;
        break;
    default:
        idx = ADC_IDX_SOIL;
        break;
    }

    const float inv4095 = 1.0f / 4095.0f;
    float acc = (float)g_adc_dma_raw[idx] * inv4095;
    int collected = 1;
    uint32_t last_seq = g_adc_dma_seq;

    while (collected < sample_count)
    {
        uint32_t wait_start = g_ms;
        while ((g_adc_dma_seq == last_seq) && ((g_ms - wait_start) < 20u))
        {
            __NOP();
        }

        if (g_adc_dma_seq == last_seq)
        {
            /* Timed out waiting for new data; use current sample once more */
            acc += (float)g_adc_dma_raw[idx] * inv4095;
            collected++;
            continue;
        }

        last_seq = g_adc_dma_seq;
        acc += (float)g_adc_dma_raw[idx] * inv4095;
        collected++;
    }

    float avg = acc / (float)collected;
    return clamp01(avg);
}

static float normalize_moisture(float x)
{
    const float eps = 1e-3f;
    float dry = cal_dry;
    float wet = cal_wet;
    if (wet < dry)
    {
        float t = wet;
        wet = dry;
        dry = t;
    }
    float span = wet - dry;
    if (span < eps)
        return 0.0f;
    float y = (x - dry) / span;
    return clamp01(y);
}

static void calibration_sanitize(void)
{
    cal_dry = clamp01(cal_dry);
    cal_wet = clamp01(cal_wet);
    if (cal_wet < cal_dry)
    {
        float t = cal_wet;
        cal_wet = cal_dry;
        cal_dry = t;
    }
    float span = cal_wet - cal_dry;
    if (span < CAL_MIN_SPAN)
    {
        float prev = span;
        cal_wet = clamp01(cal_dry + CAL_MIN_SPAN);
        span = cal_wet - cal_dry;
        display_printf("[Cal] Dry/Wet span grew from %.3f to %.3f\n", (double)prev, (double)span);
    }
    if (T_low < 0.f)
        T_low = 0.f;
    if (T_low > 1.f)
        T_low = 1.f;
    if (T_high < 0.f)
        T_high = 0.f;
    if (T_high > 1.f)
        T_high = 1.f;
    const float min_window = CAL_THRESH_MIN_WINDOW;
    if (T_high <= T_low + min_window)
    {
        float prev_window = T_high - T_low;
        T_high = clamp01(T_low + min_window);
        display_printf("[Cal] Threshold window widened from %.3f to %.3f\n",
                       (double)prev_window,
                       (double)(T_high - T_low));
    }
}

/* -------------------- State helpers -------------------- */
static void show_big_state(app_state_t s)
{
    switch (s)
    {
    case ST_IDLE_STANDBY:
    case ST_BOOT_SELFTEST:
    case ST_BOOT_LOADCFG:
    case ST_BOOT_CALCHECK:
        seg_write_bcd(DISP_IDLE);
        break;
    case ST_MONITOR:
        seg_write_bcd(DISP_MON);
        break;
    case ST_ALERT_VISUAL:
    case ST_ALERT_LATCH:
    case ST_ALERT_SNOOZE:
        seg_write_bcd(DISP_ALERT);
        break;
    case ST_CAL_MENU:
    case ST_CAL_SET_DRY:
    case ST_CAL_SET_WET:
    case ST_CAL_SET_THRESH:
        seg_write_bcd(DISP_CAL);
        break;
    case ST_FAULT:
        seg_write_bcd(DISP_FAULT);
        break;
    default:
        break;
    }
}

static const char *state_name(app_state_t s)
{
    switch (s)
    {
    case ST_BOOT_SELFTEST:
        return "BOOT_SELFTEST";
    case ST_BOOT_LOADCFG:
        return "BOOT_LOADCFG";
    case ST_BOOT_CALCHECK:
        return "BOOT_CALCHECK";
    case ST_IDLE_STANDBY:
        return "IDLE_STANDBY";
    case ST_MONITOR:
        return "MONITOR";
    case ST_ALERT_VISUAL:
        return "ALERT_VISUAL";
    case ST_ALERT_LATCH:
        return "ALERT_LATCH";
    case ST_ALERT_SNOOZE:
        return "ALERT_SNOOZE";
    case ST_CAL_MENU:
        return "CAL_MENU";
    case ST_CAL_SET_DRY:
        return "CAL_SET_DRY";
    case ST_CAL_SET_WET:
        return "CAL_SET_WET";
    case ST_CAL_SET_THRESH:
        return "CAL_SET_THRESH";
    case ST_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

static const char *state_short_label(app_state_t s)
{
    switch (s)
    {
    case ST_BOOT_SELFTEST:
        return "BOOT TEST";
    case ST_BOOT_LOADCFG:
        return "BOOT CFG";
    case ST_BOOT_CALCHECK:
        return "BOOT CAL";
    case ST_IDLE_STANDBY:
        return "IDLE";
    case ST_MONITOR:
        return "MONITOR";
    case ST_ALERT_VISUAL:
        return "ALERT VIS";
    case ST_ALERT_LATCH:
        return "ACK WAIT";
    case ST_ALERT_SNOOZE:
        return "SNOOZE";
    case ST_CAL_MENU:
        return "CAL MENU";
    case ST_CAL_SET_DRY:
        return "SET DRY";
    case ST_CAL_SET_WET:
        return "SET WET";
    case ST_CAL_SET_THRESH:
        return "SET THR";
    case ST_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

static void format_fixed_2(char *dst, size_t len, float value)
{
    if (!dst || len == 0u)
    {
        return;
    }

    if (len < 6u)
    {
        dst[0] = '\0';
        return;
    }

    value = clamp01(value);
    int scaled = (int)(value * 100.0f + 0.5f);

    int whole = scaled / 100;
    int frac = scaled % 100;

    (void)snprintf(dst, len, "%d.%02d", whole, frac);
}

static void oled_sync_status(void)
{
    if (!s_oled_ready)
    {
        return;
    }

    const char *line0 = "MOISTURE CTRL";
    char line1[17];
    char line2[17];
    char line3[17];
    char line4[17];

    line4[0] = '\0';

    const char *label = state_short_label(g_state);
    snprintf(line1, sizeof(line1), "STATE:%-10.10s", label ? label : "UNKNOWN");

    float soil_moisture = normalize_moisture(v_soil_moisture);
    char soil_buf[6];
    char temp_buf[6];
    char low_buf[6];
    char high_buf[6];

    format_fixed_2(soil_buf, sizeof(soil_buf), soil_moisture);
    format_fixed_2(temp_buf, sizeof(temp_buf), v_temp);
    format_fixed_2(low_buf, sizeof(low_buf), T_low);
    format_fixed_2(high_buf, sizeof(high_buf), T_high);

    snprintf(line2, sizeof(line2), "SM:%s T:%s", soil_buf, temp_buf);
    snprintf(line3, sizeof(line3), "L:%s H:%s", low_buf, high_buf);

    switch (g_state)
    {
    case ST_ALERT_VISUAL:
    case ST_ALERT_LATCH:
        snprintf(line2, sizeof(line2), "ALERT ACTIVE");
        snprintf(line3, sizeof(line3), "B2 ACK B3 SNOOZE");
        break;
    case ST_ALERT_SNOOZE:
    {
        uint32_t remaining_ms = 0u;
        if (snooze_until != 0u && time_is_before(g_ms, snooze_until))
        {
            remaining_ms = time_diff(snooze_until, g_ms);
        }
        uint32_t remaining_s = remaining_ms / 1000u;
        if (remaining_s > 99u)
        {
            remaining_s = 99u;
        }
        snprintf(line2, sizeof(line2), "SNOOZE %02lus", (unsigned long)remaining_s);
        snprintf(line3, sizeof(line3), "B1 CANCEL");
        break;
    }
    case ST_CAL_MENU:
        snprintf(line2, sizeof(line2), "B1 DRY B2 WET");
        snprintf(line3, sizeof(line3), "B3 THR B4 SAVE");
        break;
    case ST_CAL_SET_DRY:
        snprintf(line2, sizeof(line2), "DRY:%05.2f B4OK", (double)cal_dry);
        snprintf(line3, sizeof(line3), "KNOB ROUGH SET");
        snprintf(line4, sizeof(line4), "B1 BACK B2+/B3-");
        break;
    case ST_CAL_SET_WET:
        snprintf(line2, sizeof(line2), "WET:%05.2f B4OK", (double)cal_wet);
        snprintf(line3, sizeof(line3), "KNOB ROUGH SET");
        snprintf(line4, sizeof(line4), "B1 BACK B2+/B3-");
        break;
    case ST_CAL_SET_THRESH:
        snprintf(line2, sizeof(line2), "L:%04.2f H:%04.2f", (double)T_low, (double)T_high);
        snprintf(line3, sizeof(line3), "KNOB CTR B4OK");
        snprintf(line4, sizeof(line4), "B1 BACK B2+/B3-");
        break;
    case ST_FAULT:
        snprintf(line2, sizeof(line2), "FAULT STATE");
        snprintf(line3, sizeof(line3), "HOLD B1+B2");
        break;
    default:
        break;
    }

    oled_menu_set_lines(line0, line1, line2, line3, line4);
}

static void set_state(app_state_t next, const char *reason)
{
    app_state_t prev = g_state;
    if (prev == next)
    {
        return;
    }
    g_state = next;
    if (next != ST_CAL_SET_DRY && next != ST_CAL_SET_WET && next != ST_CAL_SET_THRESH)
    {
        s_cal_session_active = 0u;
        s_cal_session_state = ST_BOOT_SELFTEST;
    }
    if (next == ST_ALERT_VISUAL)
    {
        alert_visual_enter = g_ms;
        last_blink = g_ms;
    }
    else if (prev == ST_ALERT_VISUAL)
    {
        alert_visual_enter = 0u;
    }
    if (reason)
    {
        if (reason[0] != '\0')
        {
            display_printf("[FSM] %s -> %s (%s)\n", state_name(prev), state_name(next), reason);
        }
        else
        {
            display_printf("[FSM] %s -> %s\n", state_name(prev), state_name(next));
        }
    }
    show_big_state(next);
    oled_sync_status();
    s_last_oled_sync = g_ms;
    s_last_oled_soil = normalize_moisture(v_soil_moisture);
    s_last_oled_low = T_low;
    s_last_oled_high = T_high;
}

static void monitor_tick(void)
{
    sample_filter();

    float soil_moisture = normalize_moisture(v_soil_moisture);
    bar_show(soil_moisture * 4.0f);

    float temp_adj = (v_temp - 0.5f) * 0.10f;
    float ldr_adj = (0.5f - v_ldr) * 0.05f;
    float Tl = clamp01(T_low + temp_adj + ldr_adj);
    float Th = clamp01(T_high + temp_adj + ldr_adj);

    bool above_high = (soil_moisture > Th);
    bool in_band = (soil_moisture >= Tl);
    bool in_grace = (alert_grace_until != 0u) && time_is_before(g_ms, alert_grace_until);

    if (above_high)
    {
        below_low_accum = 0u;
        alert_grace_until = 0u;
    }
    else if (in_band)
    {
        below_low_accum = 0u;
    }
    else
    {
        if (in_grace)
        {
            below_low_accum = 0u;
        }
        else if (below_low_accum < UINT32_MAX)
        {
            below_low_accum++;
        }
    }

    debug_log_threshold_eval(Tl, Th, soil_moisture, below_low_accum);

    if (telemetry_uart_is_ready())
    {
        const bool alerting = (g_state == ST_ALERT_VISUAL) ||
                              (g_state == ST_ALERT_LATCH) ||
                              (g_state == ST_ALERT_SNOOZE);
        telemetry_sample_t sample = {
            .state = state_name(g_state),
            .soil_moisture_norm = soil_moisture,
            .temp_norm = v_temp,
            .ldr_norm = v_ldr,
            .threshold_low = Tl,
            .threshold_high = Th,
            .below_low_ticks = below_low_accum,
            .alert_active = alerting,
            .uptime_ms = g_ms};

        if (!telemetry_uart_publish(&sample))
        {
            static uint32_t last_warn_ms = 0u;
            if ((g_ms - last_warn_ms) >= 2000u)
            {
                display_printf("[Telemetry] publish failed (ESP8266 not acknowledging)\n");
                last_warn_ms = g_ms;
            }
        }
    }

    if (!in_grace && !above_high && !in_band && below_low_accum >= ALERT_ACCUM_TICKS)
    {
    display_printf("[Alert] acc=%lu Tl=%.3f Th=%.3f SM=%.3f\n",
                       (unsigned long)below_low_accum,
                       (double)Tl,
               (double)Th,
               (double)soil_moisture);
        set_state(ST_ALERT_VISUAL, "threshold violated");
    }
}

static void persist_calibration(void)
{
    config_storage_payload_t payload = {
        .cal_dry = cal_dry,
        .cal_wet = cal_wet,
        .t_low = T_low,
        .t_high = T_high};

    if (config_storage_save(&payload))
    {
        display_printf("[Cal] Saved calibration to storage\n");
    }
    else
    {
        display_printf("[Cal] Storage save failed\n");
    }
}

static void self_test_or_fault(void)
{
    if (!(RCC->APB2ENR & RCC_APB2ENR_ADC1EN))
    {
        set_state(ST_FAULT, "ADC clock disabled");
        return;
    }
    set_state(ST_BOOT_LOADCFG, "self-test ok");
}

static void load_config_ram(void)
{
    config_storage_payload_t payload;
    if (config_storage_load(&payload))
    {
        cal_dry = payload.cal_dry;
        cal_wet = payload.cal_wet;
        T_low = payload.t_low;
        T_high = payload.t_high;
        calibration_sanitize();
        display_printf("[Boot] Calibration loaded from storage\n");
        set_state(ST_BOOT_CALCHECK, "cfg loaded");
    }
    else
    {
        cal_dry = 0.20f;
        cal_wet = 0.80f;
        T_low = DEFAULT_T_LOW;
        T_high = DEFAULT_T_HIGH;
        calibration_sanitize();
        display_printf("[Boot] Using default calibration\n");
        set_state(ST_BOOT_CALCHECK, "defaults loaded");
    }
}

static void calibration_check(void)
{
    calibration_sanitize();
    const float span = cal_wet - cal_dry;
    if (!(T_low >= 0.f && T_high <= 1.f && T_low < T_high))
    {
        set_state(ST_FAULT, "threshold window invalid");
        return;
    }
    if (span < 1e-3f)
    {
        set_state(ST_CAL_MENU, "calibration required");
        display_printf("[Boot] Uncalibrated: enter Cal Menu (B1=Dry,B2=Wet,B3=Thresh,B4=Save)\n");
        return;
    }
    set_state(ST_IDLE_STANDBY, "calibration valid");
}

static void buttons_exti_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    uint32_t exticr0 = SYSCFG->EXTICR[0];
    exticr0 &= ~SYSCFG_EXTICR1_EXTI3;
    exticr0 |= SYSCFG_EXTICR1_EXTI3_PB;
    SYSCFG->EXTICR[0] = exticr0;

    uint32_t exticr1 = SYSCFG->EXTICR[1];
    exticr1 &= ~(SYSCFG_EXTICR2_EXTI4 | SYSCFG_EXTICR2_EXTI5);
    exticr1 |= SYSCFG_EXTICR2_EXTI4_PB | SYSCFG_EXTICR2_EXTI5_PB;
    SYSCFG->EXTICR[1] = exticr1;

    uint32_t exticr2 = SYSCFG->EXTICR[2];
    exticr2 &= ~SYSCFG_EXTICR3_EXTI10;
    exticr2 |= SYSCFG_EXTICR3_EXTI10_PA;
    SYSCFG->EXTICR[2] = exticr2;

    const uint32_t mask = (1u << BTN1_PIN) | (1u << BTN2_PIN) | (1u << BTN3_PIN) | (1u << BTN4_PIN);
    EXTI->IMR |= mask;
    EXTI->RTSR |= mask;
    EXTI->FTSR |= mask;
    EXTI->PR = mask;

    uint32_t init_edge = g_ms - BTN_DEBOUNCE_MS;
    b1.last_irq_ms = init_edge;
    b2.last_irq_ms = init_edge;
    b3.last_irq_ms = init_edge;
    b4.last_irq_ms = init_edge;

    b1.was_down = btn_down(&b1);
    b2.was_down = btn_down(&b2);
    b3.was_down = btn_down(&b3);
    b4.was_down = btn_down(&b4);

    b1.short_flag = b1.long_flag = b1.long_reported = 0u;
    b2.short_flag = b2.long_flag = b2.long_reported = 0u;
    b3.short_flag = b3.long_flag = b3.long_reported = 0u;
    b4.short_flag = b4.long_flag = b4.long_reported = 0u;

    if (b1.was_down)
    {
        b1.press_start_ms = g_ms;
    }
    if (b2.was_down)
    {
        b2.press_start_ms = g_ms;
    }
    if (b3.was_down)
    {
        b3.press_start_ms = g_ms;
    }
    if (b4.was_down)
    {
        b4.press_start_ms = g_ms;
    }

    NVIC_SetPriority(EXTI15_10_IRQn, 4u);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI9_5_IRQn, 4u);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_SetPriority(EXTI4_IRQn, 4u);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_SetPriority(EXTI3_IRQn, 4u);
    NVIC_EnableIRQ(EXTI3_IRQn);
}

/* -------------------- Public API -------------------- */
void app_logic_init(void)
{
    buttons_exti_init();
    led_bar_pwm_init();
    config_storage_init();
    display_printf("[Boot] end config storage init\n");

    s_oled_ready = oled_menu_init();
    display_printf("[Boot] end OLED init\n");
    display_printf("[Boot] OLED menu: %s\n", s_oled_ready ? "ready" : "not detected");
    if (s_oled_ready)
    {
        display_printf("[Boot] OLED initialized\n");
        oled_menu_process();
    }

    seg_write_bcd(0u);
    bar_show(0u);

    g_state = ST_BOOT_SELFTEST;
    show_big_state(g_state);
    oled_sync_status();

    self_test_or_fault();
    if (g_state != ST_FAULT)
    {
        load_config_ram();
    }
    if (g_state != ST_FAULT)
    {
        calibration_check();
    }
    if (g_state == ST_FAULT)
    {
        display_printf("[Boot] FAULT state\n");
    }
}

void app_logic_process(void)
{
    btn_update_hold(&b1);
    btn_update_hold(&b2);
    btn_update_hold(&b3);
    btn_update_hold(&b4);

    if ((g_state == ST_IDLE_STANDBY || g_state == ST_MONITOR) &&
        btn_take_long(&b4))
    {
        set_state(ST_CAL_MENU, "global long press");
    }

    switch (g_state)
    {
    case ST_IDLE_STANDBY:
    {
        bar_show(0u);
        if (btn_take_short(&b1))
        {
            below_low_accum = 0u;
            alert_grace_until = 0u;
            if (g_ms >= PERIODIC_TICK_MS)
            {
                last_periodic = g_ms - PERIODIC_TICK_MS;
            }
            else
            {
                last_periodic = 0u;
            }
            set_state(ST_MONITOR, "manual start");
        }
        else if ((g_ms - last_periodic) >= PERIODIC_TICK_MS)
        {
            below_low_accum = 0u;
            alert_grace_until = 0u;
            if (g_ms >= PERIODIC_TICK_MS)
            {
                last_periodic = g_ms - PERIODIC_TICK_MS;
            }
            else
            {
                last_periodic = 0u;
            }
            set_state(ST_MONITOR, "idle periodic");
        }
    }
    break;

    case ST_MONITOR:
    {
        if ((g_ms - last_periodic) >= PERIODIC_TICK_MS)
        {
            last_periodic = g_ms;
            monitor_tick();
        }
    }
    break;

    case ST_ALERT_VISUAL:
    {
        if ((g_ms - last_blink) >= ALERT_BLINK_MS)
        {
            last_blink = g_ms;
            static uint8_t on = 0;
            on ^= 1;
            bar_show(on ? 4 : 0);
        }
        uint32_t entered = alert_visual_enter;
        if (entered == 0u)
        {
            entered = g_ms;
            alert_visual_enter = entered;
        }
        if (time_is_after_or_equal(g_ms, entered + ALERT_LATCH_DELAY_MS))
        {
            set_state(ST_ALERT_LATCH, "latched");
        }
    }
    break;

    case ST_ALERT_LATCH:
    {
        if (btn_take_short(&b2))
        {
            below_low_accum = 0u;
            alert_grace_until = g_ms + ALERT_POST_ACK_GRACE_MS;
            if (g_ms >= PERIODIC_TICK_MS)
            {
                last_periodic = g_ms - PERIODIC_TICK_MS;
            }
            else
            {
                last_periodic = 0u;
            }
            display_printf("[Alert] Acknowledge -> Monitoring\n");
            set_state(ST_MONITOR, "ack");
        }
        if (btn_take_short(&b3))
        {
            snooze_until = g_ms + SNOOZE_MS;
            display_printf("[Alert] Snooze %lu ms\n", (unsigned long)SNOOZE_MS);
            set_state(ST_ALERT_SNOOZE, "snooze");
        }
    }
    break;

    case ST_ALERT_SNOOZE:
    {
        bar_show(0);
        if (btn_take_short(&b1))
        {
            snooze_until = g_ms;
        }
        if (snooze_until == 0u || time_is_after_or_equal(g_ms, snooze_until))
        {
            below_low_accum = 0u;
            alert_grace_until = g_ms + ALERT_POST_ACK_GRACE_MS;
            if (g_ms >= PERIODIC_TICK_MS)
            {
                last_periodic = g_ms - PERIODIC_TICK_MS;
            }
            else
            {
                last_periodic = 0u;
            }
            display_printf("[Alert] Snooze over -> Monitoring\n");
            set_state(ST_MONITOR, "snooze done/cancel");
        }
    }
    break;

    case ST_CAL_MENU:
    {
        if (btn_take_short(&b1))
        {
            set_state(ST_CAL_SET_DRY, "cal dry");
            display_printf("[Cal] SetDry\n");
        }
        else if (btn_take_short(&b2))
        {
            set_state(ST_CAL_SET_WET, "cal wet");
            display_printf("[Cal] SetWet\n");
        }
        else if (btn_take_short(&b3))
        {
            set_state(ST_CAL_SET_THRESH, "cal thresh");
            display_printf("[Cal] SetThresh\n");
        }
        else if (btn_take_short(&b4))
        {
            if (g_ms >= PERIODIC_TICK_MS)
            {
                last_periodic = g_ms - PERIODIC_TICK_MS;
            }
            else
            {
                last_periodic = 0u;
            }
            below_low_accum = 0u;
            alert_grace_until = 0u;
            persist_calibration();
            set_state(ST_MONITOR, "cal save");
            display_printf("[Cal] Save -> Monitoring\n");
        }
        else if (btn_take_long(&b4))
        {
            set_state(ST_IDLE_STANDBY, "cal cancel");
            display_printf("[Cal] Cancel -> Idle\n");
        }
    }
    break;

    case ST_CAL_SET_DRY:
    {
        if (!s_cal_session_active || s_cal_session_state != ST_CAL_SET_DRY)
        {
            s_cal_session_active = 1u;
            s_cal_session_state = ST_CAL_SET_DRY;
            s_cal_entry_primary = cal_dry;
            s_cal_fine_primary = 0.0f;
            display_printf("[Cal] Adjust Dry: knob rough, B2+/B3-, B4 save, B1 cancel\n");
        }

        if (btn_take_short(&b2))
        {
            s_cal_fine_primary += CAL_FINE_STEP;
        }
        if (btn_take_short(&b3))
        {
            s_cal_fine_primary -= CAL_FINE_STEP;
        }

        float raw = read_avg(CH_POT, 16);
        v_pot = raw;
        float coarse = clamp01(raw);
        float candidate = coarse + s_cal_fine_primary;
        float max_dry = cal_wet - CAL_MIN_SPAN;
        if (max_dry < 0.0f)
        {
            max_dry = 0.0f;
        }
        candidate = clampf(candidate, 0.0f, clamp01(max_dry));
        s_cal_fine_primary = candidate - coarse;
        cal_dry = candidate;

        if (btn_take_short(&b1))
        {
            cal_dry = s_cal_entry_primary;
            s_cal_session_active = 0u;
            s_cal_session_state = ST_BOOT_SELFTEST;
            set_state(ST_CAL_MENU, "dry cancel");
            display_printf("[Cal] Dry reverted to %.3f\n", (double)cal_dry);
        }
        else if (btn_take_short(&b4))
        {
            calibration_sanitize();
            s_cal_session_active = 0u;
            s_cal_session_state = ST_BOOT_SELFTEST;
            set_state(ST_CAL_MENU, "dry stored");
            display_printf("[Cal] Dry stored at %.3f\n", (double)cal_dry);
        }
    }
    break;

    case ST_CAL_SET_WET:
    {
        if (!s_cal_session_active || s_cal_session_state != ST_CAL_SET_WET)
        {
            s_cal_session_active = 1u;
            s_cal_session_state = ST_CAL_SET_WET;
            s_cal_entry_primary = cal_wet;
            s_cal_fine_primary = 0.0f;
            display_printf("[Cal] Adjust Wet: knob rough, B2+/B3-, B4 save, B1 cancel\n");
        }

        if (btn_take_short(&b2))
        {
            s_cal_fine_primary += CAL_FINE_STEP;
        }
        if (btn_take_short(&b3))
        {
            s_cal_fine_primary -= CAL_FINE_STEP;
        }

        float raw = read_avg(CH_POT, 16);
        v_pot = raw;
        float coarse = clamp01(raw);
        float candidate = coarse + s_cal_fine_primary;
        float min_wet = cal_dry + CAL_MIN_SPAN;
        if (min_wet > 1.0f)
        {
            min_wet = 1.0f;
        }
        candidate = clampf(candidate, clamp01(min_wet), 1.0f);
        s_cal_fine_primary = candidate - coarse;
        cal_wet = candidate;

        if (btn_take_short(&b1))
        {
            cal_wet = s_cal_entry_primary;
            s_cal_session_active = 0u;
            s_cal_session_state = ST_BOOT_SELFTEST;
            set_state(ST_CAL_MENU, "wet cancel");
            display_printf("[Cal] Wet reverted to %.3f\n", (double)cal_wet);
        }
        else if (btn_take_short(&b4))
        {
            calibration_sanitize();
            s_cal_session_active = 0u;
            s_cal_session_state = ST_BOOT_SELFTEST;
            set_state(ST_CAL_MENU, "wet stored");
            display_printf("[Cal] Wet stored at %.3f\n", (double)cal_wet);
        }
    }
    break;

    case ST_CAL_SET_THRESH:
    {
        if (!s_cal_session_active || s_cal_session_state != ST_CAL_SET_THRESH)
        {
            s_cal_session_active = 1u;
            s_cal_session_state = ST_CAL_SET_THRESH;
            s_cal_entry_primary = T_low;
            s_cal_entry_secondary = T_high;
            s_cal_span = clampf(T_high - T_low, CAL_THRESH_MIN_WINDOW, 1.0f);
            s_cal_fine_primary = 0.0f;
            display_printf("[Cal] Adjust Threshold: knob rough center, B2+/B3-, B4 save, B1 cancel\n");
        }

        if (btn_take_short(&b2))
        {
            s_cal_fine_primary += CAL_FINE_STEP;
        }
        if (btn_take_short(&b3))
        {
            s_cal_fine_primary -= CAL_FINE_STEP;
        }

        float raw = read_avg(CH_POT, 16);
        v_pot = raw;
        float coarse_center = clamp01(normalize_moisture(raw));
        float half_span = clampf(s_cal_span * 0.5f, CAL_THRESH_MIN_WINDOW * 0.5f, 0.5f);

        float center = coarse_center + s_cal_fine_primary;
        const float min_center = half_span;
        const float max_center = 1.0f - half_span;
        center = clampf(center, min_center, max_center);
        s_cal_fine_primary = center - coarse_center;

        T_low = clamp01(center - half_span);
        T_high = clamp01(center + half_span);
        s_cal_span = clampf(T_high - T_low, CAL_THRESH_MIN_WINDOW, 1.0f);

        if (btn_take_short(&b1))
        {
            T_low = s_cal_entry_primary;
            T_high = s_cal_entry_secondary;
            s_cal_session_active = 0u;
            s_cal_session_state = ST_BOOT_SELFTEST;
            set_state(ST_CAL_MENU, "thr cancel");
            display_printf("[Cal] Thresholds reverted L=%.2f H=%.2f\n", (double)T_low, (double)T_high);
        }
        else if (btn_take_short(&b4))
        {
            calibration_sanitize();
            s_cal_session_active = 0u;
            s_cal_session_state = ST_BOOT_SELFTEST;
            set_state(ST_CAL_MENU, "threshold stored");
            display_printf("[Cal] Thresholds stored L=%.2f H=%.2f\n", (double)T_low, (double)T_high);
        }
    }
    break;

    case ST_BOOT_SELFTEST:
        self_test_or_fault();
        break;

    case ST_BOOT_LOADCFG:
        load_config_ram();
        break;

    case ST_BOOT_CALCHECK:
        calibration_check();
        break;

    case ST_FAULT:
    {
        static uint8_t b1_latched = 0u;
        static uint8_t b2_latched = 0u;
        b1_latched |= btn_take_long(&b1);
        b2_latched |= btn_take_long(&b2);
        if (b1_latched && b2_latched)
        {
            b1_latched = 0u;
            b2_latched = 0u;
            below_low_accum = 0u;
            display_printf("[FAULT] clear request -> system reset\n");
            NVIC_SystemReset();
        }
    }
    break;

    default:
        set_state(ST_FAULT, "unexpected state");
        break;
    }

    if (s_oled_ready)
    {
        float soil_moisture = normalize_moisture(v_soil_moisture);
        float low = T_low;
        float high = T_high;

        bool metrics_changed = false;
        if (f_abs(soil_moisture - s_last_oled_soil) > 0.01f)
        {
            s_last_oled_soil = soil_moisture;
            metrics_changed = true;
        }
        if (f_abs(low - s_last_oled_low) > 0.01f)
        {
            s_last_oled_low = low;
            metrics_changed = true;
        }
        if (f_abs(high - s_last_oled_high) > 0.01f)
        {
            s_last_oled_high = high;
            metrics_changed = true;
        }

        bool keepalive_due = ((g_ms - s_last_oled_sync) >= OLED_KEEPALIVE_MS);
        if (metrics_changed || keepalive_due)
        {
            s_last_oled_sync = g_ms;
            oled_sync_status();
        }

        oled_menu_process();
    }
}

static void btn_exti_service(uint32_t line, btn_t *btn)
{
    uint32_t pending = EXTI->PR & (1u << line);
    if (pending != 0u)
    {
        EXTI->PR = pending;
        uint32_t now = g_ms;
        if ((now - btn->last_irq_ms) < BTN_DEBOUNCE_MS)
        {
            return;
        }
        btn->last_irq_ms = now;
        btn_handle_edge(btn);
    }
}

void EXTI15_10_IRQHandler(void)
{
    btn_exti_service(BTN1_PIN, &b1);
}

void EXTI9_5_IRQHandler(void)
{
    btn_exti_service(BTN3_PIN, &b3);
}

void EXTI4_IRQHandler(void)
{
    btn_exti_service(BTN4_PIN, &b4);
}

void EXTI3_IRQHandler(void)
{
    btn_exti_service(BTN2_PIN, &b2);
}
