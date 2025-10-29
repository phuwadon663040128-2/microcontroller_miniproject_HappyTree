/*
 * STM32F411RE + Training Shield
 * UART console version (no OLED yet)
 * Toolchain: CubeIDE / arm-none-eabi-gcc
 *
 * Pins (shield fixed):

        if (g_oled_ready) {
            oled_sync_status();
            oled_menu_process();
        }
 *  - 7-seg BCD: PC7 (A), PA8 (B), PB10 (C), PA9 (D)
 *  - LEDs:      PA5, PA6, PA7, PB6
 *  - Buttons:   PA10, PB3, PB5, PB4   (active LOW, pull-up)
 *  - ADC:       PA0=temp (CH0), PA1=ldr (CH1), PB0=soil (CH8), PA4=pot (CH4)
 *  - UART2:     PA2=TX, PA3=RX (AF7)
 *  - I2C:     PB8=SCL, PB9=SDA (AF4)  
 */

#ifndef STM32F411xE
#define STM32F411xE
#endif

#include <stdio.h>
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "display.h"
#include "board_pins.h"
#include "app_logic.h"
#include "telemetry_uart.h"
#include <string.h>

#include <stdbool.h>
#include <limits.h>

#ifndef HSI_VALUE
#define HSI_VALUE 16000000u
#endif

#ifndef HSE_VALUE
#define HSE_VALUE 8000000u
#endif

#ifndef ENABLE_IWDG
#define ENABLE_IWDG 1
#endif

/* Timebase tick = 1 kHz */
#define TICK_HZ 1000u
#define ADC_STARTUP_DELAY_MS 2u
#define ENABLE_DMA2_CLOCK()   (RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN)
#define ADC_DMA_NUM_CHANNELS  4u

volatile uint16_t g_adc_dma_raw[ADC_DMA_NUM_CHANNELS] = {0u};
volatile uint32_t g_adc_dma_seq = 0u;
/* Monotonic millisecond counter driven by a 1 kHz hardware timer interrupt. */
volatile uint32_t g_ms = 0;       // incremented by TIM2_IRQHandler 
extern uint32_t SystemCoreClock;  // defined in system_stm32f4xx.c

/* -------------------- RCC clock enables (AHB1/APB1/APB2) -------------------- */
#define ENABLE_GPIO_CLOCKS() (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN)
#define ENABLE_ADC_CLOCK()   (RCC->APB2ENR |= RCC_APB2ENR_ADC1EN)

/* -------------------- Timebase -------------------- */
/* Busy-wait delay that uses the millisecond counter instead of hardware timers. */
static inline void delay_ms(uint32_t ms) {
    uint32_t start = g_ms;
    while ((g_ms - start) < ms) { __NOP(); }
}

static uint32_t rcc_apb_prescaler_decode(uint32_t bits);
static uint32_t rcc_apb1_freq_hz(void);
static void adc_dma_init(void);
static void iwdg_init(void);
void iwdg_kick(void);

static uint32_t tim2_timebase_clock_hz(void) {
    uint32_t pre_bits = (RCC->CFGR >> 10u) & 0x7u; /* PPRE1[2:0] */
    uint32_t prescaler = rcc_apb_prescaler_decode(pre_bits);
    uint32_t pclk1 = rcc_apb1_freq_hz();
    return (prescaler > 1u) ? (pclk1 * 2u) : pclk1;
}

/* Configure TIM2 to generate a 1 kHz update interrupt. */
static void tim_timebase_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR; /* ensure the write completes before configuring */

    TIM2->CR1 = 0u;
    TIM2->CR2 = 0u;
    TIM2->SMCR = 0u;
    TIM2->DIER = 0u;
    TIM2->SR = 0u;

    const uint32_t tim_clk = tim2_timebase_clock_hz();
    uint32_t prescaler = (tim_clk + 999999u) / 1000000u; /* target ~1 MHz timer clock */
    if (prescaler == 0u) { prescaler = 1u; }
    uint32_t timer_freq = tim_clk / prescaler;
    if (timer_freq == 0u) { timer_freq = 1u; }
    uint32_t reload = (timer_freq + (TICK_HZ / 2u)) / TICK_HZ;
    if (reload == 0u) { reload = 1u; }

    TIM2->PSC = (uint16_t)(prescaler - 1u);
    TIM2->ARR = (uint32_t)(reload - 1u);
    TIM2->CNT = 0u;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0u;
    TIM2->DIER = TIM_DIER_UIE;

    NVIC_SetPriority(TIM2_IRQn, 0u);
    NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->CR1 = TIM_CR1_CEN;
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        g_ms++;
    }
}

void DMA2_Stream0_IRQHandler(void) {
    uint32_t isr = DMA2->LISR;
    uint32_t clear = 0u;

    if (isr & DMA_LISR_TCIF0) {
        clear |= DMA_LIFCR_CTCIF0;
        g_adc_dma_seq++;
    }
    if (isr & DMA_LISR_HTIF0) { clear |= DMA_LIFCR_CHTIF0; }
    if (isr & DMA_LISR_TEIF0) { clear |= DMA_LIFCR_CTEIF0; }
    if (isr & DMA_LISR_DMEIF0){ clear |= DMA_LIFCR_CDMEIF0; }
    if (isr & DMA_LISR_FEIF0) { clear |= DMA_LIFCR_CFEIF0; }

    if (clear != 0u) {
        DMA2->LIFCR = clear;
    }
}

/* -------------------- Clock helpers -------------------- */
/* Helper: translate encoded prescaler bits into the actual divider value. */
static uint32_t rcc_apb_prescaler_decode(uint32_t bits){
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

/* Helper: map the AHB prescaler bitfield to a divider (per datasheet table). */
static inline uint32_t ahb_prescaler_decode(uint32_t bits){
    static const uint16_t table[16] = {1u,1u,1u,1u,1u,1u,1u,1u,2u,4u,8u,16u,64u,128u,256u,512u};
    return table[bits & 0xFu];
}

/* Read RCC registers to figure out the current APB1 (peripheral) clock rate. */
static uint32_t rcc_apb1_freq_hz(void){
    uint32_t pre_bits = (RCC->CFGR >> 10u) & 0x7u; /* PPRE1[2:0] at bits 12:10 */
    uint32_t prescaler = rcc_apb_prescaler_decode(pre_bits);
    if (prescaler == 0u) { prescaler = 1u; }
    return SystemCoreClock / prescaler;
}

/* Inspect RCC state to determine the core clock frequency after boot. */
static uint32_t compute_sysclk_hz(void){
    uint32_t sws = (RCC->CFGR & RCC_CFGR_SWS_Msk) >> RCC_CFGR_SWS_Pos;
    uint32_t base_hz;
    switch (sws) {
        case RCC_CFGR_SWS_HSI:
            base_hz = HSI_VALUE;
            break;
        case RCC_CFGR_SWS_HSE:
            base_hz = HSE_VALUE;
            break;
        case RCC_CFGR_SWS_PLL: {
            uint32_t cfgr = RCC->PLLCFGR;
            uint32_t pllm = cfgr & 0x3Fu;
            uint32_t plln = (cfgr >> 6) & 0x1FFu;
            uint32_t pllp = ((((cfgr >> 16) & 0x3u) + 1u) * 2u);
            uint32_t pllsrc = (cfgr & RCC_PLLCFGR_PLLSRC) ? HSE_VALUE : HSI_VALUE;
            if (pllm == 0u) { pllm = 1u; }
            if (pllp == 0u) { pllp = 2u; }
            uint64_t vco = ((uint64_t)pllsrc * (uint64_t)plln) / (uint64_t)pllm;
            base_hz = (uint32_t)(vco / (uint64_t)pllp);
            break;
        }
        default:
            base_hz = HSI_VALUE;
            break;
    }

    uint32_t hpre_bits = (RCC->CFGR >> 4u) & 0xFu;
    uint32_t ahb_presc = ahb_prescaler_decode(hpre_bits);
    if (ahb_presc == 0u) { ahb_presc = 1u; }
    return base_hz / ahb_presc;
}

/* Enable the single-precision floating-point unit so math code runs quickly. */
static inline void fpu_enable(void) {
    SCB->CPACR |= (0xFu << 20);  // full access to CP10, CP11
    __DSB(); __ISB();
}

/* Configure every GPIO pin used by the shield (LEDs, buttons, ADC, etc.). */
static void gpio_init(void) {
    ENABLE_GPIO_CLOCKS();

    /* 7-seg pins to output push-pull */
    /* PC7, PC6 (note: PC6 unused, left as output low for compatibility) */
    GPIOC->MODER  &= ~(GPIO_MODER_MODER7 | GPIO_MODER_MODER6);
    GPIOC->MODER  |=  (GPIO_MODER_MODER7_0 | GPIO_MODER_MODER6_0);
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_6);
    GPIOC->PUPDR  &= ~(GPIO_PUPDR_PUPDR7 | GPIO_PUPDR_PUPDR6);

    /* PA8, PA9 */
    GPIOA->MODER  &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOA->MODER  |=  (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
    GPIOA->PUPDR  &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);

    /* PB10 */
    GPIOB->MODER  &= ~(GPIO_MODER_MODER10);
    GPIOB->MODER  |=  (GPIO_MODER_MODER10_0);
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_10);
    GPIOB->PUPDR  &= ~(GPIO_PUPDR_PUPDR10);

    /* LEDs: PA5,6,7; PB6 */
    GPIOA->MODER  &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER  |=  (GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_5 | GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7);
    GPIOA->PUPDR  &= ~(GPIO_PUPDR_PUPDR5 | GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);

    GPIOB->MODER  &= ~(GPIO_MODER_MODER6);
    GPIOB->MODER  |=  (GPIO_MODER_MODER6_0);
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_6);
    GPIOB->PUPDR  &= ~(GPIO_PUPDR_PUPDR6);

    /* Buttons: inputs + pull-ups */
    GPIOA->MODER  &= ~(GPIO_MODER_MODER10);
    GPIOA->PUPDR  &= ~(GPIO_PUPDR_PUPDR10);
    GPIOA->PUPDR  |=  (GPIO_PUPDR_PUPDR10_0);

    GPIOB->MODER  &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIOB->PUPDR  &= ~(GPIO_PUPDR_PUPDR3 | GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5);
    GPIOB->PUPDR  |=  (GPIO_PUPDR_PUPDR3_0 | GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0);

    /* ADC pins PA0, PA1, PB0, PA4 = analog mode, no pull */
    GPIOA->MODER |= (GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER4);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR4);
    GPIOB->MODER |= GPIO_MODER_MODER0;
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0);
}

/* -------------------- ADC: continuous scan with DMA -------------------- */
static void adc_dma_init(void) {
    ENABLE_DMA2_CLOCK();

    if (DMA2_Stream0->CR & DMA_SxCR_EN) {
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        while (DMA2_Stream0->CR & DMA_SxCR_EN) { __NOP(); }
    }

    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 |
                  DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;

    DMA2_Stream0->CR = 0u;
    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)g_adc_dma_raw;
    DMA2_Stream0->NDTR = ADC_DMA_NUM_CHANNELS;
    DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 |
                        DMA_SxCR_MINC   | DMA_SxCR_CIRC    |
                        DMA_SxCR_PL_1   | DMA_SxCR_TCIE;
    DMA2_Stream0->FCR = 0u; /* direct mode */

    NVIC_SetPriority(DMA2_Stream0_IRQn, 5u);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void adc_init(void) {
    ENABLE_ADC_CLOCK();
    adc_dma_init();

    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->CR2 = ADC_CR2_ADON;

    const uint32_t SMP_84CYC = 4u;
    ADC1->SMPR2 =
        ((SMP_84CYC << (3u * CH_TEMP)) |
         (SMP_84CYC << (3u * CH_LDR))  |
         (SMP_84CYC << (3u * CH_SOIL)) |
         (SMP_84CYC << (3u * CH_POT)));

    ADC1->SQR1 = ((ADC_DMA_NUM_CHANNELS - 1u) << 20u);
    ADC1->SQR2 = 0u;
    ADC1->SQR3 = (CH_TEMP & 0x1Fu) |
                 ((CH_LDR & 0x1Fu) << 5u) |
                 ((CH_SOIL & 0x1Fu) << 10u) |
                 ((CH_POT & 0x1Fu) << 15u);

    ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_CONT;

    g_adc_dma_seq = 0u;
    memset((void*)g_adc_dma_raw, 0, sizeof(g_adc_dma_raw));

    delay_ms(ADC_STARTUP_DELAY_MS);

    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 |
                  DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;
    DMA2_Stream0->NDTR = ADC_DMA_NUM_CHANNELS;
    DMA2_Stream0->CR  |= DMA_SxCR_EN;

    ADC1->CR2 |= ADC_CR2_SWSTART;
}

#if ENABLE_IWDG
static void iwdg_init(void) {
    RCC->CSR |= RCC_CSR_LSION;
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0u) { __NOP(); }

    IWDG->KR = 0x5555u;
    IWDG->PR = 0x04u;     /* prescaler /64 */
    IWDG->KR = 0x5555u;
    IWDG->RLR = 1024u;    /* ~2 s at 32 kHz LSI */
    IWDG->KR = 0xAAAAu;   /* reload */
    IWDG->KR = 0xCCCCu;   /* start */
}

void iwdg_kick(void) {
    IWDG->KR = 0xAAAAu;
}
#else
static inline void iwdg_init(void) { }
void iwdg_kick(void) { }
#endif


/* -------------------- Main -------------------- */
int main(void)
{
    /* Bring up the core peripherals that the firmware depends on. */
    fpu_enable();
    uint32_t reset_flags = RCC->CSR;
    uint32_t had_iwdg = (reset_flags & RCC_CSR_IWDGRSTF) ? 1u : 0u;
    uint32_t had_pin = (reset_flags & RCC_CSR_PINRSTF) ? 1u : 0u;
    uint32_t had_por = (reset_flags & RCC_CSR_PORRSTF) ? 1u : 0u;
    RCC->CSR |= RCC_CSR_RMVF;
    display_uart_printf("[Boot] ResetCause: IWDG=%lu, PIN=%lu, POR/PDR=%lu\n",
                        (unsigned long)had_iwdg,
                        (unsigned long)had_pin,
                        (unsigned long)had_por);
    SystemCoreClock = compute_sysclk_hz();
    tim_timebase_init();
    gpio_init();
    adc_init();
    /* Display abstraction keeps UART today but allows a drop-in OLED driver later. */
    /* Route log output to the UART helper so we can watch the state machine. */
    const display_config_t display_cfg = {
        .backend = DISPLAY_BACKEND_UART_STREAM,
        .uart_baud = 115200u
    };
    display_init(&display_cfg);
    display_printf("[Boot] Display backend: %s\n", display_backend_name(display_current_backend()));

#if TELEMETRY_UART_ENABLE
    telemetry_uart_init(115200u);
    if (telemetry_uart_is_ready()) {
        display_printf("[Boot] Telemetry UART ready on USART6 (PA11->TX, PA12<-RX)\n");
    } else {
        display_printf("[Boot] Telemetry UART init failed\n");
    }
#else
    display_printf("[Boot] Telemetry UART disabled (TELEMETRY_UART_ENABLE=0)\n");
#endif

    /* Banner */
    display_printf("\n\n[Boot] Moisture FSM (UART console, no WiFi)\n");
    display_printf("[Boot] SystemCoreClock=%lu Hz PCLK1=%lu Hz\n",
                   (unsigned long)SystemCoreClock,
                   (unsigned long)rcc_apb1_freq_hz());

    app_logic_init();

    iwdg_init();
    if (!ENABLE_IWDG) {
        display_printf("[Boot] Watchdog disabled (ENABLE_IWDG=0)\n");
    }

    display_printf("[Boot] Init done, starting FSM...\n");

    while (1)
    {
        app_logic_process();
        iwdg_kick();
        __WFI();
    }
}
