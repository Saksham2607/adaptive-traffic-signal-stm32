/*
 * Adaptive Traffic Signal - Step 3: IR TIME-OCCUPANCY green time
 * STM32F103C8T6, register-level (no HAL)
 *
 * Method: while a lane is waiting (red), its IR sensor is sampled every
 * 20 ms. Occupancy % = (samples with vehicle present) / (all samples) * 100.
 * When the lane turns green:
 *     green = MIN + occupancy% * (MAX - MIN) / 100
 * then its statistics reset. A sensor that is blocked the whole time
 * (queue of stopped cars) therefore gives the maximum green.
 */

#include "stm32f1xx.h"
#include <stdint.h>

/* ---------- Pin table ---------- */
typedef struct {
    GPIO_TypeDef *port;
    uint8_t pin;
} PinDef;

#define NUM_LANES 4

/* [lane][0=Red, 1=Yellow, 2=Green] */
static const PinDef lane_pins[NUM_LANES][3] = {
    { {GPIOA, 0},  {GPIOA, 1},  {GPIOA, 2}  },  /* Lane 1 */
    { {GPIOA, 3},  {GPIOA, 4},  {GPIOA, 5}  },  /* Lane 2 */
    { {GPIOA, 6},  {GPIOA, 7},  {GPIOB, 0}  },  /* Lane 3 */
    { {GPIOB, 1},  {GPIOB, 10}, {GPIOB, 11} }   /* Lane 4 */
};

enum { RED = 0, YELLOW = 1, GREEN = 2 };

/* ---------- Timing (ms) - tune these for your demo ---------- */
#define YELLOW_MS        2000u
#define ALL_RED_MS       1000u   /* safety gap between lanes */

#define MIN_GREEN_MS     5000u   /* green at 0 % occupancy             */
#define MAX_GREEN_MS     15000u  /* green at 100 % occupancy           */

/* ---------- IR sensor settings ---------- */
/* Most LM393 IR modules pull OUT LOW when an object is detected.
 * If your modules do the opposite, change this to 0. */
#define IR_ACTIVE_LOW    1

#define IR_SAMPLE_MS     20u     /* sampling period                    */
#define IR_DEBOUNCE_N    3u      /* samples that must agree (3x20=60ms)*/

/* ---------- Millisecond tick ---------- */
static volatile uint32_t ms_ticks = 0;

void SysTick_Handler(void)
{
    ms_ticks++;
}

/* ---------- GPIO helpers (STM32F1: CRL/CRH, not MODER) ---------- */
static void gpio_set_output(GPIO_TypeDef *port, uint8_t pin)
{
    /* MODE=10 (2 MHz output), CNF=00 (push-pull) -> 0b0010 */
    if (pin < 8) {
        port->CRL &= ~(0xFu << (pin * 4));
        port->CRL |=  (0x2u << (pin * 4));
    } else {
        uint8_t p = pin - 8;
        port->CRH &= ~(0xFu << (p * 4));
        port->CRH |=  (0x2u << (p * 4));
    }
}

static void gpio_set_input_pullup(GPIO_TypeDef *port, uint8_t pin)
{
    /* MODE=00 (input), CNF=10 (pull-up/pull-down) -> 0b1000,
     * then ODR bit = 1 selects pull-UP */
    if (pin < 8) {
        port->CRL &= ~(0xFu << (pin * 4));
        port->CRL |=  (0x8u << (pin * 4));
    } else {
        uint8_t p = pin - 8;
        port->CRH &= ~(0xFu << (p * 4));
        port->CRH |=  (0x8u << (p * 4));
    }
    port->ODR |= (1u << pin);
}

static inline void pin_write(const PinDef *p, uint8_t on)
{
    /* BSRR: low 16 bits set, high 16 bits reset (atomic) */
    p->port->BSRR = on ? (1u << p->pin) : (1u << (p->pin + 16));
}

/* Set one lane to exactly one colour */
static void lane_set(uint8_t lane, uint8_t colour)
{
    for (uint8_t c = 0; c < 3; c++) {
        pin_write(&lane_pins[lane][c], c == colour);
    }
}

static void all_lanes_red(void)
{
    for (uint8_t l = 0; l < NUM_LANES; l++) lane_set(l, RED);
}

/* ---------- State machine variables ---------- */
typedef enum { PH_GREEN, PH_YELLOW, PH_ALL_RED } Phase;

static Phase    phase       = PH_ALL_RED;
static uint8_t  active_lane = NUM_LANES - 1;   /* first advance -> lane 0 */
static uint32_t phase_start = 0;
static uint32_t phase_len   = ALL_RED_MS;

/* Seconds left in the current phase (for LCD / 7-segment later) */
static volatile uint32_t countdown_s = 0;

/* ---------- IR density logic ---------- */
static uint32_t occ_samples[NUM_LANES];    /* samples with vehicle present */
static uint32_t wait_samples[NUM_LANES];   /* all samples while waiting    */
static uint8_t  ir_stable[NUM_LANES];      /* debounced sensor state       */
static volatile uint8_t occupancy_pct[NUM_LANES];  /* last result, for LCD */

/* Raw sensor reading: 1 = vehicle present */
static inline uint8_t ir_raw(uint8_t lane)
{
    uint8_t bit = (GPIOB->IDR >> (12 + lane)) & 1u;   /* PB12..PB15 */
    return IR_ACTIVE_LOW ? !bit : bit;
}

/* Sample only while that lane is waiting (not green/yellow).
 * The ALL_RED gap after a lane's own turn counts as waiting again. */
static inline uint8_t lane_is_waiting(uint8_t lane)
{
    return !(lane == active_lane && phase != PH_ALL_RED);
}

static void ir_update(void)
{
    static uint32_t last_sample = 0;
    static uint8_t  agree[NUM_LANES];    /* consecutive differing samples */

    if ((ms_ticks - last_sample) < IR_SAMPLE_MS) return;
    last_sample = ms_ticks;

    for (uint8_t l = 0; l < NUM_LANES; l++) {
        /* debounce */
        uint8_t r = ir_raw(l);
        if (r != ir_stable[l]) {
            if (++agree[l] >= IR_DEBOUNCE_N) {
                ir_stable[l] = r;
                agree[l]     = 0;
            }
        } else {
            agree[l] = 0;
        }

        /* occupancy bookkeeping */
        if (lane_is_waiting(l)) {
            wait_samples[l]++;
            if (ir_stable[l]) occ_samples[l]++;
        }
    }
}

/* Called when a lane turns green: convert occupancy % to green time,
 * then reset that lane's statistics. */
static uint32_t compute_green_ms(uint8_t lane)
{
    uint32_t pct = 0;
    if (wait_samples[lane] > 0) {
        pct = (occ_samples[lane] * 100u) / wait_samples[lane];
    }
    occupancy_pct[lane] = (uint8_t)pct;
    occ_samples[lane]   = 0;
    wait_samples[lane]  = 0;

    return MIN_GREEN_MS + (pct * (MAX_GREEN_MS - MIN_GREEN_MS)) / 100u;
}

/* ---------- State machine ---------- */
static void enter_phase(Phase p)
{
    phase       = p;
    phase_start = ms_ticks;

    switch (p) {
    case PH_GREEN:
        all_lanes_red();
        lane_set(active_lane, GREEN);
        phase_len = compute_green_ms(active_lane);   /* also resets stats */
        break;

    case PH_YELLOW:
        lane_set(active_lane, YELLOW);
        phase_len = YELLOW_MS;
        break;

    case PH_ALL_RED:
        all_lanes_red();
        phase_len = ALL_RED_MS;
        break;
    }
}

static void signal_update(void)
{
    uint32_t elapsed = ms_ticks - phase_start;   /* wrap-safe (unsigned) */

    if (elapsed >= phase_len) {
        switch (phase) {
        case PH_GREEN:
            enter_phase(PH_YELLOW);
            break;
        case PH_YELLOW:
            enter_phase(PH_ALL_RED);
            break;
        case PH_ALL_RED:
            active_lane = (active_lane + 1) % NUM_LANES;
            enter_phase(PH_GREEN);
            break;
        }
        elapsed = 0;
    }

    /* round up so display shows 1 (not 0) during the final second */
    countdown_s = (phase_len - elapsed + 999u) / 1000u;
}

/* ---------- main ---------- */
int main(void)
{
    /* Clocks: GPIOA, GPIOB, GPIOC */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;

    /* 12 traffic LEDs */
    for (uint8_t l = 0; l < NUM_LANES; l++)
        for (uint8_t c = 0; c < 3; c++)
            gpio_set_output(lane_pins[l][c].port, lane_pins[l][c].pin);

    /* 4 IR sensors: PB12..PB15 as inputs with pull-up */
    for (uint8_t l = 0; l < NUM_LANES; l++)
        gpio_set_input_pullup(GPIOB, 12 + l);

    /* Onboard LED PC13 (active-low) used as a heartbeat */
    gpio_set_output(GPIOC, 13);

    /* 1 ms tick */
    SysTick_Config(SystemCoreClock / 1000u);

    all_lanes_red();
    enter_phase(PH_ALL_RED);

    uint32_t last_beat = 0;

    while (1) {
        ir_update();
        signal_update();

        /* Heartbeat: toggles every 250 ms (proves loop is non-blocking) */
        if ((ms_ticks - last_beat) >= 250u) {
            last_beat = ms_ticks;
            GPIOC->ODR ^= (1u << 13);
        }

        /* --- Future hooks ---
         * check_emergency();   // flag set by EXTI ISR
         * check_pedestrian();  // flag set by EXTI ISR
         * lcd_update();        // show active_lane, phase, countdown_s
         */
    }
}
