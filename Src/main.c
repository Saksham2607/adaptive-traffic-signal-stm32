/*
 * Adaptive Traffic Signal - Step 4: IR VEHICLE COUNTING (IN / OUT sensors)
 * STM32F103C8T6, register-level (no HAL)
 *
 * Method: each monitored lane has an IN sensor (start of the stretch) and
 * an OUT sensor (stop line). A vehicle blocking a sensor is counted once,
 * at the moment the sensor goes from clear to blocked.
 *     IN event  -> queue + 1
 *     OUT event -> queue - 1   (never below 0)
 * When the lane turns green:
 *     green = MIN + queue * MS_PER_VEHICLE, capped at MAX
 * Lanes with no sensors always get the minimum green.
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

#define MIN_GREEN_MS     5000u   /* green with an empty queue          */
#define MAX_GREEN_MS     15000u  /* green is never longer than this    */
#define MS_PER_VEHICLE   2000u   /* extra green per queued vehicle     */
                                 /* (placeholder - tune from your demo) */

/* ---------- IR sensor settings ---------- */
/* Most LM393 IR modules pull OUT LOW when an object is detected.
 * If your modules do the opposite, change this to 0. */
#define IR_ACTIVE_LOW    1

#define IR_SAMPLE_MS     20u     /* sampling period                    */
#define IR_DEBOUNCE_N    3u      /* samples that must agree (3x20=60ms)*/

/* Sensor number s is wired to pin PB(12 + s):
 *   sensor 0 = PB12   sensor 1 = PB13   sensor 2 = PB14   sensor 3 = PB15 */
#define NUM_SENSORS      4
#define NO_SENSOR        0xFF
#define QUEUE_MAX        20      /* queue never counts above this      */

/* Which sensor is each lane's IN and OUT sensor (NO_SENSOR = none) */
static const uint8_t lane_in_sensor[NUM_LANES]  = { 0, 1, NO_SENSOR, NO_SENSOR };
static const uint8_t lane_out_sensor[NUM_LANES] = { 2, 3, NO_SENSOR, NO_SENSOR };

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

/* ---------- IR vehicle counting ---------- */
static uint8_t ir_stable[NUM_SENSORS];     /* debounced sensor state       */
static uint8_t queue_len[NUM_LANES];       /* vehicles waiting, per lane   */

/* Raw sensor reading: 1 = vehicle present */
static inline uint8_t ir_raw(uint8_t sensor)
{
    uint8_t bit = (GPIOB->IDR >> (12 + sensor)) & 1u;   /* PB12..PB15 */
    return IR_ACTIVE_LOW ? !bit : bit;
}

/* One vehicle just blocked this sensor: update the lane that owns it */
static void count_event(uint8_t sensor)
{
    for (uint8_t l = 0; l < NUM_LANES; l++) {
        if (lane_in_sensor[l] == sensor) {
            if (queue_len[l] < QUEUE_MAX) queue_len[l]++;
        }
        if (lane_out_sensor[l] == sensor) {
            if (queue_len[l] > 0) queue_len[l]--;
        }
    }
}

static void ir_update(void)
{
    static uint32_t last_sample = 0;
    static uint8_t  agree[NUM_SENSORS];  /* consecutive differing samples */

    if ((ms_ticks - last_sample) < IR_SAMPLE_MS) return;
    last_sample = ms_ticks;

    for (uint8_t s = 0; s < NUM_SENSORS; s++) {
        /* debounce */
        uint8_t r = ir_raw(s);
        if (r != ir_stable[s]) {
            if (++agree[s] >= IR_DEBOUNCE_N) {
                ir_stable[s] = r;
                agree[s]     = 0;
                if (r) count_event(s);   /* clear -> blocked = one vehicle */
            }
        } else {
            agree[s] = 0;
        }
    }
}

/* Called when a lane turns green: queue length -> green time */
static uint32_t compute_green_ms(uint8_t lane)
{
    uint32_t ms = MIN_GREEN_MS + (uint32_t)queue_len[lane] * MS_PER_VEHICLE;
    return (ms > MAX_GREEN_MS) ? MAX_GREEN_MS : ms;
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
        phase_len = compute_green_ms(active_lane);
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
    for (uint8_t s = 0; s < NUM_SENSORS; s++)
        gpio_set_input_pullup(GPIOB, 12 + s);

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
