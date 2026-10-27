/*
 * Adaptive Traffic Signal - Step 5
 * 4 lanes, 8 IR sensors (IN + OUT per lane), 4 pedestrian buttons,
 * 4 emergency buttons.   STM32F103C8T6, register-level (no HAL)
 *
 * NOT YET COMPILED OR FLASHED - build it first and report any errors.
 *
 * Green time when a lane turns green:
 *     green = MIN + queue * MS_PER_VEHICLE            (5 s ... 15 s)
 *     if that lane's pedestrian button was pushed:   green >= 7 s
 *     if it was pushed three times together:         green >= 12 s
 * Emergency button of lane E:
 *     current green lane -> yellow 2 s -> lane E green 12 s
 *     (if lane E is already green, it just stays green for 12 s)
 *
 * PIN MAP
 *   Lane LEDs  (R,Y,G): L1 PA0,PA1,PA2   L2 PA3,PA4,PA5
 *                       L3 PA6,PA7,PB0   L4 PB1,PB10,PB11
 *   IR sensors: s0 PB12 (L1 IN)   s1 PB13 (L2 IN)
 *               s2 PB14 (L1 OUT)  s3 PB15 (L2 OUT)
 *               s4 PA8  (L3 IN)   s5 PA9  (L4 IN)
 *               s6 PA10 (L3 OUT)  s7 PB9  (L4 OUT)
 *   Emergency buttons (to GND): L1 PB3, L2 PB4, L3 PB5, L4 PB6
 *   Pedestrian buttons (to GND): L1 PB7, L2 PB8, L3 PA11, L4 PA15
 *   Heartbeat: PC13 (onboard LED)       Spare: PA12
 */

#include "stm32f1xx.h"
#include <stdint.h>

/* ---------- Pin table ---------- */
typedef struct {
    GPIO_TypeDef *port;
    uint8_t pin;
} PinDef;

#define NUM_LANES 4
#define NO_LANE   0xFF

/* [lane][0=Red, 1=Yellow, 2=Green] */
static const PinDef lane_pins[NUM_LANES][3] = {
    { {GPIOA, 0},  {GPIOA, 1},  {GPIOA, 2}  },  /* Lane 1 */
    { {GPIOA, 3},  {GPIOA, 4},  {GPIOA, 5}  },  /* Lane 2 */
    { {GPIOA, 6},  {GPIOA, 7},  {GPIOB, 0}  },  /* Lane 3 */
    { {GPIOB, 1},  {GPIOB, 10}, {GPIOB, 11} }   /* Lane 4 */
};

enum { RED = 0, YELLOW = 1, GREEN = 2 };

/* ---------- Timing (ms) - tune these for your demo ---------- */
#define YELLOW_MS                 2000u
#define ALL_RED_MS                1000u   /* safety gap between lanes      */

#define MIN_GREEN_MS              5000u   /* green with an empty queue     */
#define MAX_GREEN_MS              15000u  /* green is never longer         */
#define MS_PER_VEHICLE            2000u   /* extra green per queued vehicle */
                                          /* (placeholder - tune on demo)  */

#define PED_MIN_GREEN_MS          7000u   /* pedestrian button pushed      */
#define PED_ELDERLY_MIN_GREEN_MS  12000u  /* pushed three times together   */
#define PED_MULTI_COUNT           3u      /* presses that make elderly mode */
#define PED_MULTI_WINDOW_MS       3000u   /* all presses within this time  */

#define EMERGENCY_GREEN_MS        12000u  /* emergency lane green time     */

#define RECOVERY_BONUS_MS         3000u   /* extra green after an emergency */
#define RECOVERY_GREENS           8u      /* 2 full cycles; set 0 to disable */

#define BTN_DEBOUNCE_MS           50u     /* ignore button edges closer    */

/* ---------- IR sensor settings ---------- */
/* Most LM393 IR modules pull OUT LOW when an object is detected.
 * If your modules do the opposite, change this to 0. */
#define IR_ACTIVE_LOW    1

#define IR_SAMPLE_MS     20u     /* sampling period                    */
#define IR_DEBOUNCE_N    3u      /* samples that must agree (3x20=60ms)*/

#define NUM_SENSORS      8
#define NO_SENSOR        0xFF
#define QUEUE_MAX        20      /* queue never counts above this      */

/* Sensor number -> pin */
static const PinDef ir_pins[NUM_SENSORS] = {
    {GPIOB, 12}, {GPIOB, 13}, {GPIOB, 14}, {GPIOB, 15},   /* s0..s3 */
    {GPIOA, 8},  {GPIOA, 9},  {GPIOA, 10}, {GPIOB, 9}     /* s4..s7 */
};

/* Which sensor is each lane's IN and OUT sensor */
static const uint8_t lane_in_sensor[NUM_LANES]  = { 0, 1, 4, 5 };
static const uint8_t lane_out_sensor[NUM_LANES] = { 2, 3, 6, 7 };

/* ---------- Buttons (all use EXTI interrupts, active-low) ---------- */
typedef enum { BTN_EMERGENCY, BTN_PED } BtnKind;

typedef struct {
    GPIO_TypeDef *port;
    uint8_t       pin;
    BtnKind       kind;
    uint8_t       lane;
} ButtonDef;

#define NUM_BUTTONS 8
static const ButtonDef buttons[NUM_BUTTONS] = {
    { GPIOB, 3,  BTN_EMERGENCY, 0 },
    { GPIOB, 4,  BTN_EMERGENCY, 1 },
    { GPIOB, 5,  BTN_EMERGENCY, 2 },
    { GPIOB, 6,  BTN_EMERGENCY, 3 },
    { GPIOB, 7,  BTN_PED,       0 },
    { GPIOB, 8,  BTN_PED,       1 },
    { GPIOA, 11, BTN_PED,       2 },
    { GPIOA, 15, BTN_PED,       3 }
};

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
typedef enum { PH_GREEN, PH_YELLOW, PH_ALL_RED, PH_EM_GREEN } Phase;

static Phase    phase       = PH_ALL_RED;
static uint8_t  active_lane = NUM_LANES - 1;   /* first advance -> lane 0 */
static uint32_t phase_start = 0;
static uint32_t phase_len   = ALL_RED_MS;

static uint8_t  em_next       = NO_LANE;  /* emergency lane to serve after yellow */
static uint8_t  recovery_left = 0;        /* greens still getting the bonus       */

/* Seconds left in the current phase (for LCD / 7-segment later) */
static volatile uint32_t countdown_s = 0;

/* ---------- Button flags (set in interrupts, used in main loop) ---------- */
static volatile uint8_t em_req[NUM_LANES];      /* emergency requested       */
static volatile uint8_t ped_req[NUM_LANES];     /* pedestrian button pushed  */
static volatile uint8_t ped_elderly[NUM_LANES]; /* pushed 3x together        */
static uint8_t  ped_count[NUM_LANES];           /* used only inside the ISR  */
static uint32_t ped_first_ms[NUM_LANES];        /* time of first press       */
static uint32_t btn_last_ms[NUM_BUTTONS];       /* debounce timestamps       */

/* Called from the interrupt: counts presses, detects "three together" */
static void ped_press(uint8_t lane, uint32_t now)
{
    if (ped_count[lane] == 0 || (now - ped_first_ms[lane]) > PED_MULTI_WINDOW_MS) {
        ped_count[lane]    = 1;          /* start a new press sequence */
        ped_first_ms[lane] = now;
    } else {
        ped_count[lane]++;
    }
    if (ped_count[lane] >= PED_MULTI_COUNT) {
        ped_elderly[lane] = 1;           /* third press: elderly mode  */
        ped_count[lane]   = 0;
    }
    ped_req[lane] = 1;
}

/* Handles every pending button interrupt (shared by all EXTI handlers) */
static void exti_service(void)
{
    uint32_t pending = EXTI->PR;
    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        uint32_t bit = 1u << buttons[i].pin;
        if (!(pending & bit)) continue;

        EXTI->PR = bit;                              /* clear (write 1)   */

        uint32_t now = ms_ticks;
        if ((now - btn_last_ms[i]) < BTN_DEBOUNCE_MS) continue;  /* bounce */
        btn_last_ms[i] = now;

        if (buttons[i].kind == BTN_EMERGENCY) em_req[buttons[i].lane] = 1;
        else                                  ped_press(buttons[i].lane, now);
    }
}

void EXTI3_IRQHandler(void)     { exti_service(); }   /* PB3            */
void EXTI4_IRQHandler(void)     { exti_service(); }   /* PB4            */
void EXTI9_5_IRQHandler(void)   { exti_service(); }   /* PB5 PB6 PB7 PB8 */
void EXTI15_10_IRQHandler(void) { exti_service(); }   /* PA11 PA15      */

static void buttons_init(void)
{
    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        uint8_t pin = buttons[i].pin;
        gpio_set_input_pullup(buttons[i].port, pin);

        /* Route this pin to its EXTI line: EXTICR[pin/4], 4 bits per pin,
         * port code A=0, B=1 */
        uint8_t idx = pin >> 2;
        uint8_t sh  = (uint8_t)((pin & 3u) * 4u);
        AFIO->EXTICR[idx] &= ~(0xFu << sh);
        AFIO->EXTICR[idx] |= ((buttons[i].port == GPIOB) ? 1u : 0u) << sh;

        EXTI->FTSR |= (1u << pin);   /* trigger on falling edge (press)   */
        EXTI->PR    = (1u << pin);   /* clear any stale flag              */
        EXTI->IMR  |= (1u << pin);   /* unmask the interrupt              */
    }
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* ---------- IR vehicle counting ---------- */
static uint8_t ir_stable[NUM_SENSORS];     /* debounced sensor state       */
static uint8_t queue_len[NUM_LANES];       /* vehicles waiting, per lane   */

/* Raw sensor reading: 1 = vehicle present */
static inline uint8_t ir_raw(uint8_t sensor)
{
    const PinDef *p = &ir_pins[sensor];
    uint8_t bit = (p->port->IDR >> p->pin) & 1u;
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

/* ---------- Pedestrian requests (main-loop side) ---------- */

/* Minimum green this lane's pedestrian needs (0 = none). Also clears the
 * request when 'clear' is non-zero. Interrupts are blocked for a moment so
 * the button ISR cannot change the flags halfway through. */
static uint32_t ped_need_ms(uint8_t lane, uint8_t clear)
{
    uint32_t need = 0;
    __disable_irq();
    if (ped_elderly[lane])   need = PED_ELDERLY_MIN_GREEN_MS;
    else if (ped_req[lane])  need = PED_MIN_GREEN_MS;
    if (clear) { ped_req[lane] = 0; ped_elderly[lane] = 0; }
    __enable_irq();
    return need;
}

/* Called when a lane turns green: queue (+ pedestrian) -> green time */
static uint32_t compute_green_ms(uint8_t lane)
{
    uint32_t ms = MIN_GREEN_MS + (uint32_t)queue_len[lane] * MS_PER_VEHICLE;

    if (recovery_left > 0) {              /* after an emergency: longer green */
        ms += RECOVERY_BONUS_MS;
        recovery_left--;
    }
    if (ms > MAX_GREEN_MS) ms = MAX_GREEN_MS;

    uint32_t need = ped_need_ms(lane, 1); /* pedestrian waiting: guarantee it */
    if (ms < need) ms = need;
    return ms;
}

/* Button pressed while this lane is ALREADY green: lengthen the green only
 * if the minimum time has not been reached yet; a late press waits for
 * the next green of this lane. */
static void ped_update_during_green(uint32_t elapsed)
{
    uint32_t need = ped_need_ms(active_lane, 0);       /* peek only */
    if (need == 0 || elapsed >= need) return;
    if (phase_len < need) phase_len = need;
    (void)ped_need_ms(active_lane, 1);                 /* now clear  */
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

    case PH_EM_GREEN:
        all_lanes_red();
        lane_set(active_lane, GREEN);
        phase_len     = EMERGENCY_GREEN_MS;
        recovery_left = RECOVERY_GREENS;   /* recovery starts after this */
        break;
    }
}

/* Looks at emergency requests set by the interrupts */
static void check_emergency(void)
{
    uint8_t lane = NO_LANE;
    for (uint8_t l = 0; l < NUM_LANES; l++) {
        if (em_req[l]) { lane = l; break; }       /* lowest lane first */
    }
    if (lane == NO_LANE) return;

    switch (phase) {
    case PH_EM_GREEN:
        if (lane == active_lane) {                /* vehicle still coming: */
            em_req[lane] = 0;                     /* restart the 12 s      */
            phase_start  = ms_ticks;
        }
        break;                                    /* other lanes wait      */

    case PH_GREEN:
        em_req[lane] = 0;
        if (lane == active_lane) {
            enter_phase(PH_EM_GREEN);             /* already green: hold   */
        } else {
            em_next = lane;                       /* yellow, then lane E   */
            enter_phase(PH_YELLOW);
        }
        break;

    case PH_YELLOW:                               /* yellow already running */
        if (em_next == NO_LANE) {
            em_next      = lane;
            em_req[lane] = 0;
        }
        break;

    case PH_ALL_RED:
        em_req[lane] = 0;
        active_lane  = lane;
        enter_phase(PH_EM_GREEN);
        break;
    }
}

static void signal_update(void)
{
    uint32_t elapsed = ms_ticks - phase_start;   /* wrap-safe (unsigned) */

    if (phase == PH_GREEN) ped_update_during_green(elapsed);

    if (elapsed >= phase_len) {
        switch (phase) {
        case PH_GREEN:
        case PH_EM_GREEN:
            enter_phase(PH_YELLOW);
            break;
        case PH_YELLOW:
            if (em_next != NO_LANE) {             /* straight to emergency */
                active_lane = em_next;
                em_next     = NO_LANE;
                enter_phase(PH_EM_GREEN);
            } else {
                enter_phase(PH_ALL_RED);
            }
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
    /* Clocks: GPIOA, GPIOB, GPIOC and AFIO (needed for EXTI + JTAG setting) */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

    /* Free PB3, PB4, PA15 from JTAG but KEEP SWD (SWJ_CFG = 0b010) so the
     * ST-Link still works. Never write 0b100 here: it disables SWD too. */
    AFIO->MAPR = (AFIO->MAPR & ~(0x7u << 24)) | (0x2u << 24);

    /* 12 traffic LEDs */
    for (uint8_t l = 0; l < NUM_LANES; l++)
        for (uint8_t c = 0; c < 3; c++)
            gpio_set_output(lane_pins[l][c].port, lane_pins[l][c].pin);

    /* 8 IR sensors as inputs with pull-up; start from their CURRENT state so
     * a sensor already blocked at power-up is not counted as a vehicle */
    for (uint8_t s = 0; s < NUM_SENSORS; s++) {
        gpio_set_input_pullup(ir_pins[s].port, ir_pins[s].pin);
        ir_stable[s] = ir_raw(s);
    }

    /* Onboard LED PC13 (active-low) used as a heartbeat */
    gpio_set_output(GPIOC, 13);

    /* 1 ms tick */
    SysTick_Config(SystemCoreClock / 1000u);

    /* 8 buttons with interrupts */
    buttons_init();

    all_lanes_red();
    enter_phase(PH_ALL_RED);

    uint32_t last_beat = 0;

    while (1) {
        ir_update();
        check_emergency();
        signal_update();

        /* Heartbeat: toggles every 250 ms (proves loop is non-blocking) */
        if ((ms_ticks - last_beat) >= 250u) {
            last_beat = ms_ticks;
            GPIOC->ODR ^= (1u << 13);
        }

        /* --- Future hooks ---
         * lcd_update();   // show active_lane, phase, countdown_s
         * display/pedestrian-LED/buzzer outputs
         */
    }
}
