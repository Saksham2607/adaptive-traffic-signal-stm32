#include "stm32f1xx.h"
#include <stdint.h>

/* ---------- Pin table ---------- */
typedef struct {
    GPIO_TypeDef *port;
    uint8_t pin;
} PinDef;

#define NUM_LANES 4
#define NO_LANE   0xFF
#define UART_BAUD 9600u

/* [lane][0=Red, 1=Yellow, 2=Green] */
static const PinDef lane_pins[NUM_LANES][3] = {
    { {GPIOA, 0},  {GPIOA, 1},  {GPIOA, 2}  },  /* Lane 1 */
    { {GPIOA, 3},  {GPIOA, 4},  {GPIOA, 5}  },  /* Lane 2 */
    { {GPIOA, 6},  {GPIOA, 7},  {GPIOB, 0}  },  /* Lane 3 */
    { {GPIOB, 1},  {GPIOB, 10}, {GPIOB, 11} }   /* Lane 4 */
};

enum { RED = 0, YELLOW = 1, GREEN = 2 };

static const PinDef em_pins[NUM_LANES] = {
    {GPIOB, 3}, {GPIOB, 4}, {GPIOB, 5}, {GPIOB, 6}
};

static const PinDef buzzer_pin = {GPIOA, 11};

/* TM1637 displays: one shared CLK, one DIO per lane.
   Lane 3 DIO is now PA12 (was PC0). */
static const PinDef tm_clk = {GPIOB, 8};
static const PinDef tm_dio[NUM_LANES] = {
    {GPIOB, 9}, {GPIOA, 8}, {GPIOA, 12}, {GPIOA, 10}
};

/* Software I2C pins */
#define SCL_PORT GPIOB
#define SCL_PIN  7
#define SDA_PORT GPIOA
#define SDA_PIN  15

/* I2C device addresses (7-bit) */
#define LCD_ADDR   0x27
#define EXPA_ADDR  0x21
#define EXPB_ADDR  0x20

/* ---------- Timing (ms) ---------- */
#define YELLOW_MS                 2000u
#define ALL_RED_MS                1000u

/* Green time = (cars in lane) x GREEN_PER_CAR_MS, limited to EXT_MIN..EXT_MAX.
   It is calculated once, at the moment the lane turns green. */
#define EXT_MIN_MS                5000u
#define EXT_MAX_MS                15000u
#define GREEN_PER_CAR_MS          1000u

/* Each lane's display starts counting this many seconds before the lane
   turns green. */
#define COUNTDOWN_S               9u
#define COUNTDOWN_MS              (COUNTDOWN_S * 1000u)

/* Last part of every green: pedestrian "don't walk" blinks */
#define PED_CLEAR_MS              3000u

#define PED_MIN_GREEN_MS          7000u
#define PED_ELDERLY_MIN_GREEN_MS  12000u
#define PED_MULTI_COUNT           3u
#define PED_MULTI_WINDOW_MS       3000u

#define EMERGENCY_GREEN_MS        12000u

#define RECOVERY_BONUS_MS         3000u
#define RECOVERY_GREENS           8u

#define BTN_DEBOUNCE_MS           50u
#define BTN_DEBOUNCE_N            2u

#define BEEP_ON_MS                80u
#define BEEP_OFF_MS               120u
#define LCD_BUILD_MS              250u

/* Emergency buzzer pattern: beeps for EM_BEEP_ON_MS out of every
   EM_BEEP_PERIOD_MS while emergency mode is active */
#define EM_BEEP_PERIOD_MS         600u
#define EM_BEEP_ON_MS             400u

/* ---------- IR sensor settings ---------- */
#define IR_ACTIVE_LOW    1
#define IR_SAMPLE_MS     20u
#define IR_DEBOUNCE_N    3u
#define NUM_SENSORS      8
#define QUEUE_MAX        20

static const uint8_t lane_in_sensor[NUM_LANES]  = { 0, 1, 4, 5 };
static const uint8_t lane_out_sensor[NUM_LANES] = { 2, 3, 6, 7 };

/* ---------- Millisecond tick ---------- */
static volatile uint32_t ms_ticks = 0;

void SysTick_Handler(void)
{
    ms_ticks++;
}

static void delay_ms(uint32_t ms)
{
    uint32_t t = ms_ticks;
    while ((ms_ticks - t) < ms) { }
}

/* Rough microsecond delay */
static void udelay(uint32_t us)
{
    volatile uint32_t n = us * (SystemCoreClock / 3000000u) + 1u;
    while (n--) { __NOP(); }
}

/* ---------- GPIO helpers ---------- */
static void gpio_cfg(GPIO_TypeDef *port, uint8_t pin, uint32_t nibble)
{
    if (pin < 8) {
        port->CRL &= ~(0xFu << (pin * 4));
        port->CRL |=  (nibble << (pin * 4));
    } else {
        uint8_t p = pin - 8;
        port->CRH &= ~(0xFu << (p * 4));
        port->CRH |=  (nibble << (p * 4));
    }
}

static void gpio_set_output(GPIO_TypeDef *port, uint8_t pin)
{
    gpio_cfg(port, pin, 0x2u);
}

static void gpio_set_output_od(GPIO_TypeDef *port, uint8_t pin)
{
    gpio_cfg(port, pin, 0x6u);
}

static void gpio_set_input_pullup(GPIO_TypeDef *port, uint8_t pin)
{
    gpio_cfg(port, pin, 0x8u);
    port->ODR |= (1u << pin);
}

static inline void pin_write(const PinDef *p, uint8_t on)
{
    p->port->BSRR = on ? (1u << p->pin) : (1u << (p->pin + 16));
}

static void lane_set(uint8_t lane, uint8_t colour)
{
    for (uint8_t c = 0; c < 3; c++) {
        pin_write(&lane_pins[lane][c], c == colour);
    }
}

static void all_lanes_red(void)
{
    for (uint8_t l = 0; l < NUM_LANES; l++) {
        lane_set(l, RED);
    }
}

/* ---------- State machine variables ---------- */
typedef enum {
    PH_GREEN_EXT,     /* green, pedestrian walk (if requested)           */
    PH_GREEN_CLEAR,   /* last PED_CLEAR_MS of green, don't-walk blinks   */
    PH_YELLOW,
    PH_ALL_RED,
    PH_EM_GREEN
} Phase;

static const char *const phase_names[5] = {
    "GREEN", "CLEAR", "YELLW", "ALLRD", "EMERG"
};

static Phase    phase        = PH_ALL_RED;
static uint8_t  active_lane  = NUM_LANES - 1;
static uint32_t phase_start  = 0;
static uint32_t phase_len    = ALL_RED_MS;

/* Absolute tick at which the current green (normal or emergency) ends */
static uint32_t green_end    = 0;

/* Green time already worked out for the lane that turns green next
   (fixed at the start of the all-red phase before that lane's green) */
static uint32_t next_green_ms = EXT_MIN_MS;

static uint8_t  em_next       = NO_LANE;
static uint8_t  recovery_left = 0;

/* Seconds left in the current phase (shown on the LCD) */
static volatile uint32_t countdown_s = 0;

/* What each lane's TM1637 shows */
#define DISP_BLANK  0xFFu    /* display off                      */
#define DISP_ZERO2  0xFEu    /* shows "00" (during yellow)       */
static uint8_t disp_val[NUM_LANES] = { DISP_BLANK, DISP_BLANK,
                                       DISP_BLANK, DISP_BLANK };

/* Pedestrian state */
static uint8_t  ped_req[NUM_LANES];
static uint8_t  ped_elderly[NUM_LANES];
static uint8_t  ped_count[NUM_LANES];
static uint32_t ped_first_ms[NUM_LANES];
static uint8_t  ped_serving = 0;
static uint8_t  ped_serving_elderly = 0;

/* Emergency flags */
static volatile uint8_t em_req[NUM_LANES];
static uint32_t btn_last_ms[NUM_LANES];

/* IR / queue */
static uint8_t ir_stable[NUM_SENSORS];
static uint8_t queue_len[NUM_LANES];
static uint32_t total_in[NUM_LANES];
static uint32_t total_out[NUM_LANES];
static uint8_t expb_state = 0xFF;

/* Emergency mode is active from the moment the other lanes start
   clearing (yellow) until the emergency green ends */
static inline uint8_t em_active(void)
{
    return (uint8_t)((phase == PH_EM_GREEN) ||
                     (phase == PH_YELLOW && em_next != NO_LANE));
}

/* Milliseconds left in the current green (0 when it has ended) */
static uint32_t green_left_ms(void)
{
    int32_t d = (int32_t)(green_end - ms_ticks);
    return (d > 0) ? (uint32_t)d : 0u;
}

/* ---------- USART1 transmit-only UART ---------- */
static void uart_init(void)
{
    uint32_t ppre2 = (RCC->CFGR >> 11) & 0x7u;
    uint32_t apb2_div;

    switch (ppre2) {
    case 4u: apb2_div = 2u;  break;
    case 5u: apb2_div = 4u;  break;
    case 6u: apb2_div = 8u;  break;
    case 7u: apb2_div = 16u; break;
    default: apb2_div = 1u; break;
    }

    uint32_t pclk2 = SystemCoreClock / apb2_div;

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    gpio_cfg(GPIOA, 9u, 0xBu);

    USART1->CR1 = 0u;
    USART1->CR2 = 0u;
    USART1->CR3 = 0u;
    USART1->BRR = (pclk2 + (UART_BAUD / 2u)) / UART_BAUD;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
}

static void uart_char(char c)
{
    while ((USART1->SR & USART_SR_TXE) == 0u) { }
    USART1->DR = (uint8_t)c;
}

static void uart_text(const char *s)
{
    while (*s) uart_char(*s++);
}

static void uart_number(uint32_t value)
{
    char digits[10];
    uint8_t n = 0u;

    do {
        digits[n++] = (char)('0' + (value % 10u));
        value /= 10u;
    } while (value != 0u && n < sizeof(digits));

    while (n > 0u) uart_char(digits[--n]);
}

static void uart_queue_report(void)
{
    uart_text("L1=");
    uart_number(queue_len[0]);
    uart_text(", L2=");
    uart_number(queue_len[1]);
    uart_text(", L3=");
    uart_number(queue_len[2]);
    uart_text(", L4=");
    uart_number(queue_len[3]);
    uart_text("\r\n");
}

/* ===================================================================== */
/* Software I2C + PCF8574                                                */
/* ===================================================================== */
static inline void scl_hi(void)
{
    SCL_PORT->BSRR = (1u << SCL_PIN);
}

static inline void scl_lo(void)
{
    SCL_PORT->BSRR = (1u << (SCL_PIN + 16));
}

static inline void sda_hi(void)
{
    SDA_PORT->BSRR = (1u << SDA_PIN);
}

static inline void sda_lo(void)
{
    SDA_PORT->BSRR = (1u << (SDA_PIN + 16));
}

static inline uint8_t sda_rd(void)
{
    return (uint8_t)((SDA_PORT->IDR >> SDA_PIN) & 1u);
}

static void i2c_delay(void)
{
    udelay(5);
}

static void i2c_init(void)
{
    gpio_set_output_od(SCL_PORT, SCL_PIN);
    gpio_set_output_od(SDA_PORT, SDA_PIN);

    sda_hi();
    scl_hi();

    for (uint8_t i = 0; i < 9; i++) {
        scl_lo();
        i2c_delay();
        scl_hi();
        i2c_delay();
    }
}

static void i2c_start(void)
{
    sda_hi();
    scl_hi();
    i2c_delay();
    sda_lo();
    i2c_delay();
    scl_lo();
    i2c_delay();
}

static void i2c_stop(void)
{
    sda_lo();
    i2c_delay();
    scl_hi();
    i2c_delay();
    sda_hi();
    i2c_delay();
}

static uint8_t i2c_write_byte(uint8_t b)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (b & 0x80u) {
            sda_hi();
        } else {
            sda_lo();
        }

        i2c_delay();
        scl_hi();
        i2c_delay();
        scl_lo();
        b = (uint8_t)(b << 1);
    }

    sda_hi();
    i2c_delay();
    scl_hi();
    i2c_delay();

    uint8_t ack = (uint8_t)!sda_rd();

    scl_lo();
    i2c_delay();

    return ack;
}

static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t v = 0;
    sda_hi();

    for (uint8_t i = 0; i < 8; i++) {
        scl_hi();
        i2c_delay();
        v = (uint8_t)((v << 1) | sda_rd());
        scl_lo();
        i2c_delay();
    }

    if (ack) {
        sda_lo();
    } else {
        sda_hi();
    }

    i2c_delay();
    scl_hi();
    i2c_delay();
    scl_lo();
    i2c_delay();
    sda_hi();

    return v;
}

static uint8_t pcf_write(uint8_t addr, uint8_t v)
{
    i2c_start();

    uint8_t ok = i2c_write_byte((uint8_t)(addr << 1));

    if (ok) {
        ok = i2c_write_byte(v);
    }

    i2c_stop();
    return ok;
}

static uint8_t pcf_read(uint8_t addr, uint8_t *v)
{
    i2c_start();

    uint8_t ok = i2c_write_byte((uint8_t)((addr << 1) | 1u));

    if (ok) {
        *v = i2c_read_byte(0);
    }

    i2c_stop();
    return ok;
}

/* ===================================================================== */
/* 16x2 LCD through PCF8574 backpack                                     */
/* ===================================================================== */
static uint8_t lcd_ok = 0;
static char lcd_want[2][16];
static char lcd_have[2][16];

static uint8_t lcd_nibble(uint8_t nib, uint8_t rs)
{
    uint8_t b = (uint8_t)((nib << 4) | 0x08u |
                          (rs ? 0x01u : 0x00u));

    i2c_start();

    uint8_t ok = i2c_write_byte((uint8_t)(LCD_ADDR << 1));

    if (ok) {
        i2c_write_byte((uint8_t)(b | 0x04u));
        i2c_write_byte(b);
    }

    i2c_stop();
    return ok;
}

static void lcd_byte(uint8_t v, uint8_t rs)
{
    lcd_nibble((uint8_t)(v >> 4), rs);
    lcd_nibble((uint8_t)(v & 0x0Fu), rs);
}

static void lcd_init(void)
{
    delay_ms(50);

    if (!lcd_nibble(0x3, 0)) {
        lcd_ok = 0;
        return;
    }

    lcd_ok = 1;
    delay_ms(6);
    lcd_nibble(0x3, 0);
    delay_ms(2);
    lcd_nibble(0x3, 0);
    delay_ms(2);
    lcd_nibble(0x2, 0);
    delay_ms(2);

    lcd_byte(0x28, 0);
    lcd_byte(0x08, 0);
    lcd_byte(0x01, 0);
    delay_ms(3);
    lcd_byte(0x06, 0);
    lcd_byte(0x0C, 0);
}

static void put2(char *p, uint32_t v, char pad)
{
    p[0] = (v >= 10u) ?
           (char)('0' + (v / 10u) % 10u) : pad;

    p[1] = (char)('0' + v % 10u);
}

/* Copy text into an LCD row buffer (caller keeps it inside 16 columns) */
static void lcd_puts(char *dst, const char *s)
{
    while (*s) {
        *dst++ = *s++;
    }
}

static void lcd_build(void)
{
    for (uint8_t r = 0; r < 2; r++) {
        for (uint8_t c = 0; c < 16; c++) {
            lcd_want[r][c] = ' ';
        }
    }

    /* ----- Emergency mode screen ----- */
    if (em_active()) {
        char *e1 = lcd_want[0];
        char *e2 = lcd_want[1];

        lcd_puts(e1, "EMERGENCY MODE");        /* columns 0..13 */

        lcd_puts(e2, "LANE ");                 /* columns 0..4  */

        if (phase == PH_EM_GREEN) {
            /* "LANE 2 GREEN 12s" */
            e2[5] = (char)('1' + active_lane);
            lcd_puts(&e2[7], "GREEN");         /* columns 7..11 */
            put2(&e2[13], countdown_s, ' ');   /* columns 13..14 */
            e2[15] = 's';
        } else {
            /* "LANE 2  CLEARING": other lanes being cleared for lane 2 */
            uint8_t t = (em_next != NO_LANE) ? em_next : active_lane;
            e2[5] = (char)('1' + t);
            lcd_puts(&e2[8], "CLEARING");      /* columns 8..15 */
        }

        return;
    }

    /* ----- Normal screen ----- */
    char *l1 = lcd_want[0];

    l1[0] = 'L';
    l1[1] = (char)('1' + active_lane);

    const char *nm = phase_names[phase];

    for (uint8_t i = 0; i < 5; i++) {
        l1[3 + i] = nm[i];
    }

    put2(&l1[9], countdown_s, ' ');
    l1[11] = 's';

    char *l2 = lcd_want[1];
    l2[0] = 'Q';

    for (uint8_t l = 0; l < NUM_LANES; l++) {
        put2(&l2[2 + 3 * l], queue_len[l], '0');
    }

    l2[14] = ped_serving ?
             (ped_serving_elderly ? 'S' : 'P') : ' ';
}

static void lcd_step(void)
{
    static uint32_t last_build = 0;
    static uint8_t pos = 0;
    static uint8_t cur_row = 0xFF;
    static uint8_t cur_col = 0xFF;

    if (!lcd_ok) {
        return;
    }

    if ((ms_ticks - last_build) >= LCD_BUILD_MS) {
        last_build = ms_ticks;
        lcd_build();
    }

    for (uint8_t k = 0; k < 32; k++) {
        uint8_t idx = (uint8_t)((pos + k) & 31u);
        uint8_t r = (uint8_t)(idx >> 4);
        uint8_t c = (uint8_t)(idx & 15u);

        if (lcd_want[r][c] == lcd_have[r][c]) {
            continue;
        }

        if (r != cur_row || c != cur_col) {
            lcd_byte((uint8_t)(0x80u |
                     (r ? 0x40u : 0x00u) | c), 0);
        }

        lcd_byte((uint8_t)lcd_want[r][c], 1);
        lcd_have[r][c] = lcd_want[r][c];

        cur_row = r;
        cur_col = (uint8_t)(c + 1u);
        pos = (uint8_t)((idx + 1u) & 31u);

        return;
    }
}

/* ===================================================================== */
/* TM1637 countdown displays                                             */
/* ===================================================================== */
#define TM_BRIGHT   0x8A
#define SEG_E       0x79

static const uint8_t seg_digit[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,
    0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

static void tm_delay(void)
{
    udelay(5);
}

static void tm_start(uint8_t l)
{
    pin_write(&tm_dio[l], 1);
    pin_write(&tm_clk, 1);
    tm_delay();
    pin_write(&tm_dio[l], 0);
    tm_delay();
}

static void tm_stop(uint8_t l)
{
    pin_write(&tm_clk, 0);
    tm_delay();
    pin_write(&tm_dio[l], 0);
    tm_delay();
    pin_write(&tm_clk, 1);
    tm_delay();
    pin_write(&tm_dio[l], 1);
    tm_delay();
}

static void tm_write_byte(uint8_t l, uint8_t b)
{
    for (uint8_t i = 0; i < 8; i++) {
        pin_write(&tm_clk, 0);
        pin_write(&tm_dio[l], (uint8_t)(b & 1u));
        tm_delay();
        pin_write(&tm_clk, 1);
        tm_delay();
        b = (uint8_t)(b >> 1);
    }

    pin_write(&tm_clk, 0);
    gpio_set_input_pullup(tm_dio[l].port, tm_dio[l].pin);
    tm_delay();
    pin_write(&tm_clk, 1);
    tm_delay();
    pin_write(&tm_clk, 0);

    pin_write(&tm_dio[l], 0);
    gpio_set_output(tm_dio[l].port, tm_dio[l].pin);
}

static void tm_show(uint8_t l, const uint8_t seg[4])
{
    tm_start(l);
    tm_write_byte(l, 0x40);
    tm_stop(l);

    tm_start(l);
    tm_write_byte(l, 0xC0);

    for (uint8_t i = 0; i < 4; i++) {
        tm_write_byte(l, seg[i]);
    }

    tm_stop(l);

    tm_start(l);
    tm_write_byte(l, TM_BRIGHT);
    tm_stop(l);
}

/* Segment pattern for a lane, taken from disp_val[] which is refreshed
   by countdown_update() every loop */
static void tm_lane_segments(uint8_t lane, uint8_t seg[4])
{
    seg[0] = 0;
    seg[1] = 0;
    seg[2] = 0;
    seg[3] = 0;

    uint8_t v = disp_val[lane];

    if (v == DISP_BLANK) {
        return;
    }

    if (v == DISP_ZERO2) {
        seg[2] = seg_digit[0];
        seg[3] = seg_digit[0];
        return;
    }

    if (v > 99u) {
        v = 99u;
    }

    /* Emergency green is flagged with an "E" on the left */
    if (phase == PH_EM_GREEN && lane == active_lane) {
        seg[0] = SEG_E;
    }

    if (v >= 10u) {
        seg[2] = seg_digit[v / 10u];
    }

    seg[3] = seg_digit[v % 10u];
}

static void displays_update(void)
{
    static uint8_t shown[NUM_LANES][4];
    static uint8_t valid[NUM_LANES];
    static uint8_t next = 0;

    uint8_t l = next;
    next = (uint8_t)((next + 1u) % NUM_LANES);

    uint8_t seg[4];
    tm_lane_segments(l, seg);

    if (valid[l] &&
        seg[0] == shown[l][0] &&
        seg[1] == shown[l][1] &&
        seg[2] == shown[l][2] &&
        seg[3] == shown[l][3]) {
        return;
    }

    tm_show(l, seg);

    for (uint8_t i = 0; i < 4; i++) {
        shown[l][i] = seg[i];
    }

    valid[l] = 1;
}

/* ===================================================================== */
/* Buzzer                                                                */
/* ===================================================================== */
static uint8_t bz_beeps = 0;
static uint8_t bz_on = 0;
static uint32_t bz_t = 0;

static void buzzer_beeps(uint8_t n)
{
    bz_beeps = n;
    bz_on = 0;
    bz_t = ms_ticks;
}

static void buzzer_update(void)
{
    uint8_t level = 0;

    if (em_active()) {
        /* Emergency mode has priority over every other buzzer sound;
           pending pedestrian beeps are dropped */
        bz_beeps = 0;
        bz_on = 0;
        level = (ms_ticks % EM_BEEP_PERIOD_MS) < EM_BEEP_ON_MS;
    } else if (bz_beeps > 0) {
        if ((int32_t)(ms_ticks - bz_t) >= 0) {
            if (!bz_on) {
                bz_on = 1;
                bz_t = ms_ticks + BEEP_ON_MS;
            } else {
                bz_on = 0;
                bz_beeps--;
                bz_t = ms_ticks + BEEP_OFF_MS;
            }
        }

        level = bz_on;
    } else if (ped_serving) {
        uint32_t m = ms_ticks;

        if (phase == PH_GREEN_EXT) {
            level = (m % 1000u) < 80u;
        } else if (phase == PH_GREEN_CLEAR) {
            level = (m % 300u) < 100u;
        }
    }

    pin_write(&buzzer_pin, level);
}

/* ===================================================================== */
/* Pedestrian requests                                                   */
/* ===================================================================== */
static void ped_press(uint8_t lane, uint32_t now)
{
    if (ped_count[lane] == 0 ||
        (now - ped_first_ms[lane]) > PED_MULTI_WINDOW_MS) {
        ped_count[lane] = 1;
        ped_first_ms[lane] = now;
        buzzer_beeps(1);
    } else {
        ped_count[lane]++;
        buzzer_beeps(1);
    }

    if (ped_count[lane] >= PED_MULTI_COUNT) {
        ped_elderly[lane] = 1;
        ped_count[lane] = 0;
        buzzer_beeps(3);
    }

    ped_req[lane] = 1;
}

static uint32_t ped_need_ms(uint8_t lane, uint8_t clear)
{
    uint32_t need = 0;

    if (ped_elderly[lane]) {
        need = PED_ELDERLY_MIN_GREEN_MS;
    } else if (ped_req[lane]) {
        need = PED_MIN_GREEN_MS;
    }

    if (clear) {
        ped_req[lane] = 0;
        ped_elderly[lane] = 0;
    }

    return need;
}

/* Green time for a lane, from the cars in the lane (IN minus OUT):
   1 s per car, limited to EXT_MIN..EXT_MAX, plus the recovery bonus after
   an emergency, and never below the pedestrian minimum */
static uint32_t plan_green_ms(uint8_t lane)
{
    uint32_t total = (uint32_t)queue_len[lane] * GREEN_PER_CAR_MS;

    if (total < EXT_MIN_MS) {
        total = EXT_MIN_MS;
    }

    if (total > EXT_MAX_MS) {
        total = EXT_MAX_MS;
    }

    if (recovery_left > 0) {
        total += RECOVERY_BONUS_MS;

        if (total > EXT_MAX_MS) {
            total = EXT_MAX_MS;
        }
    }

    uint32_t need = ped_need_ms(lane, 0);

    if (total < need) {
        total = need;
    }

    return total;
}

/* Called while the green is running (PH_GREEN_EXT only). A pedestrian press
   made during the green extends the green to the pedestrian minimum if the
   green is shorter than that. The displays always show the real time left,
   so they follow the change. */
static void ped_update_during_ext(uint32_t elapsed)
{
    uint32_t need = ped_need_ms(active_lane, 0);

    if (need == 0 || elapsed >= need) {
        return;
    }

    ped_serving = 1;

    if (need >= PED_ELDERLY_MIN_GREEN_MS) {
        ped_serving_elderly = 1;
    }

    if ((green_end - phase_start) < need) {
        green_end = phase_start + need;
    }

    (void)ped_need_ms(active_lane, 1);
}

static uint8_t ped_led_mask(void)
{
    uint8_t mask = 0;
    uint8_t blink = (uint8_t)((ms_ticks / 250u) & 1u);

    for (uint8_t c = 0; c < NUM_LANES; c++) {
        uint8_t walk = 0;
        uint8_t dont = 1;

        if (c == active_lane && ped_serving) {
            if (phase == PH_GREEN_EXT) {
                walk = 1;
                dont = 0;
            } else if (phase == PH_GREEN_CLEAR ||
                       (phase == PH_YELLOW &&
                        em_next != NO_LANE)) {
                dont = blink;
            }
        }

        if (walk) {
            mask |= (uint8_t)(1u << (2u * c));
        }

        if (dont) {
            mask |= (uint8_t)(1u << (2u * c + 1u));
        }
    }

    return mask;
}

static void ped_leds_update(void)
{
    static uint8_t last = 0;
    static uint8_t first = 1;

    uint8_t mask = ped_led_mask();

    if (first || mask != last) {
        first = 0;
        last = mask;
        pcf_write(EXPA_ADDR, (uint8_t)~mask);
    }
}

/* ===================================================================== */
/* Inputs: IR sensors and pedestrian buttons                             */
/* ===================================================================== */
static inline uint8_t ir_raw(uint8_t s)
{
    uint8_t bit;

    if (s < 4) {
        bit = (uint8_t)((GPIOB->IDR >> (12u + s)) & 1u);
    } else {
        bit = (uint8_t)((expb_state >> s) & 1u);
    }

    return IR_ACTIVE_LOW ? (uint8_t)!bit : bit;
}

static void count_event(uint8_t sensor)
{
    for (uint8_t l = 0; l < NUM_LANES; l++) {
        if (lane_in_sensor[l] == sensor) {
            if (total_in[l] < UINT32_MAX) {
                total_in[l]++;
            }

            if (queue_len[l] < QUEUE_MAX) {
                queue_len[l]++;
            }
        }

        if (lane_out_sensor[l] == sensor) {
            if (total_out[l] < UINT32_MAX) {
                total_out[l]++;
            }

            if (queue_len[l] > 0u) {
                queue_len[l]--;
            }
        }
    }
}

static void ped_buttons_update(uint8_t raw)
{
    static uint8_t stable[NUM_LANES];
    static uint8_t agree[NUM_LANES];

    for (uint8_t l = 0; l < NUM_LANES; l++) {
        uint8_t r = (uint8_t)!((raw >> l) & 1u);

        if (r != stable[l]) {
            if (++agree[l] >= BTN_DEBOUNCE_N) {
                stable[l] = r;
                agree[l] = 0;

                if (r) {
                    ped_press(l, ms_ticks);
                }
            }
        } else {
            agree[l] = 0;
        }
    }
}

static void inputs_update(void)
{
    static uint32_t last_sample = 0;
    static uint8_t agree[NUM_SENSORS];

    if ((ms_ticks - last_sample) < IR_SAMPLE_MS) {
        return;
    }

    last_sample = ms_ticks;

    uint8_t b;
    expb_state = pcf_read(EXPB_ADDR, &b) ? b : 0xFFu;

    for (uint8_t s = 0; s < NUM_SENSORS; s++) {
        uint8_t r = ir_raw(s);

        if (r != ir_stable[s]) {
            if (++agree[s] >= IR_DEBOUNCE_N) {
                ir_stable[s] = r;
                agree[s] = 0;

                if (r) {
                    count_event(s);
                }
            }
        } else {
            agree[s] = 0;
        }
    }

    ped_buttons_update(expb_state);
}

/* ===================================================================== */
/* Emergency buttons (EXTI interrupts, active-low)                       */
/* ===================================================================== */
static void exti_service(void)
{
    uint32_t pending = EXTI->PR;

    for (uint8_t l = 0; l < NUM_LANES; l++) {
        uint32_t bit = 1u << em_pins[l].pin;

        if (!(pending & bit)) {
            continue;
        }

        EXTI->PR = bit;

        uint32_t now = ms_ticks;

        if ((now - btn_last_ms[l]) < BTN_DEBOUNCE_MS) {
            continue;
        }

        btn_last_ms[l] = now;
        em_req[l] = 1;
    }
}

void EXTI3_IRQHandler(void)
{
    exti_service();
}

void EXTI4_IRQHandler(void)
{
    exti_service();
}

void EXTI9_5_IRQHandler(void)
{
    exti_service();
}

static void emergency_init(void)
{
    for (uint8_t l = 0; l < NUM_LANES; l++) {
        uint8_t pin = em_pins[l].pin;

        gpio_set_input_pullup(em_pins[l].port, pin);

        uint8_t idx = (uint8_t)(pin >> 2);
        uint8_t sh = (uint8_t)((pin & 3u) * 4u);

        AFIO->EXTICR[idx] &= ~(0xFu << sh);
        AFIO->EXTICR[idx] |= 1u << sh;

        EXTI->FTSR |= (1u << pin);
        EXTI->PR = (1u << pin);
        EXTI->IMR |= (1u << pin);
    }

    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* ===================================================================== */
/* State machine                                                         */
/* ===================================================================== */
static void enter_phase(Phase p)
{
    phase = p;
    phase_start = ms_ticks;

    switch (p) {
    case PH_GREEN_EXT: {
        all_lanes_red();
        lane_set(active_lane, GREEN);

        /* Green time that was worked out for this lane just before its
           green (cars IN minus cars OUT) */
        uint32_t total = next_green_ms;

        if (recovery_left > 0) {
            recovery_left--;
        }

        uint32_t need = ped_need_ms(active_lane, 1);

        ped_serving = (need > 0u);
        ped_serving_elderly =
            (need >= PED_ELDERLY_MIN_GREEN_MS);

        if (total < need) {
            total = need;
        }

        green_end = phase_start + total;
        phase_len = total;
        break;
    }

    case PH_GREEN_CLEAR:
        /* same green, only the last PED_CLEAR_MS; green_end unchanged */
        phase_len = PED_CLEAR_MS;
        break;

    case PH_YELLOW:
        lane_set(active_lane, YELLOW);
        phase_len = YELLOW_MS;
        break;

    case PH_ALL_RED:
        all_lanes_red();
        ped_serving = 0;
        ped_serving_elderly = 0;
        phase_len = ALL_RED_MS;
        next_green_ms =
            plan_green_ms((uint8_t)((active_lane + 1u) % NUM_LANES));
        break;

    case PH_EM_GREEN:
        all_lanes_red();
        lane_set(active_lane, GREEN);

        ped_serving = 0;
        ped_serving_elderly = 0;

        phase_len = EMERGENCY_GREEN_MS;
        green_end = phase_start + EMERGENCY_GREEN_MS;
        recovery_left = RECOVERY_GREENS;
        break;
    }
}

static void check_emergency(void)
{
    uint8_t lane = NO_LANE;

    for (uint8_t l = 0; l < NUM_LANES; l++) {
        if (em_req[l]) {
            lane = l;
            break;
        }
    }

    if (lane == NO_LANE) {
        return;
    }

    switch (phase) {
    case PH_EM_GREEN:
        if (lane == active_lane) {
            em_req[lane] = 0;
            phase_start = ms_ticks;
            green_end = phase_start + EMERGENCY_GREEN_MS;
        }
        break;

    case PH_GREEN_EXT:
    case PH_GREEN_CLEAR:
        em_req[lane] = 0;

        if (lane == active_lane) {
            enter_phase(PH_EM_GREEN);
        } else {
            em_next = lane;
            enter_phase(PH_YELLOW);
        }
        break;

    case PH_YELLOW:
        if (em_next == NO_LANE) {
            em_next = lane;
            em_req[lane] = 0;
        }
        break;

    case PH_ALL_RED:
        em_req[lane] = 0;
        active_lane = lane;
        enter_phase(PH_EM_GREEN);
        break;
    }
}

/* Works out what every lane's display shows:
   - lane that is green: seconds of green left (the green time was fixed
     when the green started)
   - lane that is in yellow: 00
   - the lane that turns green next: 9..1 during the last 9 seconds
     before it turns green (counted through the rest of the current
     green, the yellow and the all-red)
   - during the all-red, if the next lane gets only the minimum green,
     the lane after it already shows its 9..                              */
static void countdown_update(void)
{
    uint32_t elapsed = ms_ticks - phase_start;
    uint32_t left = 0;
    uint32_t eta = 0;      /* ms until the next lane turns green */
    uint8_t  next = (uint8_t)((active_lane + 1u) % NUM_LANES);

    for (uint8_t l = 0; l < NUM_LANES; l++) {
        disp_val[l] = DISP_BLANK;
    }

    switch (phase) {
    case PH_GREEN_EXT:
    case PH_GREEN_CLEAR:
    case PH_EM_GREEN:
        left = green_left_ms();
        disp_val[active_lane] = (uint8_t)((left + 999u) / 1000u);
        countdown_s = (left + 999u) / 1000u;
        eta = left + YELLOW_MS + ALL_RED_MS;
        break;

    case PH_YELLOW:
        left = (elapsed < phase_len) ? (phase_len - elapsed) : 0u;
        disp_val[active_lane] = DISP_ZERO2;
        countdown_s = (left + 999u) / 1000u;

        if (em_next != NO_LANE) {
            /* yellow leads straight into the emergency green */
            next = em_next;
            eta = left;
        } else {
            eta = left + ALL_RED_MS;
        }
        break;

    default:    /* PH_ALL_RED */
        left = (elapsed < phase_len) ? (phase_len - elapsed) : 0u;
        countdown_s = (left + 999u) / 1000u;
        eta = left;

        {
            /* With a short green for the next lane, the lane after it is
               already inside its 9-second window during this all-red */
            uint32_t eta2 = left + next_green_ms + YELLOW_MS + ALL_RED_MS;

            if (eta2 <= COUNTDOWN_MS) {
                disp_val[(next + 1u) % NUM_LANES] =
                    (uint8_t)((eta2 + 999u) / 1000u);
            }
        }
        break;
    }

    if (eta <= COUNTDOWN_MS) {
        disp_val[next] = (uint8_t)((eta + 999u) / 1000u);
    }
}

static void signal_update(void)
{
    uint32_t elapsed = ms_ticks - phase_start;

    switch (phase) {
    case PH_GREEN_EXT:
        ped_update_during_ext(elapsed);

        if (green_left_ms() <= PED_CLEAR_MS) {
            enter_phase(PH_GREEN_CLEAR);
        }
        break;

    case PH_GREEN_CLEAR:
    case PH_EM_GREEN:
        if (green_left_ms() == 0u) {
            enter_phase(PH_YELLOW);
        }
        break;

    case PH_YELLOW:
        if (elapsed >= phase_len) {
            if (em_next != NO_LANE) {
                active_lane = em_next;
                em_next = NO_LANE;
                enter_phase(PH_EM_GREEN);
            } else {
                enter_phase(PH_ALL_RED);
            }
        }
        break;

    case PH_ALL_RED:
        if (elapsed >= phase_len) {
            active_lane =
                (uint8_t)((active_lane + 1u) % NUM_LANES);
            enter_phase(PH_GREEN_EXT);
        }
        break;
    }

    countdown_update();
}

/* ===================================================================== */
/* main                                                                  */
/* ===================================================================== */
int main(void)
{
    /* Enable GPIOA, GPIOB, GPIOC and AFIO clocks */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPCEN |
                    RCC_APB2ENR_AFIOEN;

    /* Disable JTAG but retain SWD */
    AFIO->MAPR =
        (AFIO->MAPR & ~(0x7u << 24)) | (0x2u << 24);

    /* Configure traffic LEDs */
    for (uint8_t l = 0; l < NUM_LANES; l++) {
        for (uint8_t c = 0; c < 3; c++) {
            gpio_set_output(lane_pins[l][c].port,
                            lane_pins[l][c].pin);
        }
    }

    /* Direct IR inputs PB12..PB15 */
    for (uint8_t s = 0; s < 4; s++) {
        gpio_set_input_pullup(GPIOB, (uint8_t)(12u + s));
    }

    /* Buzzer */
    gpio_set_output(buzzer_pin.port, buzzer_pin.pin);
    pin_write(&buzzer_pin, 0);

    /* TM1637 display pins */
    gpio_set_output(tm_clk.port, tm_clk.pin);
    pin_write(&tm_clk, 1);

    for (uint8_t l = 0; l < NUM_LANES; l++) {
        gpio_set_output(tm_dio[l].port, tm_dio[l].pin);
        pin_write(&tm_dio[l], 1);
    }

    /* Heartbeat LED PC13 (active-low on Blue Pill) */
    gpio_set_output(GPIOC, 13);

    /* USART1 TX on PA9 */
    uart_init();

    /* 1 ms system tick */
    SysTick_Config(SystemCoreClock / 1000u);

    /* Initialize software I2C and expanders */
    i2c_init();
    pcf_write(EXPA_ADDR, 0xFF);
    pcf_write(EXPB_ADDR, 0xFF);

    /* Initialize LCD */
    lcd_init();

    /* Initialize IR sensor states */
    {
        uint8_t b;

        expb_state = pcf_read(EXPB_ADDR, &b) ? b : 0xFFu;

        for (uint8_t s = 0; s < NUM_SENSORS; s++) {
            ir_stable[s] = ir_raw(s);
        }
    }

    /* Initialize emergency button interrupts */
    emergency_init();

    /* Start with all-red */
    all_lanes_red();
    enter_phase(PH_ALL_RED);

    uint32_t last_beat = 0;
    uint32_t last_uart = 0;

    while (1) {
        inputs_update();
        check_emergency();
        signal_update();
        buzzer_update();
        ped_leds_update();
        displays_update();
        lcd_step();

        /* Send queue estimates every 500 ms */
        if ((ms_ticks - last_uart) >= 500u) {
            last_uart = ms_ticks;
            uart_queue_report();
        }

        /* Heartbeat toggles every 250 ms */
        if ((ms_ticks - last_beat) >= 250u) {
            last_beat = ms_ticks;
            GPIOC->ODR ^= (1u << 13);
        }
    }
}
