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

static const PinDef em_pins[NUM_LANES] = {      /* emergency buttons */
    {GPIOB, 3}, {GPIOB, 4}, {GPIOB, 5}, {GPIOB, 6}
};

static const PinDef buzzer_pin = {GPIOA, 11};

static const PinDef tm_clk = {GPIOB, 8};        /* shared TM1637 clock */
static const PinDef tm_dio[NUM_LANES] = {       /* one data pin per lane */
    {GPIOB, 9}, {GPIOA, 8}, {GPIOA, 9}, {GPIOA, 10}
};

/* Software I2C pins */
#define SCL_PORT GPIOB
#define SCL_PIN  7
#define SDA_PORT GPIOA
#define SDA_PIN  15

/* I2C device addresses (7-bit) */
#define LCD_ADDR   0x27   /* try 0x3F if the LCD stays blank */
#define EXPA_ADDR  0x20   /* pedestrian LEDs   (PCF8574, A0..A2 = GND)  */
#define EXPB_ADDR  0x21   /* ped buttons + IR  (PCF8574, A0 = 3.3 V)    */

/* ---------- Timing (ms) - tune these for your demo ---------- */
#define YELLOW_MS                 2000u
#define ALL_RED_MS                1000u   /* safety gap between lanes      */

#define EXT_MIN_MS                5000u   /* minimum green (extension phase) */
#define EXT_MAX_MS                15000u  /* extension never longer          */
#define COUNTDOWN_S               9u      /* countdown after extension ends  */
#define COUNTDOWN_MS              (COUNTDOWN_S * 1000u)

#define PED_MIN_GREEN_MS          7000u   /* pedestrian pushed: min green  */
#define PED_ELDERLY_MIN_GREEN_MS  12000u  /* pushed three times together   */
#define PED_MULTI_COUNT           3u
#define PED_MULTI_WINDOW_MS       3000u

#define EMERGENCY_GREEN_MS        12000u

#define RECOVERY_BONUS_MS         3000u   /* extra minimum green afterwards */
#define RECOVERY_GREENS           8u      /* 2 full cycles; 0 = disable     */

#define BTN_DEBOUNCE_MS           50u     /* emergency buttons (interrupt)  */
#define BTN_DEBOUNCE_N            2u      /* pedestrian buttons (2x20 ms)   */

#define BEEP_ON_MS                80u
#define BEEP_OFF_MS               120u
#define LCD_BUILD_MS              250u

/* ---------- IR sensor settings ---------- */
#define IR_ACTIVE_LOW    1     /* LM393 modules: OUT is LOW when blocked */
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

static void delay_ms(uint32_t ms)       /* only used during start-up */
{
    uint32_t t = ms_ticks;
    while ((ms_ticks - t) < ms) { }
}

/* Rough microsecond delay (at least about 'us'; clock speed independent) */
static void udelay(uint32_t us)
{
    volatile uint32_t n = us * (SystemCoreClock / 3000000u) + 1u;
    while (n--) { __NOP(); }
}

/* ---------- GPIO helpers (STM32F1: CRL/CRH, not MODER) ---------- */
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
{   /* MODE=10 (2 MHz), CNF=00 push-pull */
    gpio_cfg(port, pin, 0x2u);
}

static void gpio_set_output_od(GPIO_TypeDef *port, uint8_t pin)
{   /* MODE=10 (2 MHz), CNF=01 open-drain (for I2C) */
    gpio_cfg(port, pin, 0x6u);
}

static void gpio_set_input_pullup(GPIO_TypeDef *port, uint8_t pin)
{   /* MODE=00, CNF=10, ODR=1 selects pull-UP */
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
    for (uint8_t l = 0; l < NUM_LANES; l++) lane_set(l, RED);
}

/* ---------- State machine variables ---------- */
typedef enum {
    PH_GREEN_EXT,     /* green, extending while cars remain */
    PH_GREEN_CLEAR,   /* green, 9..0 countdown               */
    PH_YELLOW,
    PH_ALL_RED,
    PH_EM_GREEN       /* emergency green                     */
} Phase;

static const char *const phase_names[5] = {
    "GREEN", "CLEAR", "YELLW", "ALLRD", "EMERG"
};

static Phase    phase       = PH_ALL_RED;
static uint8_t  active_lane = NUM_LANES - 1;   /* first advance -> lane 0 */
static uint32_t phase_start = 0;
static uint32_t phase_len   = ALL_RED_MS;
static uint32_t ext_floor_ms = EXT_MIN_MS;     /* earliest end of extension */

static uint8_t  em_next       = NO_LANE;   /* emergency lane after the yellow */
static uint8_t  recovery_left = 0;

static volatile uint32_t countdown_s = 0;  /* seconds left (0 during GREEN) */

/* Pedestrian state */
static uint8_t  ped_req[NUM_LANES];        /* button pushed, waiting        */
static uint8_t  ped_elderly[NUM_LANES];    /* pushed 3x together            */
static uint8_t  ped_count[NUM_LANES];
static uint32_t ped_first_ms[NUM_LANES];
static uint8_t  ped_serving = 0;           /* open lane is serving a request */
static uint8_t  ped_serving_elderly = 0;

/* Emergency flags (set in interrupts) */
static volatile uint8_t em_req[NUM_LANES];
static uint32_t btn_last_ms[NUM_LANES];

/* IR / queue */
static uint8_t ir_stable[NUM_SENSORS];
static uint8_t queue_len[NUM_LANES];
static uint8_t expb_state = 0xFF;          /* last byte read from PCF8574 B */

/* ===================================================================== */
/*  Software I2C (open-drain, ~50 kHz) + PCF8574                          */
/* ===================================================================== */
static inline void scl_hi(void) { SCL_PORT->BSRR = (1u << SCL_PIN); }
static inline void scl_lo(void) { SCL_PORT->BSRR = (1u << (SCL_PIN + 16)); }
static inline void sda_hi(void) { SDA_PORT->BSRR = (1u << SDA_PIN); }
static inline void sda_lo(void) { SDA_PORT->BSRR = (1u << (SDA_PIN + 16)); }
static inline uint8_t sda_rd(void) { return (uint8_t)((SDA_PORT->IDR >> SDA_PIN) & 1u); }

static void i2c_delay(void) { udelay(5); }

static void i2c_init(void)
{
    gpio_set_output_od(SCL_PORT, SCL_PIN);
    gpio_set_output_od(SDA_PORT, SDA_PIN);
    sda_hi();
    scl_hi();
    /* free a slave that may be holding SDA low after a reset */
    for (uint8_t i = 0; i < 9; i++) {
        scl_lo(); i2c_delay();
        scl_hi(); i2c_delay();
    }
}

static void i2c_start(void)
{
    sda_hi(); scl_hi(); i2c_delay();
    sda_lo(); i2c_delay();
    scl_lo(); i2c_delay();
}

static void i2c_stop(void)
{
    sda_lo(); i2c_delay();
    scl_hi(); i2c_delay();
    sda_hi(); i2c_delay();
}

/* Returns 1 if the device acknowledged */
static uint8_t i2c_write_byte(uint8_t b)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (b & 0x80u) sda_hi(); else sda_lo();
        i2c_delay();
        scl_hi(); i2c_delay();
        scl_lo();
        b = (uint8_t)(b << 1);
    }
    sda_hi();                    /* release SDA for the ACK bit */
    i2c_delay();
    scl_hi(); i2c_delay();
    uint8_t ack = (uint8_t)!sda_rd();
    scl_lo(); i2c_delay();
    return ack;
}

static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t v = 0;
    sda_hi();
    for (uint8_t i = 0; i < 8; i++) {
        scl_hi(); i2c_delay();
        v = (uint8_t)((v << 1) | sda_rd());
        scl_lo(); i2c_delay();
    }
    if (ack) sda_lo(); else sda_hi();
    i2c_delay();
    scl_hi(); i2c_delay();
    scl_lo(); i2c_delay();
    sda_hi();
    return v;
}

static uint8_t pcf_write(uint8_t addr, uint8_t v)
{
    i2c_start();
    uint8_t ok = i2c_write_byte((uint8_t)(addr << 1));
    if (ok) ok = i2c_write_byte(v);
    i2c_stop();
    return ok;
}

static uint8_t pcf_read(uint8_t addr, uint8_t *v)
{
    i2c_start();
    uint8_t ok = i2c_write_byte((uint8_t)((addr << 1) | 1u));
    if (ok) *v = i2c_read_byte(0);
    i2c_stop();
    return ok;
}

/* ===================================================================== */
/*  16x2 LCD through a PCF8574 backpack                                   */
/*  backpack bits: P0=RS P1=RW P2=EN P3=backlight P4..P7=D4..D7           */
/* ===================================================================== */
static uint8_t lcd_ok = 0;
static char    lcd_want[2][16];
static char    lcd_have[2][16];

static uint8_t lcd_nibble(uint8_t nib, uint8_t rs)
{
    uint8_t b = (uint8_t)((nib << 4) | 0x08u | (rs ? 0x01u : 0x00u));
    i2c_start();
    uint8_t ok = i2c_write_byte((uint8_t)(LCD_ADDR << 1));
    if (ok) {
        i2c_write_byte((uint8_t)(b | 0x04u));   /* EN high */
        i2c_write_byte(b);                      /* EN low  */
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
    if (!lcd_nibble(0x3, 0)) { lcd_ok = 0; return; }   /* no LCD found */
    lcd_ok = 1;
    delay_ms(6);
    lcd_nibble(0x3, 0); delay_ms(2);
    lcd_nibble(0x3, 0); delay_ms(2);
    lcd_nibble(0x2, 0); delay_ms(2);     /* switch to 4-bit mode      */
    lcd_byte(0x28, 0);                   /* 4-bit, 2 lines, 5x8 font  */
    lcd_byte(0x08, 0);                   /* display off               */
    lcd_byte(0x01, 0); delay_ms(3);      /* clear                     */
    lcd_byte(0x06, 0);                   /* cursor moves right        */
    lcd_byte(0x0C, 0);                   /* display on, no cursor     */
}

/* two digits; 'pad' replaces a leading zero */
static void put2(char *p, uint32_t v, char pad)
{
    p[0] = (v >= 10u) ? (char)('0' + (v / 10u) % 10u) : pad;
    p[1] = (char)('0' + v % 10u);
}

static void lcd_build(void)
{
    for (uint8_t r = 0; r < 2; r++)
        for (uint8_t c = 0; c < 16; c++) lcd_want[r][c] = ' ';

    char *l1 = lcd_want[0];                     /* "L1 GREEN  --s"  */
    l1[0] = 'L';
    l1[1] = (char)('1' + active_lane);
    const char *nm = phase_names[phase];
    for (uint8_t i = 0; i < 5; i++) l1[3 + i] = nm[i];
    if (phase == PH_GREEN_EXT) { l1[9] = '-'; l1[10] = '-'; }
    else                       { put2(&l1[9], countdown_s, ' '); }
    l1[11] = 's';

    char *l2 = lcd_want[1];                     /* "Q 03 01 00 00 P" */
    l2[0] = 'Q';
    for (uint8_t l = 0; l < NUM_LANES; l++)
        put2(&l2[2 + 3 * l], queue_len[l], '0');
    l2[14] = ped_serving ? (ped_serving_elderly ? 'S' : 'P') : ' ';
}

/* Writes at most ONE changed character per call (keeps the loop fast) */
static void lcd_step(void)
{
    static uint32_t last_build = 0;
    static uint8_t  pos = 0;
    static uint8_t  cur_row = 0xFF, cur_col = 0xFF;

    if (!lcd_ok) return;
    if ((ms_ticks - last_build) >= LCD_BUILD_MS) {
        last_build = ms_ticks;
        lcd_build();
    }
    for (uint8_t k = 0; k < 32; k++) {
        uint8_t idx = (uint8_t)((pos + k) & 31u);
        uint8_t r = (uint8_t)(idx >> 4);
        uint8_t c = (uint8_t)(idx & 15u);
        if (lcd_want[r][c] == lcd_have[r][c]) continue;

        if (r != cur_row || c != cur_col)       /* set cursor if needed */
            lcd_byte((uint8_t)(0x80u | (r ? 0x40u : 0x00u) | c), 0);
        lcd_byte((uint8_t)lcd_want[r][c], 1);
        lcd_have[r][c] = lcd_want[r][c];
        cur_row = r;
        cur_col = (uint8_t)(c + 1u);
        pos = (uint8_t)((idx + 1u) & 31u);
        return;
    }
}

/* ===================================================================== */
/*  TM1637 countdown displays (shared CLK, one DIO per lane)              */
/* ===================================================================== */
#define TM_BRIGHT   0x8A      /* display on, brightness 2 (0x88..0x8F) */
#define SEG_DASH    0x40
#define SEG_E       0x79

static const uint8_t seg_digit[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

static void tm_delay(void) { udelay(5); }

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
    for (uint8_t i = 0; i < 8; i++) {           /* LSB first */
        pin_write(&tm_clk, 0);
        pin_write(&tm_dio[l], (uint8_t)(b & 1u));
        tm_delay();
        pin_write(&tm_clk, 1);
        tm_delay();
        b = (uint8_t)(b >> 1);
    }
    /* ACK clock: let the module pull DIO low (we do not check it) */
    pin_write(&tm_clk, 0);
    gpio_set_input_pullup(tm_dio[l].port, tm_dio[l].pin);
    tm_delay();
    pin_write(&tm_clk, 1);
    tm_delay();
    pin_write(&tm_clk, 0);
    pin_write(&tm_dio[l], 0);                   /* ODR=0 before output mode */
    gpio_set_output(tm_dio[l].port, tm_dio[l].pin);
}

static void tm_show(uint8_t l, const uint8_t seg[4])
{
    tm_start(l); tm_write_byte(l, 0x40); tm_stop(l);        /* auto-increment */
    tm_start(l);
    tm_write_byte(l, 0xC0);                                  /* first digit    */
    for (uint8_t i = 0; i < 4; i++) tm_write_byte(l, seg[i]);
    tm_stop(l);
    tm_start(l); tm_write_byte(l, TM_BRIGHT); tm_stop(l);   /* display on     */
}

/* What this lane's display should show right now */
static void tm_lane_segments(uint8_t lane, uint8_t seg[4])
{
    seg[0] = 0; seg[1] = 0; seg[2] = 0; seg[3] = 0;
    if (lane != active_lane) return;                 /* waiting lanes: blank */

    uint32_t s = countdown_s;
    if (s > 99u) s = 99u;

    switch (phase) {
    case PH_GREEN_EXT:                               /* "--"                 */
        seg[2] = SEG_DASH; seg[3] = SEG_DASH;
        break;
    case PH_GREEN_CLEAR:                             /* " 9" ... " 1"        */
    case PH_EM_GREEN:                                /* "E 12" ... "E  1"    */
        if (phase == PH_EM_GREEN) seg[0] = SEG_E;
        if (s >= 10u) seg[2] = seg_digit[s / 10u];
        seg[3] = seg_digit[s % 10u];
        break;
    case PH_YELLOW:                                  /* countdown hit "00"   */
        seg[2] = seg_digit[0]; seg[3] = seg_digit[0];
        break;
    default:
        break;
    }
}

/* Refreshes ONE lane's module per call, and only when its content changed */
static void displays_update(void)
{
    static uint8_t shown[NUM_LANES][4];
    static uint8_t valid[NUM_LANES];
    static uint8_t next = 0;

    uint8_t l = next;
    next = (uint8_t)((next + 1u) % NUM_LANES);

    uint8_t seg[4];
    tm_lane_segments(l, seg);

    if (valid[l] && seg[0] == shown[l][0] && seg[1] == shown[l][1] &&
        seg[2] == shown[l][2] && seg[3] == shown[l][3]) return;

    tm_show(l, seg);
    for (uint8_t i = 0; i < 4; i++) shown[l][i] = seg[i];
    valid[l] = 1;
}

/* ===================================================================== */
/*  Buzzer                                                                */
/* ===================================================================== */
static uint8_t  bz_beeps = 0;     /* acknowledgement beeps still to play */
static uint8_t  bz_on    = 0;
static uint32_t bz_t     = 0;

static void buzzer_beeps(uint8_t n)
{
    bz_beeps = n;
    bz_on    = 0;
    bz_t     = ms_ticks;
}

static void buzzer_update(void)
{
    uint8_t level = 0;

    if (bz_beeps > 0) {                                  /* ack beeps first */
        if ((int32_t)(ms_ticks - bz_t) >= 0) {
            if (!bz_on) { bz_on = 1; bz_t = ms_ticks + BEEP_ON_MS; }
            else        { bz_on = 0; bz_beeps--; bz_t = ms_ticks + BEEP_OFF_MS; }
        }
        level = bz_on;
    } else if (ped_serving) {                            /* walk / clearance */
        uint32_t m = ms_ticks;
        if (phase == PH_GREEN_EXT)        level = (m % 1000u) < 80u;  /* slow tick */
        else if (phase == PH_GREEN_CLEAR) level = (m % 300u) < 100u;  /* fast tick */
    }
    pin_write(&buzzer_pin, level);
}

/* ===================================================================== */
/*  Pedestrian requests                                                   */
/* ===================================================================== */

/* Called when a pedestrian button press is confirmed */
static void ped_press(uint8_t lane, uint32_t now)
{
    if (ped_count[lane] == 0 || (now - ped_first_ms[lane]) > PED_MULTI_WINDOW_MS) {
        ped_count[lane]    = 1;
        ped_first_ms[lane] = now;
        buzzer_beeps(1);
    } else {
        ped_count[lane]++;
        buzzer_beeps(1);
    }
    if (ped_count[lane] >= PED_MULTI_COUNT) {
        ped_elderly[lane] = 1;           /* third press: elderly mode */
        ped_count[lane]   = 0;
        buzzer_beeps(3);
    }
    ped_req[lane] = 1;
}

/* Minimum green this lane's pedestrian needs (0 = none); optionally clears */
static uint32_t ped_need_ms(uint8_t lane, uint8_t clear)
{
    uint32_t need = 0;
    if (ped_elderly[lane])   need = PED_ELDERLY_MIN_GREEN_MS;
    else if (ped_req[lane])  need = PED_MIN_GREEN_MS;
    if (clear) { ped_req[lane] = 0; ped_elderly[lane] = 0; }
    return need;
}

/* Pedestrian button pressed while the lane is already in its extension
 * phase: accepted only if the minimum time has not yet been reached,
 * otherwise it waits for the next green of this lane. */
static void ped_update_during_ext(uint32_t elapsed)
{
    uint32_t need = ped_need_ms(active_lane, 0);
    if (need == 0 || elapsed >= need) return;
    ped_serving = 1;
    if (need >= PED_ELDERLY_MIN_GREEN_MS) ped_serving_elderly = 1;
    if (ext_floor_ms < need) ext_floor_ms = need;
    (void)ped_need_ms(active_lane, 1);
}

/* 8-bit mask of pedestrian LEDs that should be ON (bit 2c = walk,
 * bit 2c+1 = don't walk, for crossing c) */
static uint8_t ped_led_mask(void)
{
    uint8_t mask  = 0;
    uint8_t blink = (uint8_t)((ms_ticks / 250u) & 1u);

    for (uint8_t c = 0; c < NUM_LANES; c++) {
        uint8_t walk = 0, dont = 1;                  /* default: don't walk */
        if (c == active_lane && ped_serving) {
            if (phase == PH_GREEN_EXT) {
                walk = 1; dont = 0;
            } else if (phase == PH_GREEN_CLEAR ||
                       (phase == PH_YELLOW && em_next != NO_LANE)) {
                dont = blink;                        /* flashing clearance   */
            }
        }
        if (walk) mask |= (uint8_t)(1u << (2u * c));
        if (dont) mask |= (uint8_t)(1u << (2u * c + 1u));
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
        last  = mask;
        pcf_write(EXPA_ADDR, (uint8_t)~mask);        /* LEDs are active-low */
    }
}

/* ===================================================================== */
/*  Inputs: IR sensors (direct + via PCF8574 B) and pedestrian buttons    */
/* ===================================================================== */
static inline uint8_t ir_raw(uint8_t s)              /* 1 = vehicle present */
{
    uint8_t bit;
    if (s < 4) bit = (uint8_t)((GPIOB->IDR >> (12u + s)) & 1u);  /* PB12..15 */
    else       bit = (uint8_t)((expb_state >> s) & 1u);          /* P4..P7   */
    return IR_ACTIVE_LOW ? (uint8_t)!bit : bit;
}

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

static void ped_buttons_update(uint8_t raw)          /* raw bits 0..3 */
{
    static uint8_t stable[NUM_LANES];                /* 1 = pressed */
    static uint8_t agree[NUM_LANES];

    for (uint8_t l = 0; l < NUM_LANES; l++) {
        uint8_t r = (uint8_t)!((raw >> l) & 1u);
        if (r != stable[l]) {
            if (++agree[l] >= BTN_DEBOUNCE_N) {
                stable[l] = r;
                agree[l]  = 0;
                if (r) ped_press(l, ms_ticks);
            }
        } else {
            agree[l] = 0;
        }
    }
}

static void inputs_update(void)
{
    static uint32_t last_sample = 0;
    static uint8_t  agree[NUM_SENSORS];

    if ((ms_ticks - last_sample) < IR_SAMPLE_MS) return;
    last_sample = ms_ticks;

    uint8_t b;
    expb_state = pcf_read(EXPB_ADDR, &b) ? b : 0xFFu;   /* missing = idle */

    for (uint8_t s = 0; s < NUM_SENSORS; s++) {
        uint8_t r = ir_raw(s);
        if (r != ir_stable[s]) {
            if (++agree[s] >= IR_DEBOUNCE_N) {
                ir_stable[s] = r;
                agree[s]     = 0;
                if (r) count_event(s);       /* clear -> blocked = 1 vehicle */
            }
        } else {
            agree[s] = 0;
        }
    }

    ped_buttons_update(expb_state);
}

/* ===================================================================== */
/*  Emergency buttons (EXTI interrupts, active-low)                       */
/* ===================================================================== */
static void exti_service(void)
{
    uint32_t pending = EXTI->PR;
    for (uint8_t l = 0; l < NUM_LANES; l++) {
        uint32_t bit = 1u << em_pins[l].pin;
        if (!(pending & bit)) continue;

        EXTI->PR = bit;                                   /* clear (write 1) */
        uint32_t now = ms_ticks;
        if ((now - btn_last_ms[l]) < BTN_DEBOUNCE_MS) continue;   /* bounce */
        btn_last_ms[l] = now;
        em_req[l] = 1;
    }
}

void EXTI3_IRQHandler(void)     { exti_service(); }   /* PB3       */
void EXTI4_IRQHandler(void)     { exti_service(); }   /* PB4       */
void EXTI9_5_IRQHandler(void)   { exti_service(); }   /* PB5, PB6  */

static void emergency_init(void)
{
    for (uint8_t l = 0; l < NUM_LANES; l++) {
        uint8_t pin = em_pins[l].pin;
        gpio_set_input_pullup(em_pins[l].port, pin);

        uint8_t idx = (uint8_t)(pin >> 2);
        uint8_t sh  = (uint8_t)((pin & 3u) * 4u);
        AFIO->EXTICR[idx] &= ~(0xFu << sh);
        AFIO->EXTICR[idx] |= 1u << sh;               /* port B = 1 */

        EXTI->FTSR |= (1u << pin);                   /* falling edge */
        EXTI->PR    = (1u << pin);
        EXTI->IMR  |= (1u << pin);
    }
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* ===================================================================== */
/*  State machine                                                         */
/* ===================================================================== */
static void enter_phase(Phase p)
{
    phase       = p;
    phase_start = ms_ticks;

    switch (p) {
    case PH_GREEN_EXT: {
        all_lanes_red();
        lane_set(active_lane, GREEN);

        ext_floor_ms = EXT_MIN_MS;
        if (recovery_left > 0) {                 /* after an emergency */
            ext_floor_ms += RECOVERY_BONUS_MS;
            recovery_left--;
        }
        if (ext_floor_ms > EXT_MAX_MS) ext_floor_ms = EXT_MAX_MS;

        uint32_t need = ped_need_ms(active_lane, 1);  /* pedestrian waiting */
        ped_serving         = (need > 0u);
        ped_serving_elderly = (need >= PED_ELDERLY_MIN_GREEN_MS);
        if (ext_floor_ms < need) ext_floor_ms = need;

        phase_len = EXT_MAX_MS;                  /* upper limit only */
        break;
    }

    case PH_GREEN_CLEAR:                         /* lights stay green */
        phase_len = COUNTDOWN_MS;
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
        break;

    case PH_EM_GREEN:
        all_lanes_red();
        lane_set(active_lane, GREEN);
        ped_serving = 0;
        ped_serving_elderly = 0;
        phase_len     = EMERGENCY_GREEN_MS;
        recovery_left = RECOVERY_GREENS;
        break;
    }
}

static void check_emergency(void)
{
    uint8_t lane = NO_LANE;
    for (uint8_t l = 0; l < NUM_LANES; l++) {
        if (em_req[l]) { lane = l; break; }           /* lowest lane first */
    }
    if (lane == NO_LANE) return;

    switch (phase) {
    case PH_EM_GREEN:
        if (lane == active_lane) {                    /* vehicle still coming */
            em_req[lane] = 0;
            phase_start  = ms_ticks;                  /* restart the 12 s     */
        }
        break;                                        /* other lanes wait     */

    case PH_GREEN_EXT:
    case PH_GREEN_CLEAR:
        em_req[lane] = 0;
        if (lane == active_lane) {
            enter_phase(PH_EM_GREEN);                 /* already green: hold  */
        } else {
            em_next = lane;                           /* yellow, then lane E  */
            enter_phase(PH_YELLOW);
        }
        break;

    case PH_YELLOW:                                   /* yellow already running */
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
    uint32_t elapsed = ms_ticks - phase_start;       /* wrap-safe (unsigned) */

    if (phase == PH_GREEN_EXT) {
        ped_update_during_ext(elapsed);
        /* Decision point: minimum time done AND (queue empty OR cap hit) */
        if (elapsed >= ext_floor_ms &&
            (queue_len[active_lane] == 0 || elapsed >= EXT_MAX_MS)) {
            enter_phase(PH_GREEN_CLEAR);
            elapsed = 0;
        }
    } else if (elapsed >= phase_len) {
        switch (phase) {
        case PH_GREEN_CLEAR:
        case PH_EM_GREEN:
            enter_phase(PH_YELLOW);
            break;
        case PH_YELLOW:
            if (em_next != NO_LANE) {                /* straight to emergency */
                active_lane = em_next;
                em_next     = NO_LANE;
                enter_phase(PH_EM_GREEN);
            } else {
                enter_phase(PH_ALL_RED);
            }
            break;
        case PH_ALL_RED:
            active_lane = (uint8_t)((active_lane + 1u) % NUM_LANES);
            enter_phase(PH_GREEN_EXT);
            break;
        default:
            break;
        }
        elapsed = 0;
    }

    if (phase == PH_GREEN_EXT || elapsed >= phase_len) countdown_s = 0;
    else countdown_s = (phase_len - elapsed + 999u) / 1000u;   /* round up */
}

/* ===================================================================== */
/*  main                                                                  */
/* ===================================================================== */
int main(void)
{
    /* Clocks: GPIOA, GPIOB, GPIOC and AFIO (EXTI + JTAG setting) */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

    /* Free PB3, PB4, PA15 from JTAG but KEEP SWD (SWJ_CFG = 0b010) so the
     * ST-Link still works. Never write 0b100: it disables SWD too. */
    AFIO->MAPR = (AFIO->MAPR & ~(0x7u << 24)) | (0x2u << 24);

    /* 12 traffic LEDs */
    for (uint8_t l = 0; l < NUM_LANES; l++)
        for (uint8_t c = 0; c < 3; c++)
            gpio_set_output(lane_pins[l][c].port, lane_pins[l][c].pin);

    /* Direct IR inputs s0..s3 on PB12..PB15 (pull-up) */
    for (uint8_t s = 0; s < 4; s++)
        gpio_set_input_pullup(GPIOB, (uint8_t)(12u + s));

    /* Buzzer (off) */
    gpio_set_output(buzzer_pin.port, buzzer_pin.pin);
    pin_write(&buzzer_pin, 0);

    /* TM1637 pins: clock and the 4 data lines idle high */
    gpio_set_output(tm_clk.port, tm_clk.pin);
    pin_write(&tm_clk, 1);
    for (uint8_t l = 0; l < NUM_LANES; l++) {
        gpio_set_output(tm_dio[l].port, tm_dio[l].pin);
        pin_write(&tm_dio[l], 1);
    }

    /* Heartbeat LED PC13 (active-low) */
    gpio_set_output(GPIOC, 13);

    /* 1 ms tick (needed before the LCD start-up delays) */
    SysTick_Config(SystemCoreClock / 1000u);

    /* I2C devices: expanders idle (LEDs off, inputs released), then LCD */
    i2c_init();
    pcf_write(EXPA_ADDR, 0xFF);
    pcf_write(EXPB_ADDR, 0xFF);
    lcd_init();

    /* Start the IR debounce from the CURRENT state, so a sensor already
     * blocked at power-up is not counted as a vehicle */
    {
        uint8_t b;
        expb_state = pcf_read(EXPB_ADDR, &b) ? b : 0xFFu;
        for (uint8_t s = 0; s < NUM_SENSORS; s++) ir_stable[s] = ir_raw(s);
    }

    emergency_init();

    all_lanes_red();
    enter_phase(PH_ALL_RED);

    uint32_t last_beat = 0;

    while (1) {
        inputs_update();        /* IR counting + pedestrian buttons   */
        check_emergency();      /* flags set by the EXTI interrupts   */
        signal_update();        /* light sequence and countdown       */
        buzzer_update();
        ped_leds_update();
        displays_update();      /* one TM1637 module per pass         */
        lcd_step();             /* one LCD character per pass         */

        /* Heartbeat: toggles every 250 ms (proves loop is running) */
        if ((ms_ticks - last_beat) >= 250u) {
            last_beat = ms_ticks;
            GPIOC->ODR ^= (1u << 13);
        }
    }
}
