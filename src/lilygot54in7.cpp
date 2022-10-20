#include <stdbool.h>
#include <stdint.h>
#include <driver/gpio.h>
#include <esp_attr.h>
#include <driver/periph_ctrl.h>
#include <esp_heap_caps.h>
#include <esp_adc_cal.h>
// #include <esp_idf_version.h>
// #if ESP_IDF_VERSION_MAJOR >= 4
// #include <esp32/rom/lldesc.h>
// #else

#include <rom/lldesc.h>

// #endif
#include <soc/i2s_reg.h>
#include <soc/i2s_struct.h>
#include <soc/rtc.h>
#include <lilygot54in7.hpp>

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <esp_assert.h>
#include <esp_log.h>
#include <esp_types.h>
#include <esp_timer.h>
#include <xtensa/core-macros.h>
#include <driver/rmt.h>
 #include <esp_idf_version.h>
 #if ESP_IDF_VERSION_MAJOR >= 4
 #include <hal/rmt_ll.h>
 #endif
 // battery pin
#define BATT_PIN            36
/* Config Register Control */
#define CFG_DATA GPIO_NUM_23
#define CFG_CLK GPIO_NUM_18
#define CFG_STR GPIO_NUM_0

/* Control Lines */
#define CKV GPIO_NUM_25
#define STH GPIO_NUM_26

/* Edges */
#define CKH GPIO_NUM_5

/* Data Lines */
#define D7 GPIO_NUM_22
#define D6 GPIO_NUM_21
#define D5 GPIO_NUM_27
#define D4 GPIO_NUM_2
#define D3 GPIO_NUM_19
#define D2 GPIO_NUM_4
#define D1 GPIO_NUM_32
#define D0 GPIO_NUM_33

using namespace gfx;

namespace lilygot54in7_helpers {
constexpr static const uint16_t native_width = 960;
constexpr static const uint16_t native_height = 540;
        
#define EPD_WIDTH (native_width)
#define EPD_HEIGHT (native_height)

using frame_buffer_type = gfx::bitmap<arduino::lilygot54in7::pixel_type>;
        
/////////////////////////////
// I2S Bus Implementation
/////////////////////////////


/**
 * I2S bus configuration parameters.
 */
typedef struct {
  /// GPIO numbers of the parallel bus pins.
  gpio_num_t data_0;
  gpio_num_t data_1;
  gpio_num_t data_2;
  gpio_num_t data_3;
  gpio_num_t data_4;
  gpio_num_t data_5;
  gpio_num_t data_6;
  gpio_num_t data_7;

  /// Data clock pin.
  gpio_num_t clock;

  /// "Start Pulse", enabling data input on the slave device (active low)
  gpio_num_t start_pulse;

  // Width of a display row in pixels.
  uint32_t epd_row_width;
} i2s_bus_config;

/// DMA descriptors for front and back line buffer.
/// We use two buffers, so one can be filled while the other
/// is transmitted.
typedef struct {
    volatile lldesc_t *dma_desc_a;
    volatile lldesc_t *dma_desc_b;

    /// Front and back line buffer.
    uint8_t *buf_a;
    uint8_t *buf_b;
} i2s_parallel_state_t;


/// Indicates which line buffer is currently back / front.
static int current_buffer = 0;

/// The I2S state instance.
static i2s_parallel_state_t i2s_state;

static intr_handle_t gI2S_intr_handle = NULL;

/// Indicates the device has finished its transmission and is ready again.
static volatile bool output_done = true;
/// The start pulse pin extracted from the configuration for use in the "done"
/// interrupt.
static gpio_num_t start_pulse_pin;

/// Initializes a DMA descriptor.
static void fill_dma_desc(volatile lldesc_t *dmadesc, uint8_t *buf,
                          i2s_bus_config *cfg)
{
    dmadesc->size = cfg->epd_row_width / 4;
    dmadesc->length = cfg->epd_row_width / 4;
    dmadesc->buf = buf;
    dmadesc->eof = 1;
    dmadesc->sosf = 1;
    dmadesc->owner = 1;
    dmadesc->qe.stqe_next = 0;
    dmadesc->offset = 0;
}

/// Address of the currently front DMA descriptor,
/// which uses only the lower 20bits (according to TRM)
uint32_t dma_desc_addr()
{
    return (uint32_t)(current_buffer ? i2s_state.dma_desc_a
                      : i2s_state.dma_desc_b) &
           0x000FFFFF;
}

/// Set up a GPIO as output and route it to a signal.
static void gpio_setup_out(int gpio, int sig, bool invert)
{
    if (gpio == -1)
        return;
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    gpio_set_direction((gpio_num_t)gpio, (gpio_mode_t)GPIO_MODE_DEF_OUTPUT);
    gpio_matrix_out(gpio, sig, invert, false);
}

/// Resets "Start Pulse" signal when the current row output is done.
static void IRAM_ATTR i2s_int_hdl(void *arg)
{
    i2s_dev_t *dev = &I2S1;
    if (dev->int_st.out_done) {
        gpio_set_level(start_pulse_pin, 1);
        output_done = true;
    }
    // Clear the interrupt. Otherwise, the whole device would hang.
    dev->int_clr.val = dev->int_raw.val;
}

volatile uint8_t IRAM_ATTR *i2s_get_current_buffer()
{
    return current_buffer ? i2s_state.dma_desc_a->buf : i2s_state.dma_desc_b->buf;
}

bool IRAM_ATTR i2s_is_busy()
{
    // DMA and FIFO must be done
    return !output_done || !I2S1.state.tx_idle;
}

void IRAM_ATTR i2s_switch_buffer()
{
    // either device is done transmitting or the switch must be away from the
    // buffer currently used by the DMA engine.
    while (i2s_is_busy() && dma_desc_addr() != I2S1.out_link.addr) {
    };
    current_buffer = !current_buffer;
}

void IRAM_ATTR i2s_start_line_output()
{
    output_done = false;

    i2s_dev_t *dev = &I2S1;
    dev->conf.tx_start = 0;
    dev->conf.tx_reset = 1;
    dev->conf.tx_fifo_reset = 1;
    dev->conf.rx_fifo_reset = 1;
    dev->conf.tx_reset = 0;
    dev->conf.tx_fifo_reset = 0;
    dev->conf.rx_fifo_reset = 0;
    dev->out_link.addr = dma_desc_addr();
    dev->out_link.start = 1;

    // sth is pulled up through peripheral interrupt
    gpio_set_level(start_pulse_pin, 0);
    dev->conf.tx_start = 1;
}

void i2s_bus_init(i2s_bus_config *cfg)
{
    // TODO: Why?
    gpio_num_t I2S_GPIO_BUS[] = {cfg->data_6, cfg->data_7, cfg->data_4,
                                 cfg->data_5, cfg->data_2, cfg->data_3,
                                 cfg->data_0, cfg->data_1
                                };

    gpio_set_direction(cfg->start_pulse, GPIO_MODE_OUTPUT);
    gpio_set_level(cfg->start_pulse, 1);
    // store pin in global variable for use in interrupt.
    start_pulse_pin = cfg->start_pulse;

    // Use I2S1 with no signal offset (for some reason the offset seems to be
    // needed in 16-bit mode, but not in 8 bit mode.
    int signal_base = I2S1O_DATA_OUT0_IDX;

    // Setup and route GPIOS
    for (int x = 0; x < 8; x++) {
        gpio_setup_out(I2S_GPIO_BUS[x], signal_base + x, false);
    }
    // Invert word select signal
    gpio_setup_out(cfg->clock, I2S1O_WS_OUT_IDX, true);

    periph_module_enable(PERIPH_I2S1_MODULE);

    i2s_dev_t *dev = &I2S1;

    // Initialize device
    dev->conf.tx_reset = 1;
    dev->conf.tx_reset = 0;

    // Reset DMA
    dev->lc_conf.in_rst = 1;
    dev->lc_conf.in_rst = 0;
    dev->lc_conf.out_rst = 1;
    dev->lc_conf.out_rst = 0;

    // Setup I2S config. See section 12 of Technical Reference Manual
    // Enable LCD mode
    dev->conf2.val = 0;
    dev->conf2.lcd_en = 1;

    // Enable FRAME1-Mode (See technical reference manual)
    dev->conf2.lcd_tx_wrx2_en = 1;
    dev->conf2.lcd_tx_sdx2_en = 0;

    // Set to 8 bit parallel output
    dev->sample_rate_conf.val = 0;
    dev->sample_rate_conf.tx_bits_mod = 8;

    // Half speed of bit clock in LCD mode.
    // (Smallest possible divider according to the spec).
    dev->sample_rate_conf.tx_bck_div_num = 2;

//#if defined(CONFIG_EPD_DISPLAY_TYPE_ED097OC4_LQ)
    // Initialize Audio Clock (APLL) for 120 Mhz.
    rtc_clk_apll_enable(1, 0, 0, 8, 0);
//#else
    // Initialize Audio Clock (APLL) for 80 Mhz.
// rtc_clk_apll_enable(1, 0, 0, 8, 1);
//#endif


    // Set Audio Clock Dividers
    dev->clkm_conf.val = 0;
    dev->clkm_conf.clka_en = 1;
    dev->clkm_conf.clkm_div_a = 1;
    dev->clkm_conf.clkm_div_b = 0;
    // 2 is the smallest possible divider according to the spec.
    dev->clkm_conf.clkm_div_num = 2;

    // Set up FIFO
    dev->fifo_conf.val = 0;
    dev->fifo_conf.tx_fifo_mod_force_en = 1;
    dev->fifo_conf.tx_fifo_mod = 1;
    dev->fifo_conf.tx_data_num = 32;
    dev->fifo_conf.dscr_en = 1;

    // Stop after transmission complete
    dev->conf1.val = 0;
    dev->conf1.tx_stop_en = 1;
    dev->conf1.tx_pcm_bypass = 1;

    // Configure TX channel
    dev->conf_chan.val = 0;
    dev->conf_chan.tx_chan_mod = 1;
    dev->conf.tx_right_first = 1;

    dev->timing.val = 0;

    // Allocate DMA descriptors
    i2s_state.buf_a = (uint8_t*)heap_caps_malloc(cfg->epd_row_width / 4, MALLOC_CAP_DMA);
    i2s_state.buf_b = (uint8_t*)heap_caps_malloc(cfg->epd_row_width / 4, MALLOC_CAP_DMA);
    i2s_state.dma_desc_a = (lldesc_t*)heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA);
    i2s_state.dma_desc_b = (lldesc_t*)heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA);

    // and fill them
    fill_dma_desc(i2s_state.dma_desc_a, i2s_state.buf_a, cfg);
    fill_dma_desc(i2s_state.dma_desc_b, i2s_state.buf_b, cfg);

    // enable "done" interrupt
    SET_PERI_REG_BITS(I2S_INT_ENA_REG(1), I2S_OUT_DONE_INT_ENA_V, 1,
                      I2S_OUT_DONE_INT_ENA_S);
    // register interrupt
    esp_intr_alloc(ETS_I2S1_INTR_SOURCE, 0, i2s_int_hdl, 0, &gI2S_intr_handle);

    // Reset FIFO/DMA
    dev->lc_conf.in_rst = 1;
    dev->lc_conf.out_rst = 1;
    dev->lc_conf.ahbm_rst = 1;
    dev->lc_conf.ahbm_fifo_rst = 1;
    dev->lc_conf.in_rst = 0;
    dev->lc_conf.out_rst = 0;
    dev->lc_conf.ahbm_rst = 0;
    dev->lc_conf.ahbm_fifo_rst = 0;
    dev->conf.tx_reset = 1;
    dev->conf.tx_fifo_reset = 1;
    dev->conf.rx_fifo_reset = 1;
    dev->conf.tx_reset = 0;
    dev->conf.tx_fifo_reset = 0;
    dev->conf.rx_fifo_reset = 0;

    // Start dma on front buffer
    dev->lc_conf.val =
        I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN | I2S_OUT_DATA_BURST_EN;
    dev->out_link.addr = ((uint32_t)(i2s_state.dma_desc_a));
    dev->out_link.start = 1;

    dev->int_clr.val = dev->int_raw.val;

    dev->int_ena.val = 0;
    dev->int_ena.out_done = 1;

    dev->conf.tx_start = 0;
}

void i2s_deinit()
{
    esp_intr_free(gI2S_intr_handle);

    free(i2s_state.buf_a);
    free(i2s_state.buf_b);
    free((void *)i2s_state.dma_desc_a);
    free((void *)i2s_state.dma_desc_b);

    periph_module_disable(PERIPH_I2S1_MODULE);
}

////////////////////////////
// RMT Pulse implementation
////////////////////////////


static intr_handle_t gRMT_intr_handle = NULL;

// the RMT channel configuration object
static rmt_config_t row_rmt_config;

// keep track of wether the current pulse is ongoing
volatile bool rmt_tx_done = true;

/**
 * Remote peripheral interrupt. Used to signal when transmission is done.
 */
static void IRAM_ATTR rmt_interrupt_handler(void *arg)
{
    rmt_tx_done = true;
    RMT.int_clr.val = RMT.int_st.val;
}

void rmt_pulse_init(gpio_num_t pin)
{

    row_rmt_config.rmt_mode = RMT_MODE_TX;
    // currently hardcoded: use channel 0
    row_rmt_config.channel = RMT_CHANNEL_1;

    row_rmt_config.gpio_num = pin;
    row_rmt_config.mem_block_num = 2;

    // Divide 80MHz APB Clock by 8 -> .1us resolution delay
    row_rmt_config.clk_div = 8;

    row_rmt_config.tx_config.loop_en = false;
    row_rmt_config.tx_config.carrier_en = false;
    row_rmt_config.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;
    row_rmt_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    row_rmt_config.tx_config.idle_output_en = true;

#if ESP_IDF_VERSION_MAJOR >= 4
    rmt_isr_register(rmt_interrupt_handler, 0,
                     ESP_INTR_FLAG_LEVEL3, &gRMT_intr_handle);
#else
    esp_intr_alloc(ETS_RMT_INTR_SOURCE, ESP_INTR_FLAG_LEVEL3,
                   rmt_interrupt_handler, 0, &gRMT_intr_handle);
#endif

    rmt_config(&row_rmt_config);
#if ESP_IDF_VERSION_MAJOR >= 4
    rmt_ll_enable_tx_end_interrupt(&RMT, row_rmt_config.channel, true);
#else
    rmt_set_tx_intr_en(row_rmt_config.channel, true);
#endif
}

void IRAM_ATTR pulse_ckv_ticks(uint16_t high_time_ticks,
                               uint16_t low_time_ticks, bool wait)
{
    while (!rmt_tx_done) {
    };
    volatile rmt_item32_t *rmt_mem_ptr =
        &(RMTMEM.chan[row_rmt_config.channel].data32[0]);
    if (high_time_ticks > 0) {
        rmt_mem_ptr->level0 = 1;
        rmt_mem_ptr->duration0 = high_time_ticks;
        rmt_mem_ptr->level1 = 0;
        rmt_mem_ptr->duration1 = low_time_ticks;
    } else {
        rmt_mem_ptr->level0 = 1;
        rmt_mem_ptr->duration0 = low_time_ticks;
        rmt_mem_ptr->level1 = 0;
        rmt_mem_ptr->duration1 = 0;
    }
    RMTMEM.chan[row_rmt_config.channel].data32[1].val = 0;
    rmt_tx_done = false;
    RMT.conf_ch[row_rmt_config.channel].conf1.mem_rd_rst = 1;
    RMT.conf_ch[row_rmt_config.channel].conf1.mem_owner = RMT_MEM_OWNER_TX;
    RMT.conf_ch[row_rmt_config.channel].conf1.tx_start = 1;
    while (wait && !rmt_tx_done) {
    };
}

void IRAM_ATTR pulse_ckv_us(uint16_t high_time_us, uint16_t low_time_us,
                            bool wait)
{
    pulse_ckv_ticks(10 * high_time_us, 10 * low_time_us, wait);
}

bool IRAM_ATTR rmt_busy()
{
    return !rmt_tx_done;
}

//////////////////////
// Low level Control
//////////////////////


typedef struct {
  bool ep_latch_enable : 1;
  bool power_disable : 1;
  bool pos_power_enable : 1;
  bool neg_power_enable : 1;
  bool ep_stv : 1;
  bool ep_scan_direction : 1;
  bool ep_mode : 1;
  bool ep_output_enable : 1;
} epd_config_register_t;

static epd_config_register_t config_reg;

/*
 * Write bits directly using the registers.
 * Won't work for some pins (>= 32).
 */
inline static void fast_gpio_set_hi(gpio_num_t gpio_num) {
  GPIO.out_w1ts = (1 << gpio_num);
}

inline static void fast_gpio_set_lo(gpio_num_t gpio_num) {
  GPIO.out_w1tc = (1 << gpio_num);
}

void IRAM_ATTR busy_delay(uint32_t cycles) {
  volatile unsigned long counts = XTHAL_GET_CCOUNT() + cycles;
  while (XTHAL_GET_CCOUNT() < counts) {
  };
}

inline static void IRAM_ATTR push_cfg_bit(bool bit) {
  fast_gpio_set_lo(CFG_CLK);
  if (bit) {
    fast_gpio_set_hi(CFG_DATA);
  } else {
    fast_gpio_set_lo(CFG_DATA);
  }
  fast_gpio_set_hi(CFG_CLK);
}

static void IRAM_ATTR push_cfg(epd_config_register_t *cfg) {
  fast_gpio_set_lo(CFG_STR);

  // push config bits in reverse order
  push_cfg_bit(cfg->ep_output_enable);
  push_cfg_bit(cfg->ep_mode);
  push_cfg_bit(cfg->ep_scan_direction);
  push_cfg_bit(cfg->ep_stv);

  push_cfg_bit(cfg->neg_power_enable);
  push_cfg_bit(cfg->pos_power_enable);
  push_cfg_bit(cfg->power_disable);
  push_cfg_bit(cfg->ep_latch_enable);

  fast_gpio_set_hi(CFG_STR);
}

void epd_base_init(uint32_t epd_row_width) {

  config_reg.ep_latch_enable = false;
  config_reg.power_disable = true;
  config_reg.pos_power_enable = false;
  config_reg.neg_power_enable = false;
  config_reg.ep_stv = true;
  config_reg.ep_scan_direction = true;
  config_reg.ep_mode = false;
  config_reg.ep_output_enable = false;

  /* Power Control Output/Off */
  gpio_set_direction(CFG_DATA, GPIO_MODE_OUTPUT);
  gpio_set_direction(CFG_CLK, GPIO_MODE_OUTPUT);
  gpio_set_direction(CFG_STR, GPIO_MODE_OUTPUT);
  fast_gpio_set_lo(CFG_STR);

  push_cfg(&config_reg);

  // Setup I2S
  i2s_bus_config i2s_config;
  // add an offset off dummy bytes to allow for enough timing headroom
  i2s_config.epd_row_width = epd_row_width + 32;
  i2s_config.clock = CKH;
  i2s_config.start_pulse = STH;
  i2s_config.data_0 = D0;
  i2s_config.data_1 = D1;
  i2s_config.data_2 = D2;
  i2s_config.data_3 = D3;
  i2s_config.data_4 = D4;
  i2s_config.data_5 = D5;
  i2s_config.data_6 = D6;
  i2s_config.data_7 = D7;

  i2s_bus_init(&i2s_config);

  rmt_pulse_init(CKV);
}

void epd_poweron() {
  // POWERON
  config_reg.ep_scan_direction = true;
  config_reg.power_disable = false;
  push_cfg(&config_reg);
  busy_delay(100 * 240);
  config_reg.neg_power_enable = true;
  push_cfg(&config_reg);
  busy_delay(500 * 240);
  config_reg.pos_power_enable = true;
  push_cfg(&config_reg);
  busy_delay(100 * 240);
  config_reg.ep_stv = true;
  push_cfg(&config_reg);
  fast_gpio_set_hi(STH);
  // END POWERON
}

void epd_poweroff() {
  // POWEROFF
  config_reg.pos_power_enable = false;
  push_cfg(&config_reg);
  busy_delay(10 * 240);
  config_reg.neg_power_enable = false;
  push_cfg(&config_reg);
  busy_delay(100 * 240);
  config_reg.power_disable = true;
  push_cfg(&config_reg);

  config_reg.ep_stv = false;
  push_cfg(&config_reg);

//   config_reg.ep_scan_direction = false;
//   push_cfg(&config_reg);

  // END POWEROFF
}

void epd_poweroff_all()
{
    memset(&config_reg, 0, sizeof(config_reg));
    push_cfg(&config_reg);
}

void epd_start_frame() {
  while (i2s_is_busy() || rmt_busy()) {
  };
  config_reg.ep_mode = true;
  push_cfg(&config_reg);

  pulse_ckv_us(1, 1, true);

  // This is very timing-sensitive!
  config_reg.ep_stv = false;
  push_cfg(&config_reg);
  busy_delay(240);
  pulse_ckv_us(10, 10, false);
  config_reg.ep_stv = true;
  push_cfg(&config_reg);
  pulse_ckv_us(0, 10, true);

  config_reg.ep_output_enable = true;
  push_cfg(&config_reg);

  pulse_ckv_us(1, 1, true);
}

static inline void latch_row() {
  config_reg.ep_latch_enable = true;
  push_cfg(&config_reg);

  config_reg.ep_latch_enable = false;
  push_cfg(&config_reg);
}

void IRAM_ATTR epd_skip() {
#if defined(CONFIG_EPD_DISPLAY_TYPE_ED097TC2)
  pulse_ckv_ticks(2, 2, false);
#else
  // According to the spec, the OC4 maximum CKV frequency is 200kHz.
  pulse_ckv_ticks(45, 5, false);
#endif
}

void IRAM_ATTR epd_output_row(uint32_t output_time_dus) {

  while (i2s_is_busy() || rmt_busy()) {
  };
  latch_row();

  pulse_ckv_ticks(output_time_dus, 50, false);

  i2s_start_line_output();
  i2s_switch_buffer();
}

void epd_end_frame() {
  config_reg.ep_output_enable = false;
  push_cfg(&config_reg);
  config_reg.ep_mode = false;
  push_cfg(&config_reg);
  pulse_ckv_us(1, 1, true);
  pulse_ckv_us(1, 1, true);
}

void IRAM_ATTR epd_switch_buffer() { i2s_switch_buffer(); }
uint8_t IRAM_ATTR *epd_get_current_buffer() {
  return (uint8_t *)i2s_get_current_buffer();
};

///////////////////////////
// driver core
///////////////////////////

/// The image drawing mode.
enum DrawMode {
    /// Draw black / grayscale image on a white display.
    BLACK_ON_WHITE = 1 << 0,
    /// "Draw with white ink" on a white display.
    WHITE_ON_WHITE = 1 << 1,
    /// Draw with white ink on a black display.
    WHITE_ON_BLACK = 1 << 2,
};

// number of bytes needed for one line of EPD pixel data.
#define EPD_LINE_BYTES EPD_WIDTH / 4

// status tracker for row skipping
uint32_t skipping;

#define CLEAR_BYTE 0B10101010
#define DARK_BYTE 0B01010101


/* 4bpp Contrast cycles in order of contrast (Darkest first).  */
const int contrast_cycles_4[15] = {30, 30, 20, 20, 30,  30,  30, 40, 40, 50, 50, 50, 100, 200, 300};

const int contrast_cycles_4_white[15] = {10, 10, 8, 8, 8,  8,  8, 10, 10, 15, 15, 20, 20, 100, 300};


#ifndef _swap_int
#define _swap_int(a, b)                                                    \
  {                                                                            \
    int t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

// Heap space to use for the EPD output lookup table, which
// is calculated for each cycle.
static uint8_t *conversion_lut;
static QueueHandle_t output_queue;

// output a row to the display.
static void write_row(uint32_t output_time_dus)
{
    // avoid too light output after skipping on some displays
    if (skipping) {
        //vTaskDelay(20);
    }
    skipping = 0;
    epd_output_row(output_time_dus);
}

void reorder_line_buffer(uint32_t *line_data);

void epd_init()
{
    skipping = 0;
    epd_base_init(EPD_WIDTH);

    conversion_lut = (uint8_t *)heap_caps_malloc(1 << 16, MALLOC_CAP_8BIT);
    assert(conversion_lut != NULL);
    output_queue = xQueueCreate(64, EPD_WIDTH / 2);
}

// skip a display row
void skip_row(uint8_t pipeline_finish_time)
{
    // output previously loaded row, fill buffer with no-ops.
    if (skipping == 0) {
        epd_switch_buffer();
        memset(epd_get_current_buffer(), 0, EPD_LINE_BYTES);
        epd_switch_buffer();
        memset(epd_get_current_buffer(), 0, EPD_LINE_BYTES);
        epd_output_row(pipeline_finish_time);
        // avoid tainting of following rows by
        // allowing residual charge to dissipate
        //vTaskDelay(10);
        /*
        unsigned counts = XTHAL_GET_CCOUNT() + 50 * 240;
        while (XTHAL_GET_CCOUNT() < counts) {
        };
        */
    } else if (skipping < 2) {
        epd_output_row(10);
    } else {
        //epd_output_row(5);
        epd_skip();
    }
    skipping++;
}

void epd_push_pixels(const rect16& area, short time, int color)
{

    uint8_t row[EPD_LINE_BYTES] = {0};
    int w = area.width();
    int h = area.height();

    for (uint32_t i = 0; i < w; i++) {
        uint32_t position = i + area.x1 % 4;
        uint8_t mask =
            (color ? CLEAR_BYTE : DARK_BYTE) & (0b00000011 << (2 * (position % 4)));
        row[area.x1 / 4 + position / 4] |= mask;
    }
    reorder_line_buffer((uint32_t *)row);

    epd_start_frame();

    for (int i = 0; i < EPD_HEIGHT; i++) {
        // before are of interest: skip
        if (i < area.y1) {
            skip_row(time);
            // start area of interest: set row data
        } else if (i == area.y1) {
            epd_switch_buffer();
            memcpy(epd_get_current_buffer(), row, EPD_LINE_BYTES);
            epd_switch_buffer();
            memcpy(epd_get_current_buffer(), row, EPD_LINE_BYTES);

            write_row(time * 10);
            // load nop row if done with area
        } else if (i >= area.y1 + h) {
            skip_row(time);
            // output the same as before
        } else {
            write_row(time * 10);
        }
    }
    // Since we "pipeline" row output, we still have to latch out the last row.
    write_row(time * 10);

    epd_end_frame();
}

void epd_clear_area_cycles(const rect16& area, int cycles, int cycle_time)
{
    const short white_time = cycle_time;
    const short dark_time = cycle_time;

    for (int c = 0; c < cycles; c++) {
        for (int i = 0; i < 4; i++) {
            epd_push_pixels(area, dark_time, 0);
        }
        for (int i = 0; i < 4; i++) {
            epd_push_pixels(area, white_time, 1);
        }
    }
}

void epd_clear_area(const rect16& area)
{
    epd_clear_area_cycles(area, 4, 50);
}
void epd_clear()
{
    epd_clear_area(rect16(0,0,EPD_WIDTH-1,EPD_HEIGHT-1));
}

/*
 * Reorder the output buffer to account for I2S FIFO order.
 */
void reorder_line_buffer(uint32_t *line_data)
{
    for (uint32_t i = 0; i < EPD_LINE_BYTES / 4; i++) {
        uint32_t val = *line_data;
        *(line_data++) = val >> 16 | ((val & 0x0000FFFF) << 16);
    }
}

void IRAM_ATTR calc_epd_input_4bpp(uint32_t *line_data, uint8_t *epd_input,
                                   uint8_t k, uint8_t *conversion_lut)
{

    uint32_t *wide_epd_input = (uint32_t *)epd_input;
    uint16_t *line_data_16 = (uint16_t *)line_data;

    // this is reversed for little-endian, but this is later compensated
    // through the output peripheral.
    for (uint32_t j = 0; j < EPD_WIDTH / 16; j++) {

        uint16_t v1 = *(line_data_16++);
        uint16_t v2 = *(line_data_16++);
        uint16_t v3 = *(line_data_16++);
        uint16_t v4 = *(line_data_16++);
        uint32_t pixel = conversion_lut[v1] << 16 | conversion_lut[v2] << 24 |
                         conversion_lut[v3] | conversion_lut[v4] << 8;
        wide_epd_input[j] = pixel;
    }
}

const DRAM_ATTR uint32_t lut_1bpp[256] = {
    0x0000, 0x0001, 0x0004, 0x0005, 0x0010, 0x0011, 0x0014, 0x0015, 0x0040,
    0x0041, 0x0044, 0x0045, 0x0050, 0x0051, 0x0054, 0x0055, 0x0100, 0x0101,
    0x0104, 0x0105, 0x0110, 0x0111, 0x0114, 0x0115, 0x0140, 0x0141, 0x0144,
    0x0145, 0x0150, 0x0151, 0x0154, 0x0155, 0x0400, 0x0401, 0x0404, 0x0405,
    0x0410, 0x0411, 0x0414, 0x0415, 0x0440, 0x0441, 0x0444, 0x0445, 0x0450,
    0x0451, 0x0454, 0x0455, 0x0500, 0x0501, 0x0504, 0x0505, 0x0510, 0x0511,
    0x0514, 0x0515, 0x0540, 0x0541, 0x0544, 0x0545, 0x0550, 0x0551, 0x0554,
    0x0555, 0x1000, 0x1001, 0x1004, 0x1005, 0x1010, 0x1011, 0x1014, 0x1015,
    0x1040, 0x1041, 0x1044, 0x1045, 0x1050, 0x1051, 0x1054, 0x1055, 0x1100,
    0x1101, 0x1104, 0x1105, 0x1110, 0x1111, 0x1114, 0x1115, 0x1140, 0x1141,
    0x1144, 0x1145, 0x1150, 0x1151, 0x1154, 0x1155, 0x1400, 0x1401, 0x1404,
    0x1405, 0x1410, 0x1411, 0x1414, 0x1415, 0x1440, 0x1441, 0x1444, 0x1445,
    0x1450, 0x1451, 0x1454, 0x1455, 0x1500, 0x1501, 0x1504, 0x1505, 0x1510,
    0x1511, 0x1514, 0x1515, 0x1540, 0x1541, 0x1544, 0x1545, 0x1550, 0x1551,
    0x1554, 0x1555, 0x4000, 0x4001, 0x4004, 0x4005, 0x4010, 0x4011, 0x4014,
    0x4015, 0x4040, 0x4041, 0x4044, 0x4045, 0x4050, 0x4051, 0x4054, 0x4055,
    0x4100, 0x4101, 0x4104, 0x4105, 0x4110, 0x4111, 0x4114, 0x4115, 0x4140,
    0x4141, 0x4144, 0x4145, 0x4150, 0x4151, 0x4154, 0x4155, 0x4400, 0x4401,
    0x4404, 0x4405, 0x4410, 0x4411, 0x4414, 0x4415, 0x4440, 0x4441, 0x4444,
    0x4445, 0x4450, 0x4451, 0x4454, 0x4455, 0x4500, 0x4501, 0x4504, 0x4505,
    0x4510, 0x4511, 0x4514, 0x4515, 0x4540, 0x4541, 0x4544, 0x4545, 0x4550,
    0x4551, 0x4554, 0x4555, 0x5000, 0x5001, 0x5004, 0x5005, 0x5010, 0x5011,
    0x5014, 0x5015, 0x5040, 0x5041, 0x5044, 0x5045, 0x5050, 0x5051, 0x5054,
    0x5055, 0x5100, 0x5101, 0x5104, 0x5105, 0x5110, 0x5111, 0x5114, 0x5115,
    0x5140, 0x5141, 0x5144, 0x5145, 0x5150, 0x5151, 0x5154, 0x5155, 0x5400,
    0x5401, 0x5404, 0x5405, 0x5410, 0x5411, 0x5414, 0x5415, 0x5440, 0x5441,
    0x5444, 0x5445, 0x5450, 0x5451, 0x5454, 0x5455, 0x5500, 0x5501, 0x5504,
    0x5505, 0x5510, 0x5511, 0x5514, 0x5515, 0x5540, 0x5541, 0x5544, 0x5545,
    0x5550, 0x5551, 0x5554, 0x5555
};

void IRAM_ATTR calc_epd_input_1bpp(uint8_t *line_data, uint8_t *epd_input,
                                   enum DrawMode mode)
{

    uint32_t *wide_epd_input = (uint32_t *)epd_input;

    // this is reversed for little-endian, but this is later compensated
    // through the output peripheral.
    for (uint32_t j = 0; j < EPD_WIDTH / 16; j++) {
        uint8_t v1 = *(line_data++);
        uint8_t v2 = *(line_data++);
        wide_epd_input[j] = (lut_1bpp[v1] << 16) | lut_1bpp[v2];
    }
}

static void IRAM_ATTR reset_lut(uint8_t *lut_mem, enum DrawMode mode)
{
    switch (mode) {
    case BLACK_ON_WHITE:
        memset(lut_mem, 0x55, (1 << 16));
        break;
    case WHITE_ON_BLACK:
    case WHITE_ON_WHITE:
        memset(lut_mem, 0xAA, (1 << 16));
        break;
    default:
        ESP_LOGW("epd_driver", "unknown draw mode %d!", mode);
        break;
    }
}

static void IRAM_ATTR update_LUT(uint8_t *lut_mem, uint8_t k,
                                 enum DrawMode mode)
{
    if (mode == BLACK_ON_WHITE || mode == WHITE_ON_WHITE) {
        k = 15 - k;
    }

    // reset the pixels which are not to be lightened / darkened
    // any longer in the current frame
    for (uint32_t l = k; l < (1 << 16); l += 16) {
        lut_mem[l] &= 0xFC;
    }

    for (uint32_t l = (k << 4); l < (1 << 16); l += (1 << 8)) {
        for (uint32_t p = 0; p < 16; p++) {
            lut_mem[l + p] &= 0xF3;
        }
    }
    for (uint32_t l = (k << 8); l < (1 << 16); l += (1 << 12)) {
        for (uint32_t p = 0; p < (1 << 8); p++) {
            lut_mem[l + p] &= 0xCF;
        }
    }
    for (uint32_t p = (k << 12); p < ((k + 1) << 12); p++) {
        lut_mem[p] &= 0x3F;
    }
}

void IRAM_ATTR nibble_shift_buffer_right(uint8_t *buf, uint32_t len)
{
    uint8_t carry = 0xF;
    for (uint32_t i = 0; i < len; i++) {
        uint8_t val = buf[i];
        buf[i] = (val << 4) | carry;
        carry = (val & 0xF0) >> 4;
    }
}

/*
 * bit-shift a buffer `shift` <= 7 bits to the right.
 */
void IRAM_ATTR bit_shift_buffer_right(uint8_t *buf, uint32_t len, int shift)
{
    uint8_t carry = 0x00;
    for (uint32_t i = 0; i < len; i++) {
        uint8_t val = buf[i];
        buf[i] = (val << shift) | carry;
        carry = val >> (8 - shift);
    }
}

inline uint32_t min(uint32_t x, uint32_t y)
{
    return x < y ? x : y;
}

typedef struct {
    uint8_t *data_ptr;
    SemaphoreHandle_t done_smphr;
    rect16 area;
    int frame;
    enum DrawMode mode;
} OutputParams;

void IRAM_ATTR provide_out(OutputParams *params)
{
    uint8_t line[EPD_WIDTH / 2];
    memset(line, 255, EPD_WIDTH / 2);
    rect16 area = params->area;
    uint8_t *ptr = params->data_ptr;

    if (params->frame == 0) {
        reset_lut(conversion_lut, params->mode);
    }

    update_LUT(conversion_lut, params->frame, params->mode);

    /*if (area.x < 0) {
        ptr += -area.x / 2;
    }
    if (area.y < 0) {
        ptr += (area.width / 2 + area.width % 2) * -area.y;
    }*/
    int w = area.width();
    int h = area.height();
    for (int i = 0; i < EPD_HEIGHT; i++) {
        if (i < area.y1 || i >= area.y1 + h) {
            continue;
        }

        uint32_t *lp;
        bool shifted = false;
        if (w == EPD_WIDTH && area.x1 == 0) {
            lp = (uint32_t *)ptr;
            ptr += EPD_WIDTH / 2;
        } else {
            uint8_t *buf_start = (uint8_t *)line;
            uint32_t line_bytes = w / 2 + w % 2;
            if (area.x1 >= 0) {
                buf_start += area.x1 / 2;
            } else {
                // reduce line_bytes to actually used bytes
                line_bytes += area.x1 / 2;
            }
            line_bytes =
                min(line_bytes, EPD_WIDTH / 2 - (uint32_t)(buf_start - line));
            memcpy(buf_start, ptr, line_bytes);
            ptr += w / 2 + w % 2;

            // mask last nibble for uneven width
            if (w % 2 == 1 && area.x1 / 2 + w / 2 + 1 < EPD_WIDTH) {
                *(buf_start + line_bytes - 1) |= 0xF0;
            }
            if (area.x1 % 2 == 1 && area.x1 < EPD_WIDTH) {
                shifted = true;
                // shift one nibble to right
                nibble_shift_buffer_right(
                    buf_start, min(line_bytes + 1, (uint32_t)line + EPD_WIDTH / 2 -
                                   (uint32_t)buf_start));
            }
            lp = (uint32_t *)line;
        }
        xQueueSendToBack(output_queue, lp, portMAX_DELAY);
        if (shifted) {
            memset(line, 255, EPD_WIDTH / 2);
        }
    }

    xSemaphoreGive(params->done_smphr);
    vTaskDelay(portMAX_DELAY);
}

void IRAM_ATTR feed_display(OutputParams *params)
{
    rect16 area = params->area;
    const int *contrast_lut = contrast_cycles_4;
    switch (params->mode) {
    case WHITE_ON_WHITE:
    case BLACK_ON_WHITE:
        contrast_lut = contrast_cycles_4;
        break;
    case WHITE_ON_BLACK:
        contrast_lut = contrast_cycles_4_white;
        break;
    }

    epd_start_frame();
    int h = area.height();
    for (int i = 0; i < EPD_HEIGHT; i++) {
        if (i < area.y1 || i >= area.y1 + h) {
            skip_row(contrast_lut[params->frame]);
            continue;
        }
        uint8_t output[EPD_WIDTH / 2];
        xQueueReceive(output_queue, output, portMAX_DELAY);
        calc_epd_input_4bpp((uint32_t *)output, epd_get_current_buffer(),
                            params->frame, conversion_lut);
        write_row(contrast_lut[params->frame]);
    }
    if (!skipping) {
        // Since we "pipeline" row output, we still have to latch out the last row.
        write_row(contrast_lut[params->frame]);
    }
    epd_end_frame();

    xSemaphoreGive(params->done_smphr);
    vTaskDelay(portMAX_DELAY);
}

void IRAM_ATTR epd_draw_image(rect16 area, uint8_t *data, enum DrawMode mode)
{
    uint8_t line[EPD_WIDTH / 2];
    memset(line, 255, EPD_WIDTH / 2);
    uint8_t frame_count = 15;

    SemaphoreHandle_t fetch_sem = xSemaphoreCreateBinary();
    SemaphoreHandle_t feed_sem = xSemaphoreCreateBinary();
    vTaskDelay(10);
    for (uint8_t k = 0; k < frame_count; k++) {
        OutputParams p1;
            p1.area = area;
            p1.data_ptr = data;
            p1.frame = k;
            p1.mode = mode;
            p1.done_smphr = fetch_sem;
        
        OutputParams p2;
            p2.area = area;
            p2.data_ptr = data;
            p2.frame = k;
            p2.mode = mode;
            p2.done_smphr = feed_sem;
        

        TaskHandle_t t1, t2;
        xTaskCreatePinnedToCore((void (*)(void *))provide_out, "provide_out", 8000,
                                &p1, 10, &t1, 0);
        xTaskCreatePinnedToCore((void (*)(void *))feed_display, "render", 8000, &p2,
                                10, &t2, 1);

        xSemaphoreTake(fetch_sem, portMAX_DELAY);
        xSemaphoreTake(feed_sem, portMAX_DELAY);

        vTaskDelete(t1);
        vTaskDelete(t2);
        vTaskDelay(5);
    }
    vSemaphoreDelete(fetch_sem);
    vSemaphoreDelete(feed_sem);
}

void IRAM_ATTR epd_draw_grayscale_image(rect16 area, uint8_t *data)
{
    epd_draw_image(area, data, BLACK_ON_WHITE);
}

int vref = 1100;

///////////////////////////
// driver implementation
///////////////////////////

// driver state
static uint8_t* frame_buffer = nullptr;
static int suspend_count = 0;
static rect16 suspend_bounds = {uint16_t(-1),uint16_t(-1),uint16_t(-1),uint16_t(-1)};
static bool is_initialized = false;
static bool is_sleep = false;
static bool is_auto_clear = true;
static uint8_t rot = 0;
static void expand_rect(rect16& dst,const rect16& src) {
    if(dst.x1==uint16_t(-1)||src.x1<dst.x1) {
        dst.x1=src.x1;
    }
    if(dst.x2==uint16_t(-1)||src.x2>dst.x2) {
        dst.x2=src.x2;
    }
    if(dst.y1==uint16_t(-1)||src.y1<dst.y1) {
        dst.y1=src.y1;
    }
    if(dst.y2==uint16_t(-1)||src.y2>dst.y2) {
        dst.y2=src.y2;
    }
}

void update_display() {
    if(suspend_count>0){
        return;
    } 
    if(!is_initialized || is_sleep) {
        return;
    }
    if(suspend_bounds.x1==uint16_t(-1)) {
        return;
    }
    if(suspend_bounds.width()&1) {
        if(suspend_bounds.x2!=EPD_WIDTH-1) {
            suspend_bounds.x2|=1;
        } else {
            suspend_bounds.x1&=(~1);
        }
    }
    if(suspend_bounds==rect16(0,0,uint16_t(EPD_WIDTH-1),uint16_t(EPD_HEIGHT-1))) {
        epd_clear();
        epd_draw_grayscale_image(suspend_bounds,frame_buffer);
    } else {
        if(is_auto_clear) {
            epd_clear_area(suspend_bounds);
        }
        uint8_t* buf = nullptr;
        buf = (uint8_t*)ps_malloc(suspend_bounds.width()*suspend_bounds.height()/2);
        if(buf!=nullptr) {
            int w = suspend_bounds.width();
            for(int y = suspend_bounds.y1;y<=suspend_bounds.y2;++y) {
                uint8_t* fp = frame_buffer+(((y*EPD_WIDTH)+suspend_bounds.x1)/2);
                int yy = y-suspend_bounds.y1;
                memcpy(buf+((yy*w)/2),fp,w/2);
            }
            epd_draw_grayscale_image(rect16(suspend_bounds.x1,suspend_bounds.y1,w+suspend_bounds.x1-1,suspend_bounds.height()+suspend_bounds.y1-1),buf);
            free(buf);
        } else {
            int w = suspend_bounds.width();
            for(int y = suspend_bounds.y1;y<=suspend_bounds.y2;++y) {
                uint8_t* fp = frame_buffer+(((y*EPD_WIDTH)+suspend_bounds.x1)/2);
                epd_draw_grayscale_image(rect16(suspend_bounds.x1,y,w+suspend_bounds.x1-1,y+1),fp);
            }   
        }
    }
    suspend_bounds = {uint16_t(-1),uint16_t(-1),uint16_t(-1),uint16_t(-1)};
}
void translate_rotation(point16& location) {
    if(0==(rot&1)) {
        uint16_t tmp = location.x;
        location.x = location.y;
        location.y = tmp;
    }
    switch(rot) {
        case 0:
            location.y = EPD_HEIGHT-location.y-1;
            break;
        case 2:
            location.x = EPD_WIDTH-location.x-1;
            break;
        case 3:
            location.x = EPD_WIDTH-location.x-1;
            location.y = EPD_HEIGHT-location.y-1;
            break;

    }
}
void translate_rotation(rect16& bounds) {
    if(0==(rot&1)) {

        uint16_t tmp = bounds.x1;
        bounds.x1 = bounds.y1;
        bounds.y1 = tmp;
        tmp = bounds.x2;
        bounds.x2 = bounds.y2;
        bounds.y2 = tmp;
    }
    switch(rot) {
        case 0:
            bounds.y1 = EPD_HEIGHT-bounds.y1-1;
            bounds.y2 = EPD_HEIGHT-bounds.y2-1;
            break;
        case 2:
            bounds.x1 = EPD_WIDTH-bounds.x1-1;
            bounds.x2 = EPD_WIDTH-bounds.x2-1;
            break;
        case 3:
            bounds.x1 = EPD_WIDTH-bounds.x1-1;
            bounds.x2 = EPD_WIDTH-bounds.x2-1;
            bounds.y1 = EPD_HEIGHT-bounds.y1-1;
            bounds.y2 = EPD_HEIGHT-bounds.y2-1;
            break;
    }
}
rect16 native_bounds() {
    return {0,0,uint16_t(EPD_WIDTH-1),uint16_t(EPD_HEIGHT-1)};
}
} // lilygot54in7_helpers

using namespace lilygot54in7_helpers;

namespace arduino {
    gfx_result lilygot54in7::initialize() {
        if(!is_initialized) {
            const size_t size = frame_buffer_type::sizeof_buffer({native_width,native_height});
            frame_buffer = (uint8_t*)ps_malloc(size);
            if(frame_buffer==nullptr) {
                return gfx_result::out_of_memory;
            }
            memset(frame_buffer,0xFF,size);
            epd_init();
            epd_poweron();
            is_sleep = false;
            rot = 0;
            suspend_count = 0;
            suspend_bounds = {uint16_t(-1),uint16_t(-1),uint16_t(-1),uint16_t(-1)};
            esp_adc_cal_characteristics_t adc_chars;
            esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
            if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
                vref = adc_chars.vref;
            }
            
            is_initialized = true;
        } else if(is_sleep) {
            epd_poweron();
            is_sleep = false;
        }
        return gfx_result::success;
    }
    
    bool lilygot54in7::initialized() const {
        return is_initialized;
    }
    void lilygot54in7::sleep(bool all) {
        if(is_initialized && !is_sleep) {
            if(all) {
                epd_poweroff_all();
            } else {
                epd_poweroff();
            }
            is_sleep = true;
        }
    }
    void lilygot54in7::invalidate() {
        if(is_initialized) {
            suspend_bounds = bounds();
        }
    }
    uint8_t lilygot54in7::rotation() const {
        return rot;
    }
    void lilygot54in7::rotation(uint8_t value) {
        uint8_t orot = rot;
        rot= value &3;
        if(rot!=orot) {
            switch(rot) {
                case 0:
                    break;
            }
        }
    }
    void lilygot54in7::wash() {
        if(!is_initialized || is_sleep) {
            return;
        }
        delay(10);
        epd_clear();
        int i;
        rect16 b = bounds();
        translate_rotation(b);    
        for (i = 0; i < 20; i++)
        {
            epd_push_pixels(b, 50, 0);
            delay(500);
        }
        epd_clear();
        for (i = 0; i < 40; i++)
        {
            epd_push_pixels(b, 50, 1);
            delay(500);
        }
        epd_clear();
        epd_poweroff_all();
        delay(10);
        epd_poweron();
        if(suspend_count<1) {
            update_display();
        }
    }
    bool lilygot54in7::auto_clear() const {
        return is_auto_clear;
    }
    void lilygot54in7::auto_clear(bool value) {
        is_auto_clear=value;
    }
    float lilygot54in7::voltage() const {
        if(!is_initialized || is_sleep) {
            return NAN;
        }
        delay(10); // Make adc measurement more accurate
        uint16_t v = analogRead(BATT_PIN);
        return ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    }
    size16 lilygot54in7::dimensions() const {
        if(rot&1) {
            return {native_width,native_height};
        } else {
            return {native_height,native_width};
        }
    }
    rect16 lilygot54in7::bounds() const {
        return dimensions().bounds();
    }
    gfx_result lilygot54in7::clear(const rect16& bounds) {
        gfx_result r = initialize();
        if(r!=gfx_result::success) {
            return r;
        }
        rect16 rr =bounds;
        translate_rotation(rr);
        if(!native_bounds().intersects(rr)) {
            return gfx_result::success;
        }
        rr = rr.crop(native_bounds()).normalize();
        frame_buffer_type fb({native_width,native_height},frame_buffer);
        fb.fill(rr,pixel_type(true,1));
        epd_clear_area(rr);
        expand_rect(suspend_bounds,rr);
        update_display();
        return gfx_result::success;
    }
    gfx_result lilygot54in7::fill(const rect16& bounds,pixel_type color) {
        gfx_result r = initialize();
        if(r!=gfx_result::success) {
            return r;
        }
        rect16 rr = bounds;
        translate_rotation(rr);
        if(!native_bounds().intersects(rr)) {
            return gfx_result::success;
        }
        rr = rr.crop(native_bounds()).normalize();
        frame_buffer_type fb({native_width,native_height},frame_buffer);
        fb.fill(rr,color);
        expand_rect(suspend_bounds,rr);
        update_display();
        return gfx_result::success;
    }
    gfx_result lilygot54in7::point(point16 location,pixel_type color) {
        gfx_result r = initialize();
        if(r!=gfx_result::success) {
            return r;
        }
        translate_rotation(location);
        if(!native_bounds().intersects(location)) {
            return gfx_result::success;
        }
        frame_buffer_type fb({native_width,native_height},frame_buffer);
        const rect16 bounds = {location.x,location.y,location.x,location.y};
        fb.point(location,color);
        expand_rect(suspend_bounds,bounds);
        update_display();
        return gfx_result::success;
    }
    gfx_result lilygot54in7::point(point16 location,pixel_type* out_color) const {
        if(!is_initialized) {
            return gfx_result::invalid_state;
        }
        translate_rotation(location);
        return frame_buffer_type::point({native_width,native_height},frame_buffer,location,out_color);
    }
    gfx_result lilygot54in7::suspend() {
        gfx_result r = initialize();
        if(r!=gfx_result::success) {
            return r;
        }
        ++suspend_count;
        return gfx_result::success;
    }
    gfx_result lilygot54in7::resume(bool force) {
        gfx_result r = initialize();
        if(r!=gfx_result::success) {
            return r;
        }
        --suspend_count;
        if(force || suspend_count<=0) {
            suspend_count = 0;
            update_display();
            return gfx_result::success;
        }
        return gfx_result::success;
    }
}