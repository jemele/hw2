// Joshua Emele <jemele@acm.org>
#include <stdio.h>
#include "platform.h"
#include <xscugic.h>
#include <xiicps.h>
#include <xil_cache.h>
#include <xil_exception.h>
#include <xttcps.h>
#include <xgpio.h>
#include <xgpiops.h>
#include <xscuwdt.h>
#include <sleep.h>
#include <xtime_l.h>
#include "font_5x7.h"
#include "ff.h"

// A queue, usefull for scheduling.
#define QUEUE_MAXSIZE 10
typedef struct {
    int head;
    int tail;
    void *q[QUEUE_MAXSIZE];
} queue_t;

static queue_t *schedulerq = 0;

// Initialize the queue.
static void queue_init(queue_t *q)
{
    q->head = q->tail = -1;
}

// Return 1 if the queue is full, 0 otherwise.
// The queue is full if the tail wraps to the head.
static int queue_isfull(queue_t *q)
{
    return (q->tail - QUEUE_MAXSIZE) == q->head;
}

// Return 1 if the queue is empty, 0 otherwise.
// The queue is empty if the head and tail are the same.
static int queue_isempty(queue_t *q)
{
    return q->tail == q->head;
}

// Enqueue an item onto the queue.
// Return non-zero on failure.
static int queue_enqueue(queue_t *q, void *p)
{
    if (queue_isfull(q)) {
        return XST_FAILURE;
    }
    q->tail++;
    q->q[q->tail % QUEUE_MAXSIZE] = p;
    return 0;
}

// Dequeue an item from the queue.
// Return non-zero on failure.
static int queue_dequeue(queue_t *q, void **p)
{
    if (queue_isempty(q)) {
        return XST_FAILURE;
    }
    q->head++;
    *p = q->q[q->head % QUEUE_MAXSIZE];
    return 0;
}

// Test the queue implementation
// Return non-zero on failure.
static int queue_test()
{
    queue_t q;
    queue_init(&q);

    // enqueue to capacity.
    printf("queue enqueue test\n");
    int i;
    for (i = 0; i < QUEUE_MAXSIZE; ++i) {
        if (queue_enqueue(&q, (void*)i)) {
            printf("queue failed for %d\n", i);
            return XST_FAILURE;
        }
    }

    // Verify dequeue matches enqueue.
    printf("queue dequeue test\n");
    for (i = 0; i < QUEUE_MAXSIZE; ++i) {
        int n;
        if (queue_dequeue(&q,(void**)&n)) {
            printf("dequeue failed for %d\n", i);
            return XST_FAILURE;
        }
        if (n != i) {
            printf("dequeue failed %d %d\n", n, i);
            return XST_FAILURE;
        }
    }

    // Verify mingled queue/dequeue.
    printf("queue interleave test\n");
    for (i = 0; i < QUEUE_MAXSIZE; ++i) {
        if (queue_enqueue(&q, (void*)i)) {
            printf("queue failed for %d\n", i);
            return XST_FAILURE;
        }
        int n;
        if (queue_dequeue(&q, (void**)&n)) {
            printf("dequeue failed for %d\n", i);
            return XST_FAILURE;
        }
        if (n != i) {
            printf("dequeue failed %d %d\n", n, i);
            return XST_FAILURE;
        }
    }
    return 0;
}

// Used by the mmc/sdcard driver.
u32 FlashReadBaseAddress = XPAR_PS7_SD_0_S_AXI_BASEADDR;

static const u8 oled_addr = 0x3c;
static const int oled_reset_pin = 12;
static const int i2c_write_delay = 300; // us

// Write a command out to the specified i2c device.
void i2c_command(XIicPs *device, u8 addr, u8 command)
{
    const u8 buf[] = {0x80,command};
    XIicPs_MasterSend(device, (u8*)buf, sizeof(buf), addr);
    usleep(i2c_write_delay);
}

// Write data out to the specified i2c device.
void i2c_data(XIicPs *device, u8 addr, u8 data)
{
    const u8 buf[] = {0x40,data};
    XIicPs_MasterSend(device, (u8*)buf, sizeof(buf), addr);
    usleep(i2c_write_delay);
}

// A font descriptor.
typedef struct {
    const u8 * base;
    int size;
    char start;
    int width;
    int pad;
} font_t;

// Tailor the font to the display.
const font_t font = {
    .base = Font5x7,
    .size = sizeof(Font5x7),
    .start = ' ',
    .width = 5,
    .pad = 128%5
};

// Display a character on the device.
// Map the character onto the font index, and then write the font data out.
void display_character(XIicPs *device, u8 addr, char c)
{
    int index = (font.width * (c - font.start));
    if ((index < 0) || (index >= font.size)) {

        // default to '?' if unknown.
        // This isn't optimal, but it's good enough for now.
        c = '?';
        index = (font.width * (c - font.start));
    }

    int i;
    for (i = 0; i < font.width; ++i) {
        i2c_data(device, addr, font.base[index+i]);
    }
    for (i = 0; i < font.pad; ++i) {
        i2c_data(device, addr, 0);
    }
    putchar(c);
}

// An i2c device context.
typedef struct {
    int id;
    int clock;
    int clear_options;
    int options;

    XIicPs_Config *config;
    XIicPs device;
} i2c_t;

// Set Page Start: 1 0 1 1 0 X2 X1 X0
static void ssd1306_set_page_start(i2c_t *i2c, u8 page)
{
    const u8 buf[] = {0x80,0xb0|(page&0x7)};
    XIicPs_MasterSend(&i2c->device, (u8*)buf, sizeof(buf), oled_addr);
    usleep(i2c_write_delay);
}

// Clear a line and return the cursor to the beginning of the line.
static void ssd1306_clear_line(i2c_t *i2c, u8 line)
{
    ssd1306_set_page_start(i2c, line);
    int i;
    for (i = 0; i < 128; ++i) {
        i2c_data(&i2c->device, oled_addr, 0);
    }
    ssd1306_set_page_start(i2c, line);
    putchar('\n');
}

// Display a word to the display, with the following semantics: Words may only
// break across lines if 3+ characters can first be rendered; otherwise, pad
// out the current line. This assumes we can support 16 characters per line,
// and that there are 8 lines on the display.
// @todo there's probably a better way to do this.
// @note This emulates the display output on the console for debugging purposes.
void display_word(i2c_t *i2c, u8 addr, char *w, int n)
{
    int i;
    static u8 x = 0, y = 0;

    // If the word is multiline, check for wrapping/padding, and make it so.
    static const int max_chars = 16;
    static const int max_lines = 8;
    const int remaining = max_chars - x;

    if ((n > remaining) && (remaining < 3)) {
        x = 0;
        y = (y+1)%max_lines;
        for (i = 0; i < remaining; ++i) {
            display_character(&i2c->device, addr, ' ');
        }
        ssd1306_clear_line(i2c, y);
    }

    // Keep track of line breaks.
    for (i = 0; i < n; ++i) {

        // If this is the beginning of the line, eat whitespace.
        if (!x && w[i] == ' ') {
            continue;
        }

        // Otherwise, display the character. Blank line wraps to make things
        // easier on the reader.
        display_character(&i2c->device, addr, w[i]);
        x=(x+1)%max_chars;
        if (!x) {
            y=(y+1)%max_lines;
            ssd1306_clear_line(i2c, y);
        }
    }
}

// Initialize an i2c device.
static int initialize_i2c(i2c_t *i2c)
{
    i2c->config = XIicPs_LookupConfig(i2c->id); //XPAR_PS7_I2C_0_DEVICE_ID);
    if (!i2c->config) {
        printf("XIicPs_LookupConfig failed\n");
        return XST_FAILURE;
    }
    printf("i2c initialization\n");
    int status = XIicPs_CfgInitialize(&i2c->device, i2c->config,
            i2c->config->BaseAddress);
    if (status != XST_SUCCESS) {
        printf("XIicPs_CfgInitialize failed %d\n", status);
        return status;
    }
    status = XIicPs_SelfTest(&i2c->device);
    if (status != XST_SUCCESS) {
        printf("XIicPs_SelfTest failed %d\n", status);
        return status;
    }
    status = XIicPs_SetSClk(&i2c->device, i2c->clock); //100000);
    if (status != XST_SUCCESS) {
        printf("XIicPs_SetSClk failed %d\n", status);
        return status;
    }
    status = XIicPs_ClearOptions(&i2c->device, i2c->clear_options);//XIICPS_10_BIT_ADDR_OPTION);
    if (status != XST_SUCCESS) {
        printf("XIicPs_ClearOptions failed %d\n", status);
        return status;
    }
    status = XIicPs_SetOptions(&i2c->device, i2c->options);//XIICPS_7_BIT_ADDR_OPTION);
    if (status != XST_SUCCESS) {
        printf("XIicPs_SetOptions failed %d\n", status);
        return status;
    }
    return 0;
}

// The ttc driver struct.
typedef struct {
    int id;
    int interrupt;
    Xil_ExceptionHandler isr;
    void *isr_context;
    u32 frequency_hz;
    u16 options;

    XTtcPs_Config *config;
    XTtcPs device;
    u16 interval;
    u8 prescalar;

} ttc_t;

// Set the ttc frequency specified in the driver context.
// Note that the calculation can fail: Upon unsuccessful calculation, Interval
// and Prescaler are set to 0xFF(FF) for their maximum values to signal the
// caller of failure.
static int ttc_set_frequency(ttc_t *ttc)
{
    XTtcPs_CalcIntervalFromFreq(&ttc->device, ttc->frequency_hz,
        &ttc->interval, &ttc->prescalar);
    if ((~ttc->interval | ~ttc->prescalar) == 0) {
        return XST_FAILURE;
    }
    XTtcPs_SetInterval(&ttc->device, ttc->interval);
    XTtcPs_SetPrescaler(&ttc->device, ttc->prescalar);
    return 0;
}

// The ttc0 interrupt service routine.
static void ttc0_isr(void *context)
{
    if (!context) {
        return;
    }
    ttc_t *ttc = (ttc_t*)context;
    const int status = XTtcPs_GetInterruptStatus(&ttc->device);
    XTtcPs_ClearInterruptStatus(&ttc->device, status);

    // Add an item to the scheduler runqueue.
    // This gives us a chance to test an approach to scheduling.
    if (schedulerq) {
        queue_enqueue(schedulerq, (void*)0);
    }
}

// The timer1 interrupt service routine, which toggles led mio7.
static void ttc1_isr(void *context)
{
    if (!context) {
        return;
    }
    ttc_t *ttc = (ttc_t*)context;
    const int status = XTtcPs_GetInterruptStatus(&ttc->device);
    XTtcPs_ClearInterruptStatus(&ttc->device, status);

    const int pin = 7;
    XGpioPs *gpio = ttc->isr_context;
    if (!gpio) {
        return;
    }
    const int pin_state = XGpioPs_ReadPin(gpio, pin);
    XGpioPs_WritePin(gpio, pin, !pin_state);
}

// The terminate flag. If set, the program should terminate.
volatile int terminate = 0;
volatile int display_update_delay_ms = 1000;
static const int display_update_minimum_delay_ms = 50;

// Scrolling settings.
static const int scroll_update_maximum = 16;
static const int scroll_update_minimum = 1;
volatile int scroll_update = 1;

// The buttons interrupt service routine.
static void buttons_isr(void *context)
{
    enum button_state {
        button_up       = 1<<0,
        button_right    = 1<<1,
        button_left     = 1<<2,
        button_down     = 1<<3,
        button_center   = 1<<4,
    };

    if (!context) {
        return;
    }
    XGpio *buttons = (XGpio*)context;
    XGpio_InterruptDisable(buttons, XGPIO_IR_CH1_MASK);
    if ((XGpio_InterruptGetStatus(buttons) & XGPIO_IR_CH1_MASK) !=
            XGPIO_IR_CH1_MASK) {
        return;
    }

    // read the button state and respond.
    const int state = XGpio_DiscreteRead(buttons, 1);
    switch (state) {

    // Increase the display update delay.
    case button_up:
        display_update_delay_ms += display_update_minimum_delay_ms;
        break;

    // Decrease the display update delay.
    case button_down:
        if (display_update_delay_ms > display_update_minimum_delay_ms) {
            display_update_delay_ms -= display_update_minimum_delay_ms;
        }
        break;

    // Terminate the application.
    case button_center:
        terminate = 1;
        break;

    case button_right:
        if (scroll_update < scroll_update_maximum) {
            ++scroll_update;
        }
        break;

    case button_left:
        if (scroll_update > scroll_update_minimum) {
            --scroll_update;
        }
        break;
    }
    XGpio_InterruptClear(buttons, XGPIO_IR_CH1_MASK);
    XGpio_InterruptEnable(buttons, XGPIO_IR_CH1_MASK);
}


// Initialize internal gpio.
// This is used to control the mio7 led.
static int initialize_gpio(XGpioPs *gpio)
{
    XGpioPs_Config *gpio_config =
        XGpioPs_LookupConfig(XPAR_PS7_GPIO_0_DEVICE_ID);
    if (!gpio_config) {
        printf("XGpioPs_LookupConfig failed\n");
        return XST_FAILURE;
    }

    printf("gpio initialization\n");
    int status = XGpioPs_CfgInitialize(gpio, gpio_config, gpio_config->BaseAddr);
    if (status != XST_SUCCESS) {
        printf("XGpioPs_CfgInitialize failed %d\n", status);
        return status;
    }

    printf("gpio selftest\n");
    status = XGpioPs_SelfTest(gpio);
    if (status != XST_SUCCESS) {
        printf("XGpioPs_SelfTest %d\n", status);
        return status;
    }

    printf("lighting up mio7\n");
    XGpioPs_SetDirectionPin(gpio, 7, 1);
    XGpioPs_SetOutputEnablePin(gpio, 7, 1);
    XGpioPs_WritePin(gpio, 7, 1);
    return 0;
}

// Initialize the global interrupt controller.
// We'll need this to configure interrupt sources.
static int initialize_gic(XScuGic *gic)
{
    XScuGic_Config *gic_config = XScuGic_LookupConfig(XPAR_PS7_SCUGIC_0_DEVICE_ID);
    if (!gic_config) {
        printf("XScuGic_LookupConfig failed\n");
        return XST_FAILURE;
    }

    printf("initializing gic\n");
    int status = XScuGic_CfgInitialize(gic, gic_config,
        gic_config->CpuBaseAddress);
    if (status != XST_SUCCESS) {
        printf("XScuGic_CfgInitialize failed %d\n", status);
        return status;
    }
    return 0;
}

// Initialize ttc0.
// This will configure and enable interrupts for the device.
static int initialize_ttc(XScuGic *gic, ttc_t *ttc)
{
    ttc->config = XTtcPs_LookupConfig(ttc->id);
    if (!ttc->config) {
        printf("XTtcPs_LookupConfig failed\n");
        return XST_FAILURE;
    }

    // https://github.com/joshuaspence/ThesisCode/blob/master/TopN_Outlier_Pruning_Block/FPGA/SDK/demo_threadx/src/demo_threadx.c
    //! @note This is required to use ttc multiple times without a board reset.
    printf("stopping ttc\n");
    XTtcPs_WriteReg(ttc->config->BaseAddress, XTTCPS_CNT_CNTRL_OFFSET, 0x33);
    XTtcPs_ReadReg(ttc->config->BaseAddress, XTTCPS_ISR_OFFSET);

    printf("initializing ttc\n");
    int status = XTtcPs_CfgInitialize(&ttc->device, ttc->config,
        ttc->config->BaseAddress);
    if (status != XST_SUCCESS) {
        printf("XTtcPs_CfgInitialize failed %d\n", status);
        return status;
    }

    printf("ttc setting options\n");
    status = XTtcPs_SetOptions(&ttc->device, ttc->options);
    if (status != XST_SUCCESS) {
        printf("XTtcPs_SetOptions failed %d\n", status);
        return status;
    }

    printf("setting ttc frequency\n");
    status = ttc_set_frequency(ttc);
    if (status != XST_SUCCESS) {
        printf("ttc_set_frequency failed %d\n", status);
        return status;
    }

    // Configure individual interrupt sources.
    // Begin by initializing the ttc0 interrupt.
    printf("connect ttc interrupt handler\n");
    status = XScuGic_Connect(gic, ttc->interrupt, ttc->isr, ttc);
    if (status != XST_SUCCESS) {
        printf("XScuGic_Connect failed %d\n", status);
        return status;
    }

    printf("enabling ttc interrupt\n");
    XScuGic_Enable(gic, ttc->interrupt);

    printf("enabling ttc interval mask\n");
    XTtcPs_EnableInterrupts(&ttc->device, XTTCPS_IXR_INTERVAL_MASK);

    printf("starting ttc\n");
    XTtcPs_Start(&ttc->device);
    return 0;
}

// Initialize the axi gpio used by the emio pushbuttons.
// This will configure and enable interrupts for the buttons.
static int initialize_axi_gpio(XScuGic *gic, XGpio *buttons)
{
    // Configure the buttons interrupt.
    printf("initializing axi gpio for buttons\n");
    int status = XGpio_Initialize(buttons, XPAR_AXI_GPIO_0_DEVICE_ID);
    if (status != XST_SUCCESS) {
        printf("XGpio_Initialize failed %d\n", status);
        return status;
    }
    // Trigger on the rising edge, since the debounce will only assert when
    // ready.
    printf("setting button trigger type\n");
    XScuGic_SetPriorityTriggerType(gic, XPAR_FABRIC_GPIO_0_VEC_ID,
        0xa, 3);

    printf("connect buttons interrupt handler\n");
    status = XScuGic_Connect(gic, XPAR_FABRIC_GPIO_0_VEC_ID,
        buttons_isr, buttons);
    if (status != XST_SUCCESS) {
        printf("XScuGic_Connect failed %d\n", status);
        return status;
    }

    printf("enabling buttons intterupt\n");
    XScuGic_Enable(gic, XPAR_FABRIC_GPIO_0_VEC_ID);

    printf("enable channel 1 interrupts\n");
    XGpio_InterruptEnable(buttons, XGPIO_IR_CH1_MASK);
    XGpio_InterruptGlobalEnable(buttons);
    return 0;
}

// Enable the interrupts for the global interrupt controller.
static void enable_gic_interrupts(XScuGic *gic)
{
    printf("initializating exceptions\n");
    Xil_ExceptionInit();

    printf("connect gic interrupt handler\n");
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
        (Xil_ExceptionHandler)XScuGic_InterruptHandler, gic);

    printf("enabling intterupts\n");
    Xil_ExceptionEnable();
}

// Initialize the display.
static void ssd1306_reset(i2c_t *i2c, XGpioPs *gpio, int reset_pin)
{
    printf("initializing ssd1306\n");
    XGpioPs_SetDirectionPin(gpio, reset_pin, 1);
    XGpioPs_SetOutputEnablePin(gpio, reset_pin, 1);
    XGpioPs_WritePin(gpio, reset_pin, 0);
    usleep(300);
    XGpioPs_WritePin(gpio, reset_pin, 1);

    // ssd1306 initialization sequence, derived from:
    // https://github.com/Defragster/ssd1306xled/blob/master/ssd1306xled.cpp
    const u8 seq[] = {
    0xAE,           // Display OFF (sleep mode)
    0x20, 0b00,     // Set Memory Addressing Mode
                    // 00=Horizontal Addressing Mode; 01=Vertical Addressing Mode;
                    // 10=Page Addressing Mode (RESET); 11=Invalid
    0xB0,           // Set Page Start Address for Page Addressing Mode, 0-7
    0xC8,           // Set COM Output Scan Direction
    0x00,           // ---set low column address
    0x10,           // ---set high column address
    0x40,           // --set start line address
    0x81, 0x3F,     // Set contrast control register
    0xA1,           // Set Segment Re-map. A0=address mapped; A1=address 127 mapped.
    0xA6,           // Set display mode. A6=Normal; A7=Inverse
    0xA8, 0x3F,     // Set multiplex ratio(1 to 64)
    0xA4,           // Output RAM to Display
                    // 0xA4=Output follows RAM content; 0xA5,Output ignores RAM content
    0xD3, 0x00,     // Set display offset. 00 = no offset
    0xD5,           // --set display clock divide ratio/oscillator frequency
    0xF0,           // --set divide ratio
    0xD9, 0x22,     // Set pre-charge period
    0xDA, 0x12,     // Set com pins hardware configuration
    0xDB,           // --set vcomh
    0x20,           // 0x20,0.77xVcc
    0x8D, 0x14,     // Set DC-DC enable
    0xAF            // Display ON in normal mode
    };

    // Write out the oled initialization sequence.
    int i;
    for (i = 0; i < sizeof(seq)/sizeof(*seq); ++i) {
       i2c_command(&i2c->device, oled_addr, seq[i]);
    }
}

// Issue the set display start line command.
// This can be used to support scrolling.
static void ssd1306_set_display_start_line(i2c_t *i2c, int line)
{
    const u8 buf[] = {0x80,1<<6|line};
    XIicPs_MasterSend(&i2c->device, (u8*)buf, sizeof(buf), oled_addr);
    usleep(i2c_write_delay);
}

// Clear the display.
static void ssd1306_clear(i2c_t *i2c) {
    int i;
    for (i = 0; i < (128*8); ++i) {
        i2c_data(&i2c->device, oled_addr, 0);
    }
}

// The watchdog timer driver context;
typedef struct {
    int id;
    int interrupt;
    int value;
    Xil_ExceptionHandler isr;
    XScuWdt_Config *config;
    XScuWdt device;
} wdt_t;

// Used to signal the watchdog has fired.
volatile int watchdog_terminate = 0;

// The watchdog timer isr.
static void wdt_isr(void *context)
{
    printf("wdt timeout\n");
    watchdog_terminate = 1;
}

// Sleep arbitrarily while petting the dog.
// Returns non-zero if the watchdog fired while delaying.
static int wdt_sleep_us(wdt_t *wdt, int us)
{
    static const int time_per_us = COUNTS_PER_SECOND/1000000;
    const int threshold = us * time_per_us;

    XTime start_time, current_time;
    XTime_GetTime(&start_time);

    while (!watchdog_terminate) {

        // Keep the watchdog from terminating the program.
        XScuWdt_LoadWdt(&wdt->device, wdt->value);

        // Continue to sample and wait until the requested time has elapsed.
        XTime_GetTime(&current_time);
        const int elapsed_time = current_time - start_time;
        if (elapsed_time >= threshold) {
            break;
        }
    }
    return watchdog_terminate;
}
static int wdt_sleep_ms(wdt_t *wdt, int ms)
{
    return wdt_sleep_us(wdt, ms*1000);
}
#if 0
static int wdt_sleep_s(wdt_t *wdt, int s)
{
    return wdt_sleep_us(wdt, s*1000000);
}
#endif

// Initialize the watchdog.
static int initialize_wdt(XScuGic *gic, wdt_t *wdt)
{
    wdt->config = XScuWdt_LookupConfig(wdt->id);
    if (!wdt->config) {
        printf("XScuWdt_LookupConfig failed\n");
        return XST_FAILURE;
    }
    int status = XScuWdt_CfgInitialize(&wdt->device, wdt->config,
            wdt->config->BaseAddr);
    if (status) {
        printf("XScuWdt_CfgInitialize failed %d\n", status);
        return status;
    }

    // Disable the watchdog before beginning
    status = XScuWdt_SelfTest(&wdt->device);
    if (status) {
        printf("XScuWdt_SelfTest failed %d\n", status);
        return status;
    }
    XScuWdt_SetTimerMode(&wdt->device);
    XScuWdt_LoadWdt(&wdt->device, wdt->value);

    // Configure the wdt interrupt.
    status = XScuGic_Connect(gic, wdt->interrupt, wdt->isr, wdt);
    if (status) {
        printf("XScuGic_Connect failed %d\n", status);
        return status;
    }

    printf("enable wdt interrupts for timer mode\n");
    int control = XScuWdt_GetControlReg(&wdt->device);
    control |= XSCUWDT_CONTROL_IT_ENABLE_MASK;
    XScuWdt_SetControlReg(&wdt->device, control);

    printf("enable wdt interrupt\n");
	XScuGic_Enable(gic, wdt->interrupt);
    return 0;
}

// Read a space delimited word from the specified file.
// If the file is exhausted before a word is found, wrap.
// If the buffer is exhausted, return.
static int read_word(FIL *fil, char *buffer, int buffer_size, int *word_size)
{
    *word_size = 0;

    char c;
    int i = 0, status;
    unsigned bytes_read;
    while (i < buffer_size) {

        // If the file is exhausted, seek to the beginning.
        if (f_eof(fil)) {
            status = f_lseek(fil, 0);
            if (status != FR_OK) {
                printf("f_lseek failed %d\n", status);
                return status;
            }
        }
        status = f_read(fil, &c, 1, &bytes_read);
        if (status || !bytes_read) {
            printf("f_read failed %d\n", status);
            return status;
        }

        // We only care about ascii.
        c &= 0x7f;

        // Map newline to strings, since we ignore control characters.
        if (c == '\n') {
            c = ' ';
        }
        if (iscntrl((int)c)) {
            continue;
        }
        if (isspace((int)c)) {
            if (i) {
                break;
            }
            continue;
        }
        buffer[i++] = c;
    }
    *word_size = i;
    return 0;
}

// The application entry point.
int main()
{
    init_platform();

    // Because init_platform *may* enable caches,
    // flush and disable the data cache after initialization.
    // The data cache *must* be disabled to use the mmc sdcard driver.
    Xil_DCacheFlush();
    Xil_DCacheDisable();

    printf("Hello world\n");

    int status;

    // Verify queue behavior.
    status = queue_test();
    if (status) {
        printf("queue_test failed %d\n", status);
        return status;
    }

    i2c_t oled = {
        .id = XPAR_PS7_I2C_0_DEVICE_ID,
        .clock = 100000,
        .clear_options = XIICPS_10_BIT_ADDR_OPTION,
        .options = XIICPS_7_BIT_ADDR_OPTION,
    };
    status = initialize_i2c(&oled);
    if (status) {
        printf("initialize_i2c failed %d\n", status);
        return status;
    }
    XGpioPs gpio;
    status = initialize_gpio(&gpio);
    if (status) {
        printf("intialize_gpio failed %d\n", status);
        return status;
    }
    ssd1306_reset(&oled, &gpio, oled_reset_pin);
    ssd1306_clear(&oled);

    XScuGic gic;
    status = initialize_gic(&gic);
    if (status) {
        printf("initialize_gic failed %d\n", status);
        return status;
    }

    ttc_t timers[] = {
        {
            .id = XPAR_PS7_TTC_0_DEVICE_ID,
            .interrupt = XPS_TTC0_0_INT_ID,
            .isr = ttc0_isr,
            .frequency_hz = 1,
            .options = XTTCPS_OPTION_INTERVAL_MODE|XTTCPS_OPTION_WAVE_DISABLE,
        },
        // Timer1, which toggles led mio7 at 5Hz.
        {
            .id = XPAR_PS7_TTC_1_DEVICE_ID,
            .interrupt = XPS_TTC0_1_INT_ID,
            .isr = ttc1_isr,
            .isr_context = &gpio,
            .frequency_hz = 5,
            .options = XTTCPS_OPTION_INTERVAL_MODE|XTTCPS_OPTION_WAVE_DISABLE,
        },
    };

    int i;
    for (i = 0; i < sizeof(timers)/sizeof(*timers); ++i) {
        status = initialize_ttc(&gic, &timers[i]);
        if (status) {
            printf("initialize_ttc failed %d\n", status);
            return status;
        }
    }

    XGpio buttons;
    status = initialize_axi_gpio(&gic, &buttons);
    if (status) {
        printf("initialize_axi_gpio failed %d\n", status);
        return status;
    }

    // Mount the sdcard, and open the data file for buffering.
    printf("mounting sdcard\n");
    FATFS fatfs;
    status = f_mount(0, &fatfs);
    if (status != FR_OK) {
        printf("f_mount failed %d\n", status);
        return status;
    }
    const char *file_path = "test.txt";
    printf("opening %s\n", file_path);
    FIL fil;
    status = f_open(&fil, file_path, FA_READ);
    if (status != FR_OK) {
        printf("f_open failed %d\n", status);
        return status;
    }

    // Configure a ~100ms watchdog timer given the current system.
    wdt_t wdt = {
        .id = XPAR_PS7_SCUWDT_0_DEVICE_ID,
        .interrupt = XPS_SCU_WDT_INT_ID,
        .value = COUNTS_PER_SECOND,
        .isr = wdt_isr,
    };
    status = initialize_wdt(&gic, &wdt);
    if (status) {
        printf("initialize_wdt failed %d\n", status);
        return status;
    }

    // The scheduler runqueue.
    // XXX do we need to synchronize access (disable interrupts) to the queue?
    queue_t runq;
    queue_init(&runq);
    schedulerq = &runq;

    printf("press the center button to exit\n");
    enable_gic_interrupts(&gic);

    printf("starting wdt\n");
    watchdog_terminate = 0;
    XScuWdt_LoadWdt(&wdt.device, wdt.value);

    // Verify the watchdog is working. Wait for the watchdog to fire before
    // restarting it and running in earnest.
    while (!watchdog_terminate) {
        usleep(1000 * 25);
        status = XScuWdt_ReadReg(wdt.config->BaseAddr,
                XSCUWDT_COUNTER_OFFSET);
        printf("counter %u\n", status);
    }

    // Reset the watchdog. The main loop will reset the counter value to
    // prevent the watchdog from firing.
    watchdog_terminate = 0;
    XScuWdt_LoadWdt(&wdt.device, wdt.value);

    // Run until told to die.
    int scroll = 0;
    int word_size = 0;
    char word_buffer[32];
    while (!terminate && !watchdog_terminate) {
        wdt_sleep_ms(&wdt, display_update_delay_ms);

        // If we've exhausted the word buffer, read a new word.
        // Note: A pathological file that contains only spaces will trip the
        // watchdog.

        // Make sure there's enough room for a space at the end.
        status = read_word(&fil, word_buffer, sizeof(word_buffer)-1,
                &word_size);
        if (status) {
            printf("read_word failed %d\n", status);
            return status;
        }

        // Add a space, and display the word.
        word_buffer[word_size++] = ' ';
        display_word(&oled, oled_addr, word_buffer, word_size);

        // Scroll a pixel line at a time.
        ssd1306_set_display_start_line(&oled, 64-scroll);
        scroll += scroll_update;
        if (scroll > 64) {
            scroll = 0;
        }

        // If there something in the queue, grab it.
        if (!queue_isempty(schedulerq)) {
            int task;
            status = queue_dequeue(schedulerq, (void**)&task);
            if (status) {
                printf("queue_dequeue failed %d\n", status);
                return status;
            }
        }
    }

    // blank the display on exit.
    XScuWdt_Stop(&wdt.device);
    ssd1306_clear(&oled);
    return 0;
}
