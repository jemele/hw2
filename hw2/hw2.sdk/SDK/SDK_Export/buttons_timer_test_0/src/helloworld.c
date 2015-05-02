// Joshua Emele <jemele@acm.org>
#include <stdio.h>
#include "platform.h"
#include <xscugic.h>
#include <xiicps.h>
#include <xil_exception.h>
#include <xttcps.h>
#include <xgpio.h>
#include <xgpiops.h>
#include <sleep.h>
#include "font_5x7.h"

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
    char start;
    int width;
    int pad;
} font_t;

// Tailor the font to the display.
const font_t font = {
    .base = Font5x7,
    .start = ' ',
    .width = 5,
    .pad = 128%5
};

// Display a character on the device.
// Map the character onto the font index, and then write the font data out.
void display_character(XIicPs *device, u8 addr, char c, const font_t * f)
{
    const int index = (f->width * (c - f->start));
    int i;
    for (i = 0; i < f->width; ++i) {
        i2c_data(device, addr, f->base[index+i]);
    }
    for (i = 0; i < f->pad; ++i) {
        i2c_data(device, addr, 0);
    }
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
    case button_up:
        printf("up\n");
        break;
    case button_right:
        printf("right\n");
        break;
    case button_left:
        printf("left\n");
        break;
    case button_down:
        printf("down\n");
        break;
    case button_center:
        printf("center\n");
        terminate = 1;
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

// Clear the display.
static void ssd1306_clear(i2c_t *i2c) {
    int i;
    for (i = 0; i < (128*8); ++i) {
        i2c_data(&i2c->device, oled_addr, 0);
    }
}

// The application entry point.
int main()
{
    init_platform();

    printf("Hello world\n");

    int status;

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

    printf("character display test\n");
    char c;
    for (c = ' '; c <= '}'; ++c) {
        display_character(&oled.device, oled_addr, c, &font);
        usleep(2500);
    }

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
    enable_gic_interrupts(&gic);

    // Run until told to die.
    printf("press the center button to exit\n");
    while (!terminate) {}
    return 0;
}
