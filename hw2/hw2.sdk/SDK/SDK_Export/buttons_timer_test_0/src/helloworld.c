// Joshua Emele <jemele@acm.org>
#include <stdio.h>
#include "platform.h"
#include <xscugic.h>
#include <xil_exception.h>
#include <xttcps.h>
#include <xgpio.h>
#include <xgpiops.h>
#include <sleep.h>

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

    printf("ttc calculating interval\n");
    XTtcPs_CalcIntervalFromFreq(&ttc->device, ttc->frequency_hz,
        &ttc->interval, &ttc->prescalar);

    printf("ttc setting interval\n");
    XTtcPs_SetInterval(&ttc->device, ttc->interval);

    printf("ttc setting prescalar\n");
    XTtcPs_SetPrescaler(&ttc->device, ttc->prescalar);

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

// The application entry point.
int main()
{
    init_platform();

    printf("Hello world\n");

    XGpioPs gpio;
    int status = initialize_gpio(&gpio);
    if (status) {
        printf("intialize_gpio failed %d\n", status);
        return status;
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

    // Run forever.
    while (1) {}
    return 0;
}
