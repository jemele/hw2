// Joshua Emele <jemele@acm.org>
#include <stdio.h>
#include "platform.h"
#include <xscugic.h>
#include <xil_exception.h>
#include <xttcps.h>
#include <xgpio.h>
#include <xgpiops.h>
#include <sleep.h>

// http://forums.xilinx.com/t5/Xcell-Daily-Blog/Introduction-to-the-Zynq-Triple-Timer-Counter-Part-Three-Adam/ba-p/413105
// A struct to contain all relevant ttc options.
typedef struct {
    int id;
    int interrupt;
    Xil_ExceptionHandler isr;
    u32 frequency_hz;
    u16 options;

    XTtcPs_Config *config;
    XTtcPs device;
    u16 interval;
    u8 prescalar;

} ttc_t;

// The ttc0 interrupt service routine.
// For now, toggle MIO7.
static void ttc0_isr(void *context)
{
    printf("ttc0 isr\n");
    if (!context) {
        return;
    }
    ttc_t *ttc = (ttc_t*)context;
    const int status = XTtcPs_GetInterruptStatus(&ttc->device);
    XTtcPs_ClearInterruptStatus(&ttc->device, status);
}

// The buttons interrupt service routine.
static void buttons_isr(void *context)
{
    if (!context) {
        return;
    }
    XGpio *buttons = (XGpio*)context;
    XGpio_InterruptDisable(buttons, XGPIO_IR_CH1_MASK);
    XGpio_InterruptClear(buttons, XGPIO_IR_CH1_MASK);
    XGpio_InterruptEnable(buttons, XGPIO_IR_CH1_MASK);
    printf("buttons\n");
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
static int initialize_ttc0(XScuGic *gic, ttc_t *ttc)
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
    //! @todo Review trigger type.
    printf("setting button trigger type\n");
    XScuGic_SetPriorityTriggerType(gic, XPAR_FABRIC_GPIO_0_VEC_ID,
        0xa0, 0x3);

    printf("connect buttons interrupt handler\n");
    status = XScuGic_Connect(gic, XPAR_FABRIC_GPIO_0_VEC_ID,
        (Xil_ExceptionHandler)buttons_isr, buttons);
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
    ttc_t ttc0 = {
        .id = XPAR_PS7_TTC_0_DEVICE_ID,
        .interrupt = XPS_TTC0_0_INT_ID,
        .isr = ttc0_isr,
        .frequency_hz = 1,
        .options = XTTCPS_OPTION_INTERVAL_MODE|XTTCPS_OPTION_WAVE_DISABLE,
    };
    status = initialize_ttc0(&gic, &ttc0);
    if (status) {
        printf("initialize_ttc0 failed %d\n", status);
        return status;
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
