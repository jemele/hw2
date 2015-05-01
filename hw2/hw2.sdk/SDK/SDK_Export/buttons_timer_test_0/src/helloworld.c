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
    u32 hz;
    u16 interval;
    u8 prescalar;
    u16 options;
} ttc_config_t;

// A global gpio context, for use in interrupts.
// @todo This may not be wise ... I need to figure out if this was the source
// of the undefined behavior observed earlier...
static XGpioPs gpio;

// The ttc0 interrupt service routine.
// For now, toggle MIO7.
static void ttc0_isr(void *context)
{
    printf("ttc0 isr\n");
    if (!context) {
        return;
    }
    XTtcPs *ttc0 = (XTtcPs*)context;
    int status = XTtcPs_GetInterruptStatus(ttc0);
    XTtcPs_ClearInterruptStatus(ttc0, status);
#if 0
    const u8 pin = XGpioPs_ReadPin(&gpio, 7);
    XGpioPs_WritePin(&gpio, 7, !pin);
#endif
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

// The application entry point.
int main()
{
    init_platform();

    printf("Hello world\n");

    // Initialize the gpio subsystem.
    // We'll use this to toggle MIO7.
    XGpioPs_Config *gpio_config =
        XGpioPs_LookupConfig(XPAR_PS7_GPIO_0_DEVICE_ID);
    if (!gpio_config) {
        printf("XGpioPs_LookupConfig failed\n");
        return XST_FAILURE;
    }

    printf("gpio initialization\n");
    int status = XGpioPs_CfgInitialize(&gpio, gpio_config, gpio_config->BaseAddr);
    if (status != XST_SUCCESS) {
        printf("XGpioPs_CfgInitialize failed %d\n", status);
        return status;
    }

    printf("gpio selftest\n");
    status = XGpioPs_SelfTest(&gpio);
    if (status != XST_SUCCESS) {
        printf("XGpioPs_SelfTest %d\n", status);
        return status;
    }

    printf("lighting up mio7\n");
    XGpioPs_SetDirectionPin(&gpio, 7, 1); 
    XGpioPs_SetOutputEnablePin(&gpio, 7, 1);
    XGpioPs_WritePin(&gpio, 7, 1);

    // Initialize the global interrupt controller.
    // We'll need this to configure interrupt sources.
    XScuGic_Config *gic_config = XScuGic_LookupConfig(XPAR_PS7_SCUGIC_0_DEVICE_ID);
    if (!gic_config) {
        printf("XScuGic_LookupConfig failed\n");
        return XST_FAILURE;
    }

    printf("initializing gic\n"); 
    XScuGic gic;
    status = XScuGic_CfgInitialize(&gic, gic_config,
        gic_config->CpuBaseAddress);
    if (status != XST_SUCCESS) {
        printf("XScuGic_CfgInitialize failed %d\n", status);
        return status;
    }

    // Initialize ttc0.
    XTtcPs_Config *ttc0_config = XTtcPs_LookupConfig(XPAR_PS7_TTC_0_DEVICE_ID);
    if (!ttc0_config) {
        printf("XTtcPs_LookupConfig failed\n");
        return XST_FAILURE;
    }    

    // https://github.com/joshuaspence/ThesisCode/blob/master/TopN_Outlier_Pruning_Block/FPGA/SDK/demo_threadx/src/demo_threadx.c
    printf("stopping ttc0\n");
    XTtcPs_WriteReg(ttc0_config->BaseAddress, XTTCPS_CNT_CNTRL_OFFSET, 0x33); 
    XTtcPs_ReadReg(ttc0_config->BaseAddress, XTTCPS_ISR_OFFSET);

    printf("initializing ttc0\n");
    XTtcPs ttc0;
    status = XTtcPs_CfgInitialize(&ttc0, ttc0_config, ttc0_config->BaseAddress);
    if (status != XST_SUCCESS) {
        printf("XTtcPs_CfgInitialize failed %d\n", status);
        return status;
    }

    ttc_config_t ttc0_setup = {
        1, // Hz
        0,
        0,
        XTTCPS_OPTION_INTERVAL_MODE|XTTCPS_OPTION_WAVE_DISABLE
    };
    printf("ttc0 setting options\n");
    status = XTtcPs_SetOptions(&ttc0, ttc0_setup.options);
    if (status != XST_SUCCESS) {
        printf("XTtcPs_SetOptions failed %d\n", status);
        return status;
    }

    printf("ttc0 calculating interval\n");
    XTtcPs_CalcIntervalFromFreq(&ttc0, ttc0_setup.hz,
        &ttc0_setup.interval, &ttc0_setup.prescalar);

    printf("ttc0 setting interval\n");
    XTtcPs_SetInterval(&ttc0, ttc0_setup.interval);

    printf("ttc0 setting prescalar\n");
    XTtcPs_SetPrescaler(&ttc0, ttc0_setup.prescalar);

    // Configure individual interrupt sources.
    // Begin by initializing the ttc0 interrupt.
    printf("connect ttc0 interrupt handler\n");
    status = XScuGic_Connect(&gic, XPAR_XTTCPS_0_INTR,
        (Xil_ExceptionHandler)ttc0_isr, (void*)&ttc0);
    if (status != XST_SUCCESS) {
        printf("XScuGic_Connect failed %d\n", status);
        return status;
    }

    printf("enabling ttc0 interrupt\n");
    XScuGic_Enable(&gic, XPAR_XTTCPS_0_INTR);

    printf("enabling ttc0 interval mask\n");
    XTtcPs_EnableInterrupts(&ttc0, XTTCPS_IXR_INTERVAL_MASK);

    printf("starting ttc0\n");
    XTtcPs_Start(&ttc0);

    // Configure the buttons interrupt.
    printf("initializing axi gpio for buttons\n");
    XGpio buttons;
    status = XGpio_Initialize(&buttons, XPAR_AXI_GPIO_0_DEVICE_ID);
    if (status != XST_SUCCESS) {
        printf("XGpio_Initialize failed %d\n", status);
        return status;
    }
    printf("setting button trigger type\n");
    XScuGic_SetPriorityTriggerType(&gic, XPAR_FABRIC_GPIO_0_VEC_ID,
        0xa0, 0x3);

    printf("connect buttons interrupt handler\n");
    status = XScuGic_Connect(&gic, XPAR_FABRIC_GPIO_0_VEC_ID,
        (Xil_ExceptionHandler)buttons_isr, &buttons);
    if (status != XST_SUCCESS) {
        printf("XScuGic_Connect failed %d\n", status);
        return status;
    }
    
    printf("enabling buttons intterupt\n");
    XScuGic_Enable(&gic, XPAR_FABRIC_GPIO_0_VEC_ID);
    
    printf("enable channel 1 interrupts\n");
    XGpio_InterruptEnable(&buttons, XGPIO_IR_CH1_MASK);
    XGpio_InterruptGlobalEnable(&buttons);

    // Now that the individual interrupt sources are connected and configured,
    // enable gic interrupts.
    printf("initializating exceptions\n");
    Xil_ExceptionInit();

    printf("connect gic interrupt handler\n");
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
        (Xil_ExceptionHandler)XScuGic_InterruptHandler, &gic);

    printf("enabling intterupts\n");
    Xil_ExceptionEnable();

    while (1) {}
    return 0;
}
