// Joshua Emele <jemele@acm.org>
#include <stdio.h>
#include "platform.h"
#include <xscugic.h>
#include <xil_exception.h>
#include <xttcps.h>
#include <xgpiops.h>
#include <sleep.h>

// http://forums.xilinx.com/t5/Xcell-Daily-Blog/Introduction-to-the-Zynq-Triple-Timer-Counter-Part-Three-Adam/ba-p/413105
// A struct to contain all relevant ttc options.
typedef struct
{
    u32 hz;
    u16 interval;
    u8 prescalar;
    u16 options;

} ttc_config_t;


static XGpioPs gpio;

// The ttc0 interrupt service routine.
static void ttc0_isr(void *context)
{
    if (!context) {
        return;
    }

    XTtcPs *ttc0 = (XTtcPs*)context;
    int status = XTtcPs_GetInterruptStatus(ttc0);
    printf("ttc0 %d\n", status);
    XTtcPs_ClearInterruptStatus(ttc0, status);

    const u8 pin = XGpioPs_ReadPin(&gpio, 7);
    XGpioPs_WritePin(&gpio, 7, !pin);
}

// The application entry point.
int main()
{
    init_platform();

    printf("Hello World\n\r");

    // Configure gpio and pull the oled out of reset.
    XGpioPs_Config *gpio_config =
        XGpioPs_LookupConfig(XPAR_PS7_GPIO_0_DEVICE_ID);
    if (!gpio_config) {
        printf("XGpioPs_LookupConfig failed\n");
        return XST_FAILURE;
    }
    usleep(25);

    printf("gpio initialization\n");
    int status = XGpioPs_CfgInitialize(&gpio, gpio_config, gpio_config->BaseAddr);
    if (status != XST_SUCCESS) {
        printf("XGpioPs_CfgInitialize failed %d\n", status);
        return status;
    }
    usleep(25);

    printf("gpio selftest\n");
    status = XGpioPs_SelfTest(&gpio);
    if (status != XST_SUCCESS) {
        printf("XGpioPs_SelfTest %d\n", status);
        return status;
    }
    usleep(25);

    // Turn on mio7 led - we'll toggle this with the timer interrupt.
    XGpioPs_SetDirectionPin(&gpio, 7, 1); 
    XGpioPs_SetOutputEnablePin(&gpio, 7, 1);
    XGpioPs_WritePin(&gpio, 7, 1);
 
    // Initialize ttc.
    XTtcPs_Config *ttc0_config = XTtcPs_LookupConfig(XPAR_PS7_TTC_0_DEVICE_ID);
    if (!ttc0_config) {
        printf("XTtcPs_LookupConfig failed\n");
        return XST_FAILURE;
    }    

    printf("stopping ttc0\n");
    XTtcPs ttc0;
    XTtcPs_Stop(&ttc0);

    printf("initializing ttc0\n");
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

    // Configure the global interrupt controller.
    printf("initializating exceptions\n");
    Xil_ExceptionInit();

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
        
    printf("connect gic interrupt handler\n");
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
        (Xil_ExceptionHandler)XScuGic_InterruptHandler, &gic);

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

    printf("enabling intterupts\n");
    Xil_ExceptionEnableMask(XIL_EXCEPTION_IRQ);

    while (1) {}

    return 0;
}
