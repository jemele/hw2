// Joshua Emele <jemele@acm.org>
#include <stdio.h>
#include "platform.h"
#include <xscugic.h>
#include <xil_exception.h>
#include <xgpio.h>
#include <xgpiops.h>
#include <sleep.h>

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
    XGpioPs gpio;
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
    
    // Configure individual interrupt sources.
    // Begin by configuring the buttons interrupt.
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
