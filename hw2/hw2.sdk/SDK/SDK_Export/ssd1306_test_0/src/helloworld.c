// Joshua Emele <jemele@acm.org>
#include <stdio.h>
#include "platform.h"
#include <xparameters.h>
#include <xil_cache.h>
#include <xiicps.h>
#include <sleep.h>
#include <xgpiops.h>
#include "font_5x7.h"
#include "Inspire.h"
#include "clean128x64.h"

// https://github.com/Defragster/ssd1306xled/blob/master/ssd1306xled.cpp
const u8 ssd1306_init_sequence [] = {   // Initialization Sequence
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

static const u8 oled_addr = 0x3c;
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

// Display a character on the device.
// Map the character onto the font index, and then write the font data out.
void display_character(XIicPs *device, u8 addr, char c)
{
    const int index = (5 * ((int)c - ' '));

    int i;
    for (i = 0; i < 5; ++i) {
        i2c_data(device, addr, Font5x7[index+i]);
    }
}

// The application entry point.
int main()
{
    // Flush and disable caches.
    // This is needed to support sdcard operations.
    Xil_DCacheFlush();
    Xil_DCacheDisable();

    init_platform();

    printf("Hello World\n\r");

    // Find the i2c0 configuration and initialize the ssd1306 oled part.
    XIicPs_Config *i2c0_config = XIicPs_LookupConfig(XPAR_PS7_I2C_0_DEVICE_ID);
    if (!i2c0_config) {
        printf("XIicPs_LookupConfig failed\n");
        return XST_FAILURE;
    }

    printf("i2c initialization\n");
    XIicPs oled;
    int status = XIicPs_CfgInitialize(&oled, i2c0_config, i2c0_config->BaseAddress);
    if (status != XST_SUCCESS) {
        printf("XIicPs_CfgInitialize failed %d\n", status);
        return status;
    }
    usleep(25);

    status = XIicPs_SelfTest(&oled);
    if (status != XST_SUCCESS) {
        printf("XIicPs_SelfTest failed %d\n", status);
        return status;
    }
    usleep(25);

    status = XIicPs_SetSClk(&oled, 100000);
    if (status != XST_SUCCESS) {
        printf("XIicPs_SetSClk failed %d\n", status);
        return status;
    }
    usleep(25);

    status = XIicPs_ClearOptions(&oled, XIICPS_10_BIT_ADDR_OPTION);
    if (status != XST_SUCCESS) {
        printf("XIicPs_ClearOptions failed %d\n", status);
        return status;
    }
    usleep(25);

    status = XIicPs_SetOptions(&oled, XIICPS_7_BIT_ADDR_OPTION);
    if (status != XST_SUCCESS) {
        printf("XIicPs_SetOptions failed %d\n", status);
        return status;
    }
    usleep(25);
 

    // Configure gpio and pull the oled out of reset.
    XGpioPs_Config *gpio_config =
        XGpioPs_LookupConfig(XPAR_PS7_GPIO_0_DEVICE_ID);
    if (!gpio_config) {
        printf("XGpioPs_LookupConfig failed\n");
        return XST_FAILURE;
    }
    usleep(25);

    printf("gpio initialization\n");
    XGpioPs gpio;
    status = XGpioPs_CfgInitialize(&gpio, gpio_config, gpio_config->BaseAddr);
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

    printf("oled reset\n");
    XGpioPs_SetDirectionPin(&gpio, 12, 1); 
    XGpioPs_SetOutputEnablePin(&gpio, 12, 1);
    XGpioPs_WritePin(&gpio, 12, 0);
    usleep(300);
    XGpioPs_WritePin(&gpio, 12, 1);
    usleep(300);

    // Write out the oled initialization sequence.
    printf("oled initialization sequence\n");
    int i;
    for (i = 0; i < sizeof(ssd1306_init_sequence)/sizeof(*ssd1306_init_sequence); ++i) {
       i2c_command(&oled, oled_addr, ssd1306_init_sequence[i]);
    }

    // Display the inspire logo, then give some time to admire.
    printf("flashing logo\n");
    for (i = 0; i < sizeof(Inspire)/sizeof(*Inspire); ++i) {
        i2c_data(&oled, oled_addr, Inspire[i]);
    }
    sleep(5);

    // Blank the display, and then test character display.
    printf("blanking display\n");
    for (i = 0; i < sizeof(Inspire)/sizeof(*Inspire); ++i) {
        i2c_data(&oled, oled_addr, 0);
    }

    printf("character display test\n");
    char c;
    for (c = ' '; c <= '}'; ++c) {
        display_character(&oled, oled_addr, c);
        usleep(2500);
    }

    return 0;
}
