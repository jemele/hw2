// Joshua Emele <jemele@acm.org
// Derived from FSBL application.

#include <stdio.h>
#include "platform.h"
#include "xil_cache.h"
#include "xil_exception.h"
#include "xstatus.h"
#include "xtime_l.h"
#include "xwdtps.h"
#include "ff.h"

u32 FlashReadBaseAddress = XPAR_PS7_SD_0_S_AXI_BASEADDR;

static FATFS fatfs;
static FIL fil;

char *strcpy_rom(char *Dest, const char *Src)
{
   unsigned i;
   for (i=0; Src[i] != '\0'; ++i)
           Dest[i] = Src[i];
   Dest[i] = '\0';
   return Dest;
}

int main()
{
    Xil_DCacheFlush();
    Xil_DCacheDisable();

    init_platform();

    printf("Hello world\n");
	
    printf("mounting sdcard\n");
    int status = f_mount(0, &fatfs);
    if (status != FR_OK) {
        printf("f_mount failed %d\n", status);
        return status;
    }
	
    const char *file_path = "test.txt";
    printf("opening %s\n", file_path);
    status = f_open(&fil, file_path, FA_READ);
    if (status != FR_OK) {
        printf("f_open failed %d\n", status);
        return status;
    }

    printf("seeking to beginning\n");
    status = f_lseek(&fil, 0);
    if (status != FR_OK) {
        printf("f_lseek failed %d\n", status);
        return status;
    }
    printf("eof %d\n", f_eof(&fil));

    char buffer;
    unsigned bytes_read;
    while (!f_eof(&fil)) {
        status = f_read(&fil, &buffer, 1, &bytes_read);
        if (status != FR_OK) {
            printf("f_read failed %d\n", status);
            return status;
        }
        printf("read %d %c\n", bytes_read, buffer);
    }

    printf("goodbye\n");
    return 0;
}

