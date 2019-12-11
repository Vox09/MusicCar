#include "sd_card.h"
#include "lcd.h"

void StartSDCardTask(void const * argument)
{
   	MX_FATFS_Init();

    osDelay(2000);
	FATFS myFATFS;
	FIL myFILE;
	UINT numberofbytes;
	char myPath[] = "TEST.TXT\0";
	char myData[] = "Hello World\0";
	FRESULT res;

  	res = f_mount(&myFATFS,SDPath,1);
	if (res == FR_OK)
	{
    	LCD_DrawString(10, 10, "SD card mount successfully\0");
		f_open(&myFILE, myPath, FA_WRITE |FA_CREATE_ALWAYS);
		f_write(&myFILE, myData, sizeof(myData), &numberofbytes);
		f_close(&myFILE);
 	}	
	else
	{
    	LCD_DrawString(10, 10, "SD card mount error!\0");
	}

	for(;;)
	{
		f_write(&myFILE, myData, sizeof(myData), &numberofbytes);
		osDelay(10);
	}
}