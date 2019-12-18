#include "sd_card.h"
#include "lcd.h"
#include "bmp.h"
#include "camera.h"
#include "cmsis_os.h"


FATFS myFATFS;
FIL myFILE;

UINT numberofbytes;
char myPath[20];
FRESULT res;
uint8_t cnt=0;
osThreadId captureTaskHandle;
uint8_t take_flag = 0;


void StartSDCardTask(void const * argument)
{
   	MX_FATFS_Init();

    osDelay(1000);

  	res = f_mount(&myFATFS,SDPath,1); // Important!
	if (res == FR_OK)
	{
    	LCD_DrawString(10, 10, "SD Card mount successfully!\0");
 	}	
	else
	{
    	LCD_DrawString(10, 10, "SD card mount error!\0");
	}
	osThreadTerminate(osThreadGetId());
}

void TakePicture(){
	
	if (cnt > 4)
	{
		LCD_DrawString(10, 100,"Stop!!You already take 5 photos!That's enough!");
		return;
	}
	char cntcc = 'A'+cnt;
	sprintf(myPath, "Picture%c.bmp\0", cntcc);
	cnt += 1;
	f_open(&myFILE, myPath, FA_WRITE |FA_CREATE_ALWAYS);



	BITMAPINF bmp;
	uint16_t color;

	bmp.bmfHeader.bfType = 0x4D42;			//bmp类型
	bmp.bmfHeader.bfOffBits=sizeof(bmp.bmfHeader)+sizeof(bmp.bmiHeader)+sizeof(bmp.RGB_MASK);			//位图信息结构体所占的字节数
	bmp.bmfHeader.bfSize= bmp.bmfHeader.bfOffBits + 320*240*2;	//文件大小（信息结构体+像素数据）
	bmp.bmfHeader.bfReserved1 = 0x0000;		//保留，必须为0
	bmp.bmfHeader.bfReserved2 = 0x0000;  			
			
	bmp.bmiHeader.biSize=sizeof(bmp.bmiHeader);  		//位图信息头的大小
	bmp.bmiHeader.biWidth=320;  		//位图的宽度
	bmp.bmiHeader.biHeight=240;  			    //图像的高度
	bmp.bmiHeader.biPlanes=1;  		//目标设别的级别，必须是1
	bmp.bmiHeader.biBitCount=16;            //每像素位数
	bmp.bmiHeader.biCompression=3;  	       //RGB555格式
	bmp.bmiHeader.biSizeImage=320*240*2;  //实际位图所占用的字节数（仅考虑位图像素数据）
	bmp.bmiHeader.biXPelsPerMeter=0;		    //水平分辨率
	bmp.bmiHeader.biYPelsPerMeter=0; 		    //垂直分辨率
	bmp.bmiHeader.biClrImportant=0;   	  //说明图像显示有重要影响的颜色索引数目，0代表所有的颜色一样重要
	bmp.bmiHeader.biClrUsed=0;  			    //位图实际使用的彩色表中的颜色索引数，0表示使用所有的调色板项
			
	bmp.RGB_MASK[0]=0X00F800;
	bmp.RGB_MASK[1]=0X0007E0;
	bmp.RGB_MASK[2]=0X00001F;
			
	res=f_write(&myFILE, &bmp, sizeof(bmp), &numberofbytes);
			
	for(int i=0;i<240;i++)
	{
		for(int j=0;j<320;j++)
		{
			READ_FIFO_PIXEL(color);
				
			f_write(&myFILE, &color, sizeof(color), &numberofbytes);
		}
	}
	f_close(&myFILE);
	char cntc='0'+ cnt;
	LCD_DrawString(10, 100, "Wow!It's the");
	LCD_DrawChar(130, 100, cntc);
	LCD_DrawString(150, 100, "time you're taking a picture!\0");
	
	//osThreadTerminate(osThreadGetId());
}

void startTakePicture(){take_flag = 1;}



