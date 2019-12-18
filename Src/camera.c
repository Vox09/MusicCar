#include "camera.h"
#include "sd_card.h"

void StartCameraTask(void const * argument)
{	
	// LCD_Clear (50, 80, 140, 70, RED);
	// LCD_DrawString(75, 100, "CAMERA DEMO");
	// osDelay(2000);

	// while(Ov7725_Init() != SUCCESS);
	// Ov7725_vsync = 0;
	// while(Ov7725_vsync != 0) osDelay(100);
  	while (1)
  	{
		if (Ov7725_vsync == 2)
		{
			FIFO_PREPARE;
			if(take_flag==1) 
			{
				take_flag = 0;
				TakePicture();
			}
			if(take_flag==2) 
			{
				take_flag = 0;
				ImagDisp();
			}			
			Ov7725_vsync = 0;
		}
		osDelay(50);
  	}
}	
void startDisplay(void)
{ 	take_flag = 2;}