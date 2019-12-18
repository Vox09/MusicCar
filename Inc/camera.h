#ifndef __CAMERA_H
#define __CAMERA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_ov7725.h"
#include "bsp_sccb.h"
#include "lcd.h"
#include "cmsis_os.h"

extern uint8_t Ov7725_vsync;
extern uint8_t take_flag;
	
#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_H */


