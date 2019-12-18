#ifndef SD_CARD_H
#define SD_CARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "fatfs.h"

void startTakePicture(void);
void startDisplay(void);
void TakePicture(void);

#ifdef __cplusplus
}
#endif

#endif /* __SD_CARD_H */
