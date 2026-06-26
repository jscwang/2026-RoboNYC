#ifndef COMPASS_H
#define COMPASS_H
#include "main.h"

void updateCamera();
void bno_write(uint8_t i2c_addr, uint8_t reg, uint8_t data);
void setupCompass();

#endif
