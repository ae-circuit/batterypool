#ifndef DHT22_H
#define DHT22_H

#include "stm32f4xx.h"

// Function prototypes
void DHT22_Init(void);
uint8_t DHT22_ReadTemperature(void);

#endif /* DHT22_H */