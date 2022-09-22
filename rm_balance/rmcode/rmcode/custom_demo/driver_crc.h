#ifndef __DRIVER_CRC_H
#define __DRIVER_CRC_H

#include "stm32f4xx.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"

uint8_t getCrc8CheckSum(uint8_t *pchMessage, uint32_t dwLength, uint8_t crc);
uint8_t verifyCrc8CheckSum(uint8_t *pchMessage, uint32_t dwLength);
void appendCrc8CheckSum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t getCrc16CheckSum(uint8_t *pchMessage,uint32_t dwLength,uint16_t crc);
uint8_t verifyCrc16CheckSum(uint8_t *pchMessage, uint32_t dwLength);
void appendCrc16CheckSum(uint8_t *pchMessage,uint32_t dwLength);
uint32_t getCrc32CheckSum(uint8_t *pchMessage, uint32_t dwLength, uint32_t crc);
uint8_t verifyCrc32CheckSum(uint8_t *pchMessage, uint32_t dwLength);
void appendCrc32CheckSum(uint8_t *pchMessage, uint32_t dwLength);

#endif
