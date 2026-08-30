#ifndef _BSP_W25Q64_H
#define _BSP_W25Q64_H

#include <string.h>
#include <stdlib.h>
#include "stdio.h"	
#include "main.h"
#define SPI_FLASH_PageSize      256
#define SPI_FLASH_PerWritePageSize      256
#define W25QXX_CS_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define W25QXX_CS_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define W25X_WriteEnable		      0x06 
#define W25X_WriteDisable		      0x04 
#define W25X_ReadStatusReg		    0x05 
#define W25X_WriteStatusReg		    0x01 
#define W25X_ReadData			        0x03 
#define W25X_FastReadData		      0x0B 
#define W25X_FastReadDual		      0x3B 
#define W25X_PageProgram		      0x02 
#define W25X_BlockErase			      0xD8 
#define W25X_SectorErase		      0x20 
#define W25X_ChipErase			      0xC7 
#define W25X_PowerDown			      0xB9 
#define W25X_ReleasePowerDown	    0xAB 
#define W25X_DeviceID			        0xAB 
#define W25X_ManufactDeviceID   	0x90 
#define W25X_JedecDeviceID		    0x9F 
#define Dummy_Byte                0xFF
extern SPI_HandleTypeDef hspi1;
uint32_t W25QXX_ReadJedecID(void);
void W25QXX_SectorErase(uint32_t SectorAddr);
void W25QXX_BulkErase(void);
void W25QXX_PageWrite(uint8_t* pTxBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void W25QXX_BufferRead(uint8_t* pRxBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);

/* 崩溃黑匣子：把日志体轮询写入 W25Q64 保留扇区（不依赖 RTOS/DMA，可在 HardFault 调用） */
int w25q_crashlog_save(const uint8_t *data, uint32_t len);

/* 上电自检：JEDEC 校验 + 擦/写/读回环，验证 40MHz SCK 稳定性。返回 0=通过，<0=失败 */
int W25QXX_Test(void);
#endif


