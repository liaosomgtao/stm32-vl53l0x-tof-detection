/**
  ******************************************************************************
  * @file    vl53l0x.h
  * @brief   VL53L0X TOF 激光测距传感器驱动头文件（STM32 HAL，I2C）
  ******************************************************************************
  */
#ifndef __VL53L0X_H
#define __VL53L0X_H

#include "stm32f1xx_hal.h"

/* VL53L0X 器件地址：7 位地址 0x29，HAL 使用 8 位地址需左移 1 位 */
#define VL53L0X_ADDR                (0x29 << 1)   /* 0x52 */

/* 常用寄存器地址（见 VL53L0X 数据手册） */
#define VL53L0X_REG_SYSRANGE_START            0x00   /* 写 1 开始单次测距 */
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR    0x0B   /* 写 1 清除中断 */
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS   0x13   /* bit0=新数据就绪 */
#define VL53L0X_REG_RESULT_RANGE_STATUS       0x14   /* 测距结果（16 位，mm） */
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID   0xC0   /* 设备型号 ID，应为 0xEE */

HAL_StatusTypeDef VL53L0X_Init(I2C_HandleTypeDef *hi2c);
uint16_t         VL53L0X_ReadRangeSingle(void);   /* 单次测距，返回 mm（失败返回 0xFFFF） */

#endif /* __VL53L0X_H */
