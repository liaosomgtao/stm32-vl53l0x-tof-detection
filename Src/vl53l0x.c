/**
  ******************************************************************************
  * @file    vl53l0x.c
  * @brief   VL53L0X TOF 激光测距传感器驱动（STM32 HAL，I2C）
  *
  *   - I2C 寄存器读写封装（带超时）
  *   - 器件初始化（2.8V 模式参考配置）
  *   - 单次测距与结果读取
  ******************************************************************************
  */
#include "vl53l0x.h"

/* 本驱动的 I2C 句柄（由 main.c 中初始化后传入并保存） */
static I2C_HandleTypeDef *pI2c = NULL;

/* I2C 操作超时 */
#define VL53L0X_TIMEOUT  100U

/* ---------- I2C 底层读写封装 ---------- */

static HAL_StatusTypeDef VL53L0X_WriteReg(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(pI2c, VL53L0X_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, VL53L0X_TIMEOUT);
}

static HAL_StatusTypeDef VL53L0X_ReadReg(uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(pI2c, VL53L0X_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, val, 1, VL53L0X_TIMEOUT);
}

static uint8_t VL53L0X_ReadRegU8(uint8_t reg)
{
    uint8_t val = 0;
    if (VL53L0X_ReadReg(reg, &val) != HAL_OK) {
        return 0;
    }
    return val;
}

/* ---------- 初始化 ---------- */

/**
  * @brief  初始化 VL53L0X
  * @note   采用 2.8V 模式的标准参考配置（对应 ST 官方 API 的
  *         VL53L0X_DataInit + StaticInit 参考设置）。核心寄存器：
  *         使能设备 -> 写参考设置 -> 校验型号 ID。
  */
HAL_StatusTypeDef VL53L0X_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t model = 0;

    pI2c = hi2c;
    HAL_Delay(10);

    /* 使能设备（解除复位/关断状态） */
    VL53L0X_WriteReg(0x80, 0x01);
    VL53L0X_WriteReg(0xFF, 0x01);
    VL53L0X_WriteReg(0x00, 0x00);
    VL53L0X_WriteReg(0x91, 0x00);   /* 停止变量默认 */
    VL53L0X_WriteReg(0x00, 0x01);
    VL53L0X_WriteReg(0xFF, 0x00);
    VL53L0X_WriteReg(0x80, 0x00);

    /* 2.8V 模式参考配置（关键寄存器，完整序列见 ST 官方 API） */
    VL53L0X_WriteReg(0x30, 0x09);   /* 电源设置 */
    VL53L0X_WriteReg(0x31, 0x04);
    VL53L0X_WriteReg(0x32, 0x03);
    VL53L0X_WriteReg(0x40, 0x83);   /* 量程/精度相关 */
    VL53L0X_WriteReg(0x46, 0x25);
    VL53L0X_WriteReg(0x60, 0x00);
    VL53L0X_WriteReg(0x27, 0x00);
    VL53L0X_WriteReg(0x50, 0x06);
    VL53L0X_WriteReg(0x51, 0x00);
    VL53L0X_WriteReg(0x52, 0x96);   /* 默认测距预算 */
    VL53L0X_WriteReg(0x56, 0x08);
    VL53L0X_WriteReg(0x57, 0x30);
    VL53L0X_WriteReg(0x61, 0x00);
    VL53L0X_WriteReg(0x62, 0x00);
    VL53L0X_WriteReg(0x64, 0x00);
    VL53L0X_WriteReg(0x65, 0x00);
    VL53L0X_WriteReg(0x66, 0xA0);

    VL53L0X_WriteReg(0xFF, 0x01);
    VL53L0X_WriteReg(0x22, 0x32);
    VL53L0X_WriteReg(0x47, 0x14);
    VL53L0X_WriteReg(0x49, 0xFF);
    VL53L0X_WriteReg(0x4A, 0x00);
    VL53L0X_WriteReg(0xFF, 0x00);
    VL53L0X_WriteReg(0x7A, 0x0A);
    VL53L0X_WriteReg(0x7B, 0x00);
    VL53L0X_WriteReg(0x78, 0x21);

    VL53L0X_WriteReg(0xFF, 0x01);
    VL53L0X_WriteReg(0x23, 0x34);
    VL53L0X_WriteReg(0x42, 0x00);
    VL53L0X_WriteReg(0x44, 0xFF);
    VL53L0X_WriteReg(0x45, 0x26);
    VL53L0X_WriteReg(0x46, 0x05);
    VL53L0X_WriteReg(0x40, 0x40);
    VL53L0X_WriteReg(0x0E, 0x06);
    VL53L0X_WriteReg(0x20, 0x1A);
    VL53L0X_WriteReg(0x43, 0x40);
    VL53L0X_WriteReg(0xFF, 0x00);
    VL53L0X_WriteReg(0x34, 0x03);
    VL53L0X_WriteReg(0x35, 0x44);

    VL53L0X_WriteReg(0xFF, 0x01);
    VL53L0X_WriteReg(0x31, 0x04);
    VL53L0X_WriteReg(0x4B, 0x09);
    VL53L0X_WriteReg(0x4C, 0x05);
    VL53L0X_WriteReg(0x4D, 0x04);
    VL53L0X_WriteReg(0xFF, 0x00);
    VL53L0X_WriteReg(0x44, 0x00);
    VL53L0X_WriteReg(0x45, 0x20);
    VL53L0X_WriteReg(0x47, 0x08);
    VL53L0X_WriteReg(0x48, 0x28);
    VL53L0X_WriteReg(0x67, 0x00);
    VL53L0X_WriteReg(0x70, 0x04);
    VL53L0X_WriteReg(0x71, 0x01);
    VL53L0X_WriteReg(0x72, 0xFE);
    VL53L0X_WriteReg(0x76, 0x00);
    VL53L0X_WriteReg(0x77, 0x00);

    VL53L0X_WriteReg(0xFF, 0x01);
    VL53L0X_WriteReg(0x0D, 0x01);
    VL53L0X_WriteReg(0xFF, 0x00);
    VL53L0X_WriteReg(0x80, 0x01);
    VL53L0X_WriteReg(0x01, 0xF8);
    VL53L0X_WriteReg(0xFF, 0x01);
    VL53L0X_WriteReg(0x8E, 0x01);
    VL53L0X_WriteReg(0x00, 0x01);
    VL53L0X_WriteReg(0xFF, 0x00);
    VL53L0X_WriteReg(0x80, 0x00);

    /* 校验设备型号 ID（应为 0xEE） */
    VL53L0X_ReadReg(VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model);
    if (model != 0xEE) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* ---------- 测距 ---------- */

/**
  * @brief  单次测距，返回距离（mm）
  * @note   失败返回 0xFFFF
  */
uint16_t VL53L0X_ReadRangeSingle(void)
{
    uint8_t status;
    uint8_t lo, hi;

    if (pI2c == NULL) {
        return 0xFFFF;
    }

    /* 开始单次测距 */
    VL53L0X_WriteReg(VL53L0X_REG_SYSRANGE_START, 0x01);

    /* 等待测量完成：RESULT_INTERRUPT_STATUS 低位就绪 */
    {
        uint32_t timeout = 100000U;
        do {
            status = VL53L0X_ReadRegU8(VL53L0X_REG_RESULT_INTERRUPT_STATUS);
            if (--timeout == 0U) {
                return 0xFFFF;
            }
        } while ((status & 0x07) == 0U);
    }

    /* 读取 16 位测距结果：0x14 低字节、0x15 高字节 */
    lo = VL53L0X_ReadRegU8(VL53L0X_REG_RESULT_RANGE_STATUS);
    hi = VL53L0X_ReadRegU8(VL53L0X_REG_RESULT_RANGE_STATUS + 1);

    /* 清除中断标志 */
    VL53L0X_WriteReg(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    return (uint16_t)(((uint16_t)hi << 8) | lo);
}
