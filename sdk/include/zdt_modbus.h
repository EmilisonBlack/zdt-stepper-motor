/**
 * @file    zdt_modbus.h
 * @brief   ZDT_X_V2 闭环步进电机 Modbus-RTU 驱动SDK
 * @date    2026-07-21
 * 
 * 协议: 标准 Modbus-RTU, 115200 8N1, CRC16
 * 功能码: 0x04=读输入寄存器, 0x06=写单寄存器, 0x10=写多寄存器
 */

#ifndef ZDT_MODBUS_H
#define ZDT_MODBUS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZDT_DEFAULT_ADDR     0x01
#define ZDT_RX_BUF_SIZE      256
#define ZDT_RX_TIMEOUT_MS    200

/* 方向 */
#define ZDT_DIR_CW   0
#define ZDT_DIR_CCW  1

/* 控制模式 (写入 0x00E0) */
#define ZDT_MODE_PULSE   0
#define ZDT_MODE_SPEED   1
#define ZDT_MODE_POS     2

/* 使能状态 (写入 0x00EA reg3) */
#define ZDT_EN_OFF    0
#define ZDT_EN_ON     1
#define ZDT_EN_HOLD   2

/* 回零模式 (写入 0x00DA) */
#define ZDT_HOME_NEAREST   0
#define ZDT_HOME_DIR       1
#define ZDT_HOME_COLLISION 2
#define ZDT_HOME_LIMIT_SW  3

/* ======================================================================== *
 *  平台抽象层
 * ======================================================================== */
extern int  zdt_platform_uart_send(const uint8_t *data, uint16_t len);
extern int  zdt_platform_uart_recv(uint8_t *buf, uint16_t len, uint32_t timeout_ms);
extern void zdt_platform_delay(uint32_t ms);

/* ======================================================================== *
 *  CRC16 / 收发
 * ======================================================================== */
uint16_t zdt_CRC16(const uint8_t *data, uint16_t len);
int zdt_Transmit(uint8_t *tx, uint8_t tx_len, uint8_t *rx, uint8_t *rx_len);

/* ======================================================================== *
 *  基础Modbus操作
 * ======================================================================== */
int zdt_ReadInputRegs(uint16_t reg, uint8_t cnt, uint8_t *rx, uint8_t *rx_len);
int zdt_WriteSingleReg(uint16_t reg, uint16_t val);
int zdt_WriteMultipleRegs(uint16_t reg, uint8_t cnt, const uint8_t *data);

/* ======================================================================== *
 *  读取参数
 * ======================================================================== */
int zdt_ReadVersion(uint8_t *ver);           /* 0x0010, 2 regs */
int zdt_ReadBusVoltage(uint32_t *mv);        /* 0x0024, 1 reg */
int zdt_ReadStatusFlags(uint16_t *flags);    /* 0x0050, 1 reg */
int zdt_ReadSpeed(int16_t *rpm);             /* 0x0034, 1 reg */
int zdt_ReadPosition(int32_t *deg_x10);      /* 0x0030, 3 regs */

/* ======================================================================== *
 *  电机控制
 * ======================================================================== */
int zdt_SetCtrlMode(uint16_t mode);          /* 0x10, 0x00E0 */
int zdt_Enable(uint16_t state, uint16_t speed_rpm10, uint16_t accel); /* 0x10, 0x00EA */
int zdt_Stop(void);                          /* 0x10, 0x00FE */
int zdt_SpeedMode(uint16_t accel, uint32_t speed_rpm10, uint8_t dir); /* 0x10, 0x00E6 */
int zdt_PositionMode(uint16_t accel, uint32_t speed_rpm10,
                     int32_t position, bool absF); /* 0x10, 0x00F0 */

int zdt_CalibrateEncoder(void);              /* 0x06, 0x0006 */
int zdt_ResetPosition(void);                 /* 0x06, 0x000A */
int zdt_ReleaseStall(void);                  /* 0x06, 0x000E */

int zdt_TriggerHome(uint16_t mode);          /* 0x10, 0x00DA */
int zdt_AbortHome(void);                     /* 0x10, 0x00DC */
int zdt_SetHomeZero(void);                   /* 0x10, 0x00D8 */

#ifdef __cplusplus
}
#endif

#endif /* ZDT_MODBUS_H */
