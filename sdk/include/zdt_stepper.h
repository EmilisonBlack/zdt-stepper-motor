/**
 * @file    zdt_stepper.h
 * @brief   ZDT_X_V2 闭环步进电机驱动底层SDK (自定义串口协议)
 * @date    2026-07-22
 * 
 * 【通信协议】
 *   帧格式: [地址] [功能码] [参数...] [校验字节]
 *   波特率: 115200, 8N1
 *   校验: 默认固定0x6B, 可选XOR/CRC-8 (由驱动器OLED菜单设置)
 *
 * 【DLL导出】
 *   编译DLL时定义 ZDT_BUILD_DLL, 否则为静态库或直接包含
 */

#ifndef ZDT_STEPPER_H
#define ZDT_STEPPER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DLL 导出/导入宏 */
#ifdef ZDT_BUILD_DLL
#  define ZDT_API __declspec(dllexport)
#elif defined(ZDT_USE_DLL)
#  define ZDT_API __declspec(dllimport)
#else
#  define ZDT_API
#endif

#define ZDT_DEFAULT_ADDR     0x01
#define ZDT_RX_BUF_SIZE      128
#define ZDT_RX_TIMEOUT_MS    200

/* 方向 */
#define ZDT_DIR_CW   0
#define ZDT_DIR_CCW  1

/* 状态标志位 */
#define ZDT_FLAG_ENABLE   0x01
#define ZDT_FLAG_IN_POS   0x02
#define ZDT_FLAG_STALL    0x04

/* 控制模式 */
#define ZDT_MODE_PULSE_OFF   0
#define ZDT_MODE_OPENLOOP    1
#define ZDT_MODE_CLOSEDLOOP  2
#define ZDT_MODE_LIMIT_SW    3

/* 回零模式 */
#define ZDT_HOME_NEAREST   0
#define ZDT_HOME_DIR       1
#define ZDT_HOME_COLLISION 2
#define ZDT_HOME_LIMIT_SW  3

/* ======================================================================== *
 *  平台抽象层 (Windows DLL 内置, 其他平台需用户实现)
 * ======================================================================== */
ZDT_API int  zdt_platform_uart_send(const uint8_t *data, uint16_t len);
ZDT_API int  zdt_platform_uart_recv(uint8_t *buf, uint16_t len, uint32_t timeout_ms);
ZDT_API void zdt_platform_delay(uint32_t ms);

/* Windows 平台专用: 打开/关闭串口 */
ZDT_API int  platform_uart_open(const char *port, int baudrate);
ZDT_API void platform_uart_close(void);

/* ======================================================================== *
 *  初始化
 * ======================================================================== */
ZDT_API void    zdt_Init(uint8_t addr);
ZDT_API void    zdt_SetAddr(uint8_t addr);
ZDT_API uint8_t zdt_GetAddr(void);

/* ======================================================================== *
 *  底层收发
 * ======================================================================== */
ZDT_API int zdt_Transmit(uint8_t *tx, uint8_t tx_len, uint8_t *rx, uint8_t *rx_len);

/* ======================================================================== *
 *  读取系统参数
 * ======================================================================== */
ZDT_API int zdt_ReadVersion(uint8_t *ver);
ZDT_API int zdt_ReadMotorParams(uint16_t *res_x10, uint16_t *ind_x10);
ZDT_API int zdt_ReadBusVoltage(uint32_t *mv);
ZDT_API int zdt_ReadPhaseCurrent(uint32_t *ma);
ZDT_API int zdt_ReadHallAngle(uint16_t *raw);
ZDT_API int zdt_ReadEncoder(int32_t *val);
ZDT_API int zdt_ReadSpeed(int16_t *rpm);
ZDT_API int zdt_ReadPosition(int32_t *deg_x10);
ZDT_API int zdt_ReadTargetPosition(int32_t *deg_x10);
ZDT_API int zdt_ReadAbsolutePosition(int32_t *deg_x10);
ZDT_API int zdt_ReadPositionError(int32_t *deg_x10);
ZDT_API int zdt_ReadActualCurrent(uint32_t *ma);
ZDT_API int zdt_ReadStatusFlags(uint8_t *flags);
ZDT_API int zdt_ReadHomeStatus(uint8_t *status);
ZDT_API int zdt_ReadTemperature(int16_t *deg_c);

/* ======================================================================== *
 *  电机控制
 * ======================================================================== */
ZDT_API int zdt_Enable(bool enable, bool sync);
ZDT_API int zdt_SpeedMode(uint8_t dir, uint16_t rpm, uint16_t accel, bool sync);
ZDT_API int zdt_PositionModeRel(uint8_t dir, uint16_t rpm, uint16_t accel, uint32_t pulses, bool sync);
ZDT_API int zdt_PositionModeAbs(uint8_t dir, uint16_t rpm, uint16_t accel, uint32_t pulses, bool sync);
ZDT_API int zdt_Stop(bool sync);
ZDT_API int zdt_SyncMotion(void);
ZDT_API int zdt_TorqueMode(uint8_t dir, uint16_t accel_ma_s, uint16_t current_ma, bool sync);

/* ======================================================================== *
 *  模式切换
 * ======================================================================== */
ZDT_API int zdt_SetCtrlMode(bool save, uint8_t mode);

/* ======================================================================== *
 *  回零
 * ======================================================================== */
ZDT_API int zdt_SetHomeZero(bool save);
ZDT_API int zdt_TriggerHome(uint8_t mode, bool sync);
ZDT_API int zdt_AbortHome(void);

/* ======================================================================== *
 *  工具函数
 * ======================================================================== */
ZDT_API int zdt_ResetPosition(void);
ZDT_API int zdt_ReleaseStall(void);
ZDT_API int zdt_FactoryReset(void);
ZDT_API int zdt_CalibrateEncoder(void);

/* ======================================================================== *
 *  高级配置
 * ======================================================================== */
ZDT_API int zdt_SetHomeParams(uint8_t mode, uint8_t dir, uint16_t vel_rpm,
                      uint32_t timeout_ms, uint16_t stall_vel,
                      uint16_t stall_ma, uint16_t stall_ms, bool save);
ZDT_API int zdt_SetAddrID(uint8_t new_addr, bool save);
ZDT_API int zdt_SetMicrostep(uint16_t step, bool save);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_STEPPER_H */
