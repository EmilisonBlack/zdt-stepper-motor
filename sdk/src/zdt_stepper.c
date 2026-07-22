/**
 * @file    zdt_stepper.c
 * @brief   ZDT/Emm_V5.0 闭环步进电机驱动底层SDK (自定义串口协议)
 * @date    2026-07-21
 * 
 * 已验证通信成功的指令:
 *   01 1F 6B → 读版本 (返回 01 1F 00 CC 00 78 6B)
 *   01 24 6B → 读总线电压 (返回 01 24 2B 5B 6B)
 *   01 36 6B → 读实时位置 (返回 01 36 01 00 00 00 00 6B)
 *   01 3A 6B → 读状态标志 (返回 01 3A 03 6B)
 */

#include "zdt_stepper.h"
#include <string.h>

static uint8_t g_addr = ZDT_DEFAULT_ADDR;

/* ======================================================================== *
 *  初始化
 * ======================================================================== */
void    zdt_Init(uint8_t addr)         { g_addr = addr; }
void    zdt_SetAddr(uint8_t addr)      { g_addr = addr; }
uint8_t zdt_GetAddr(void)              { return g_addr; }

/* ======================================================================== *
 *  底层收发
 * ======================================================================== */

static int read_serial(uint8_t *buf, uint16_t maxlen)
{
    return zdt_platform_uart_recv(buf, maxlen, ZDT_RX_TIMEOUT_MS);
}

int zdt_Transmit(uint8_t *tx, uint8_t tx_len, uint8_t *rx, uint8_t *rx_len)
{
    /* 发送 */
    if (zdt_platform_uart_send(tx, tx_len) != tx_len)
        return -1;

    /* 接收 */
    int n = read_serial(rx, ZDT_RX_BUF_SIZE);
    if (n < 3) return -1;
    *rx_len = (uint8_t)n;

    /* 验证地址 && 校验字节(0x6B) */
    if (rx[0] != g_addr) return -2;
    if (rx[n-1] != 0x6B) return -3;

    return 0;
}

/* ======================================================================== *
 *  内部辅助: 发送读参数命令
 *  格式: [地址] [功能码] [校验]
 *  用于: 0x1F, 0x20, 0x24, 0x26, 0x27, 0x29, 0x31,
 *        0x33, 0x34, 0x35, 0x36, 0x37, 0x3A, 0x3B
 * ======================================================================== */

static int cmd_read(uint8_t func, uint8_t aux, uint8_t *rx, uint8_t *rx_len)
{
    uint8_t cmd[4];
    uint8_t i = 0;
    cmd[i++] = g_addr;
    cmd[i++] = func;
    if (aux) cmd[i++] = aux;
    cmd[i++] = 0x6B;
    return zdt_Transmit(cmd, i, rx, rx_len);
}

/* 提取 big-endian 数据 */
#define RX_U16(buf, off) ((uint16_t)(buf)[off]<<8|(buf)[off+1])
#define RX_U32(buf, off) ((uint32_t)(buf)[off]<<24|(uint32_t)(buf)[off+1]<<16|\
                          (uint32_t)(buf)[off+2]<<8|(uint32_t)(buf)[off+3])

/*
 * 通用读位置类命令解析 (0x33, 0x34, 0x36, 0x37)
 * 响应格式: [addr][func][dir(1B)][val32 big-endian(4B)][0x6B]
 * 0x36/0x33/0x34: 单位 0.1°, 返回值 = val/10 °
 * 0x37:           单位 0.01°, 返回值 = val/100 °
 */
static int parse_pos_dir(const uint8_t *rx, uint8_t len, int32_t *out, int32_t divisor)
{
    if (len < 8) return -4;
    int32_t sign = (rx[2] != 0) ? -1 : 1;
    *out = (int32_t)RX_U32(rx, 3) / divisor * sign;
    return 0;
}

/* ======================================================================== *
 *  读取系统参数
 * ======================================================================== */

int zdt_ReadVersion(uint8_t *ver)
{
    uint8_t rx[32], len;
    int ret = cmd_read(0x1F, 0, rx, &len);
    if (ret) return ret;
    /* 响应: [addr][1F][状态(1B)][ASCII版本字符串...][6B] */
    if (len < 5) return -4;
    uint8_t vlen = len - 4;  /* 去掉 addr(1)+func(1)+status(1)+6B(1)=4 */
    if (vlen > 15) vlen = 15;
    memcpy(ver, &rx[3], vlen);  /* 从 offset 3 开始 (跳过状态字节) */
    ver[vlen] = 0;
    return 0;
}

int zdt_ReadMotorParams(uint16_t *res_x10, uint16_t *ind_x10)
{
    uint8_t rx[16], len;
    int ret = cmd_read(0x20, 0, rx, &len);
    if (ret) return ret;
    /* 响应: [addr][20][res_H][res_L][ind_H][ind_L][6B] */
    if (len >= 6) {
        *res_x10 = RX_U16(rx, 2);
        *ind_x10 = RX_U16(rx, 4);
    }
    return 0;
}

int zdt_ReadBusVoltage(uint32_t *mv)
{
    uint8_t rx[16], len;
    int ret = cmd_read(0x24, 0, rx, &len);
    if (ret) return ret;
    /* 响应: [addr][24][mV_H][mV_L][6B] */
    if (len >= 5) *mv = RX_U16(rx, 2);
    return 0;
}

int zdt_ReadPhaseCurrent(uint32_t *ma)
{
    uint8_t rx[16], len;
    int ret = cmd_read(0x27, 0, rx, &len);
    if (ret) return ret;
    /* 响应: [addr][27][mA_H][mA_L][6B] */
    if (len >= 5) *ma = RX_U16(rx, 2);
    return 0;
}

int zdt_ReadHallAngle(uint16_t *raw)
{
    uint8_t rx[16], len;
    int ret = cmd_read(0x29, 0, rx, &len);
    if (ret) return ret;
    /* 响应: [addr][29][val_H][val_L][6B] — 0~16383对应0~360° */
    if (len >= 5) *raw = RX_U16(rx, 2);
    return 0;
}

int zdt_ReadEncoder(int32_t *val)
{
    uint8_t rx[16], len;
    int ret = cmd_read(0x31, 0, rx, &len);
    if (ret) return ret;
    /* 响应: [addr][31][val_H][val_L][6B] — 16位, 0~65535 */
    if (len >= 5) *val = RX_U16(rx, 2);
    return 0;
}

/*
 * ZDT_X V2 转速:
 *   响应: [addr][35][dir][spd×10_H][spd×10_L][6B]  (6字节)
 *   dir=0 正转, dir≠0 反转
 *   值÷10 得 RPM
 */
int zdt_ReadSpeed(int16_t *rpm)
{
    uint8_t rx[16], len;
    int ret = cmd_read(0x35, 0, rx, &len);
    if (ret) return ret;
    if (len >= 6) {
        int16_t val = (int16_t)RX_U16(rx, 3);  /* 无符号转速值 */
        val = val / 10;                          /* 转为 RPM */
        if (rx[2] != 0) val = -val;             /* 方向 */
        *rpm = val;
    }
    return 0;
}

/* ZDT_X V2 位置: 响应 [addr][36][dir][pos32÷10 big-endian][6B] — 8字节 */
int zdt_ReadPosition(int32_t *deg_x10)
{
    uint8_t rx[16], len;
    int ret = cmd_read(0x36, 0, rx, &len);
    if (ret) return ret;
    return parse_pos_dir(rx, len, deg_x10, 1);  /* 单位 0.1°, 无需除 */
}

int zdt_ReadTargetPosition(int32_t *deg_x10)
{
    uint8_t rx[16], len;
    int ret = cmd_read(0x33, 0, rx, &len);
    if (ret) return ret;
    return parse_pos_dir(rx, len, deg_x10, 1);  /* 同 0x36, 单位 0.1° */
}

int zdt_ReadAbsolutePosition(int32_t *deg_x10)
{
    uint8_t rx[16], len;
    int ret = cmd_read(0x34, 0, rx, &len);
    if (ret) return ret;
    return parse_pos_dir(rx, len, deg_x10, 1);  /* 同 0x36, 单位 0.1° */
}

/*
 * ZDT_X V2 位置误差:
 *   响应: [addr][37][dir][err32÷100 big-endian][6B] — 8字节
 *   单位 0.01° (与 0x36 的 0.1° 不同!)
 */
int zdt_ReadPositionError(int32_t *deg_x10)
{
    uint8_t rx[16], len;
    int ret = cmd_read(0x37, 0, rx, &len);
    if (ret) return ret;
    return parse_pos_dir(rx, len, deg_x10, 10);  /* ÷100 再 ÷? 实际上原始值÷100 得 ° */
    /* 例: 0x08=8 → 8/100=0.08° */
}

int zdt_ReadStatusFlags(uint8_t *flags)
{
    uint8_t rx[16], len;
    int ret = cmd_read(0x3A, 0, rx, &len);
    if (ret) return ret;
    if (len >= 4) *flags = rx[2];
    return 0;
}

int zdt_ReadHomeStatus(uint8_t *status)
{
    uint8_t rx[16], len;
    int ret = cmd_read(0x3B, 0, rx, &len);
    if (ret) return ret;
    if (len >= 4) *status = rx[2];
    return 0;
}

/* 0x26 = 实际相电流 (非温度!) 单位 mA */
int zdt_ReadActualCurrent(uint32_t *ma)
{
    uint8_t rx[16], len;
    int ret = cmd_read(0x26, 0, rx, &len);
    if (ret) return ret;
    if (len >= 5) *ma = RX_U16(rx, 2);
    return 0;
}

/* 0x39 = 电机/驱动器温度, 单位 ℃ (已验证: 空闲约 31℃) */
int zdt_ReadTemperature(int16_t *deg_c)
{
    uint8_t rx[16], len;
    int ret = cmd_read(0x39, 0, rx, &len);
    if (ret) return ret;
    if (len >= 5) *deg_c = (int16_t)RX_U16(rx, 2);
    return 0;
}

/* ======================================================================== *
 *  内部辅助: 发送写命令
 *  格式: [地址] [功能码] [参数...] [辅助码/数据...] [6B]
 * ======================================================================== */

static int cmd_write(const uint8_t *body, uint8_t body_len)
{
    uint8_t cmd[32];
    memcpy(cmd, body, body_len);
    cmd[body_len] = 0x6B;
    uint8_t rx[16], rlen;
    return zdt_Transmit(cmd, body_len + 1, rx, &rlen);
}

/* ======================================================================== *
 *  电机控制
 * ======================================================================== */

int zdt_Enable(bool enable, bool sync)
{
    uint8_t cmd[] = { g_addr, 0xF3, 0xAB, enable?1:0, sync?1:0 };
    return cmd_write(cmd, 5);
}

/*
 * ZDT_X V2 速度模式:
 *   [addr][0xF6][dir][accel16_H][accel16_L][speed×10_H][speed×10_L][sync][0x6B]
 *   accel 单位 0.1RPM/s (例: 100 = 10.0 RPM/s)
 *   speed 单位 RPM (内部×10 转为 0.1RPM)
 */
int zdt_SpeedMode(uint8_t dir, uint16_t rpm, uint16_t accel, bool sync)
{
    uint16_t speed10 = rpm * 10;
    uint8_t cmd[] = { g_addr, 0xF6, dir,
                      (uint8_t)(accel>>8), (uint8_t)accel,
                      (uint8_t)(speed10>>8), (uint8_t)speed10,
                      sync?1:0 };
    return cmd_write(cmd, 8);
}

/*
 * ZDT_X V2 位置模式 (0xFD):
 *   [addr][FD][dir][accel1_H][accel1_L][accel2_H][accel2_L]
 *   [speed×10_H][speed×10_L][pos×10_32big][abs][sync][0x6B]
 *   共 15+1=16 字节
 */
static int pos_mode(uint8_t dir, uint16_t rpm, uint16_t accel,
                    uint32_t pos_x10, bool absF, bool sync)
{
    uint16_t speed10 = rpm * 10;
    uint8_t cmd[15];
    cmd[0]  = g_addr;
    cmd[1]  = 0xFD;
    cmd[2]  = dir;
    cmd[3]  = (uint8_t)(accel>>8);   cmd[4]  = (uint8_t)accel;    /* accel1 */
    cmd[5]  = (uint8_t)(accel>>8);   cmd[6]  = (uint8_t)accel;    /* accel2 */
    cmd[7]  = (uint8_t)(speed10>>8); cmd[8]  = (uint8_t)speed10;  /* speed×10 */
    cmd[9]  = (uint8_t)(pos_x10>>24);cmd[10] = (uint8_t)(pos_x10>>16);
    cmd[11] = (uint8_t)(pos_x10>>8); cmd[12] = (uint8_t)pos_x10;
    cmd[13] = absF ? 0x01 : 0x00;
    cmd[14] = sync ? 0x01 : 0x00;
    return cmd_write(cmd, 15);
}

int zdt_PositionModeRel(uint8_t dir, uint16_t rpm, uint16_t accel,
                        uint32_t pulses, bool sync)
{
    return pos_mode(dir, rpm, accel, pulses, false, sync);
}

int zdt_PositionModeAbs(uint8_t dir, uint16_t rpm, uint16_t accel,
                        uint32_t pulses, bool sync)
{
    return pos_mode(dir, rpm, accel, pulses, true, sync);
}

/*
 * 力矩模式 (0xF5):
 *   [addr][F5][dir][accel_H][accel_L][current_H][current_L][sync][0x6B]
 *   accel 单位 Ma/s, current 单位 mA
 */
int zdt_TorqueMode(uint8_t dir, uint16_t accel_ma_s, uint16_t current_ma, bool sync)
{
    uint8_t cmd[] = { g_addr, 0xF5, dir,
                      (uint8_t)(accel_ma_s>>8), (uint8_t)accel_ma_s,
                      (uint8_t)(current_ma>>8), (uint8_t)current_ma,
                      sync?1:0 };
    return cmd_write(cmd, 8);
}

int zdt_Stop(bool sync)
{
    uint8_t cmd[] = { g_addr, 0xFE, 0x98, sync?1:0 };
    return cmd_write(cmd, 4);
}

int zdt_SyncMotion(void)
{
    uint8_t cmd[] = { g_addr, 0xFF, 0x66 };
    return cmd_write(cmd, 3);
}

/* ======================================================================== *
 *  模式切换
 * ======================================================================== */

int zdt_SetCtrlMode(bool save, uint8_t mode)
{
    uint8_t cmd[] = { g_addr, 0x46, 0x69, save?1:0, mode };
    return cmd_write(cmd, 5);
}

/* ======================================================================== *
 *  回零功能
 * ======================================================================== */

int zdt_SetHomeZero(bool save)
{
    uint8_t cmd[] = { g_addr, 0x93, 0x88, save?1:0 };
    return cmd_write(cmd, 4);
}

int zdt_TriggerHome(uint8_t mode, bool sync)
{
    uint8_t cmd[] = { g_addr, 0x9A, mode, sync?1:0 };
    return cmd_write(cmd, 4);
}

int zdt_AbortHome(void)
{
    uint8_t cmd[] = { g_addr, 0x9C, 0x48 };
    return cmd_write(cmd, 3);
}

/* ======================================================================== *
 *  工具函数
 * ======================================================================== */

int zdt_ResetPosition(void)
    { uint8_t c[]={g_addr,0x0A,0x6D}; return cmd_write(c,3); }

int zdt_ReleaseStall(void)
    { uint8_t c[]={g_addr,0x0E,0x52}; return cmd_write(c,3); }

/* 手册: 0x0F + 0x5F (不是 0x42) */
int zdt_FactoryReset(void)
    { uint8_t c[]={g_addr,0x0F,0x5F}; return cmd_write(c,3); }

/* 手册: 0x06 + 0x45 (不是 0x6C) */
int zdt_CalibrateEncoder(void)
    { uint8_t c[]={g_addr,0x06,0x45}; return cmd_write(c,3); }

/* ======================================================================== *
 *  高级配置
 * ======================================================================== */

int zdt_SetHomeParams(uint8_t mode, uint8_t dir, uint16_t vel_rpm,
                      uint32_t timeout_ms, uint16_t stall_vel,
                      uint16_t stall_ma, uint16_t stall_ms, bool save)
{
    uint8_t cmd[20];
    cmd[0]=g_addr; cmd[1]=0x4C; cmd[2]=0xAE;
    cmd[3]=save?1:0; cmd[4]=mode; cmd[5]=dir;
    cmd[6]=(uint8_t)(vel_rpm>>8);    cmd[7]=(uint8_t)vel_rpm;
    cmd[8]=(uint8_t)(timeout_ms>>24); cmd[9]=(uint8_t)(timeout_ms>>16);
    cmd[10]=(uint8_t)(timeout_ms>>8); cmd[11]=(uint8_t)timeout_ms;
    cmd[12]=(uint8_t)(stall_vel>>8);  cmd[13]=(uint8_t)stall_vel;
    cmd[14]=(uint8_t)(stall_ma>>8);   cmd[15]=(uint8_t)stall_ma;
    cmd[16]=(uint8_t)(stall_ms>>8);   cmd[17]=(uint8_t)stall_ms;
    cmd[18]=0;  /* potF */
    return cmd_write(cmd, 19);
}

/* 手册: 0xAE + 0x4B (不是 0x6A) */
int zdt_SetAddrID(uint8_t new_addr, bool save)
{
    uint8_t cmd[] = { g_addr, 0xAE, 0x4B, save?1:0, new_addr };
    return cmd_write(cmd, 5);
}

/* 手册: 0x84 + 0x8A (不是 0x88) */
int zdt_SetMicrostep(uint16_t step, bool save)
{
    uint8_t cmd[] = { g_addr, 0x84, 0x8A, save?1:0,
                      (uint8_t)(step>>8), (uint8_t)step };
    return cmd_write(cmd, 6);
}
